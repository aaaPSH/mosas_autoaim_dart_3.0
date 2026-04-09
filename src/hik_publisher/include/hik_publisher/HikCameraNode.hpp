#include <algorithm>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <thread>
#include <vector>

#include "MvCameraControl.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

class HikCameraNode : public rclcpp::Node
{
private:
  // --- Raw 数据发布器 (给算法) ---
  sensor_msgs::msg::Image raw_msg_;
  image_transport::CameraPublisher raw_pub_;

  // --- RGB 数据发布器 (给人类/可视化) ---
  sensor_msgs::msg::Image rgb_msg_;
  image_transport::CameraPublisher rgb_pub_;

  sensor_msgs::msg::CameraInfo camera_info_msg_;

  std::array<double, 9> camera_matrix_;
  std::vector<double> distortion_coeff_;
  std::string distortion_model_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

private:
  std::thread capture_thread_;
  int nRet = MV_OK;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM ConvertParam_;
  void * camera_handle_;

  // 内录抽帧
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr record_pub_;

public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode (Dual Stream: Raw + RGB)!");

    // 1. 枚举与打开设备
    MV_CC_DEVICE_INFO_LIST DeviceList;
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &DeviceList);
    while (DeviceList.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found! Retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &DeviceList);
    }
    MV_CC_CreateHandle(&camera_handle_, DeviceList.pDeviceInfo[0]);
    MV_CC_OpenDevice(camera_handle_);

    // 2. 【关键】强制设置输出格式为 BayerRG8
    // 这是双流的基础：相机必须吐出 Raw，我们才能同时拥有 Raw 和转换后的 RGB
    int tempRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);
    if (tempRet != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(),
        "Warning: Failed to set PixelFormat to BayerRG8. Dual stream might fail.");
    }

    MV_CC_GetImageInfo(camera_handle_, &img_info_);

    // 3. 初始化转换参数 (用于生成 /image_color)
    ConvertParam_.nWidth = img_info_.nWidthMax;
    ConvertParam_.nHeight = img_info_.nHeightMax;
    // 目标格式：RGB8
    ConvertParam_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    // 4. 初始化发布者
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;

    // 话题 1: /image_raw (发布 Bayer Raw，给算法用)
    raw_pub_ = image_transport::create_camera_publisher(this, "/image_raw", qos);

    // 话题 2: /image_color (发布 RGB，给 RQT 用)
    rgb_pub_ = image_transport::create_camera_publisher(this, "/image_color", qos);

    record_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>("/save_frame", rclcpp::SensorDataQoS());

    declareParams();
    MV_CC_StartGrabbing(camera_handle_);

    // 加载内参 (假设内参对 raw 和 color 都是通用的)
    auto camera_matrix_vector_ = this->declare_parameter<std::vector<double>>(
      "camera_matrix", {2358.125070, 0.000000, 743.213621, 0.000000, 2357.300407, 563.210311,
                        0.000000, 0.000000, 1.000000});
    if (camera_matrix_vector_.size() == 9) {
      std::copy(camera_matrix_vector_.begin(), camera_matrix_vector_.end(), camera_matrix_.begin());
    }

    distortion_model_ = this->declare_parameter("distortion_model", "plumb_bob");
    distortion_coeff_ = this->declare_parameter<std::vector<double>>(
      "distortion_coeff", {-0.083754, 0.222157, 0.000000, 0.000000, 0.109514});

    camera_info_msg_.k = camera_matrix_;
    camera_info_msg_.distortion_model = distortion_model_;
    camera_info_msg_.d = distortion_coeff_;

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread(&HikCameraNode::captureLoop, this);
  }

  ~HikCameraNode()
  {
    if (capture_thread_.joinable()) capture_thread_.join();
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

  void captureLoop()
  {
    MV_FRAME_OUT OutFrame;
    int fail_count = 0;
    int record_fps_control = 0;

    // 预分配内存，避免循环内频繁 malloc
    raw_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax);
    rgb_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);

    MV_CC_StartGrabbing(camera_handle_);

    while (rclcpp::ok()) {
      nRet = MV_CC_GetImageBuffer(camera_handle_, &OutFrame, 1000);

      if (nRet == MV_OK) {
        fail_count = 0;
        rclcpp::Time stamp = this->now();

        // ---------------------------------------------------------
        // 步骤 1: 发布 Raw 图像 (/image_raw) -> 给算法
        // ---------------------------------------------------------
        std::string bayer_encoding = "";
        bool is_bayer = true;

        // 自动判断 Bayer 格式
        switch (OutFrame.stFrameInfo.enPixelType) {
          case PixelType_Gvsp_BayerRG8:
            bayer_encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
            break;
          case PixelType_Gvsp_BayerGB8:
            bayer_encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
            break;
          case PixelType_Gvsp_BayerGR8:
            bayer_encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
            break;
          case PixelType_Gvsp_BayerBG8:
            bayer_encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
            break;
          default:
            is_bayer = false;
            break;
        }

        if (is_bayer) {
          raw_msg_.header.stamp = stamp;
          raw_msg_.header.frame_id = "camera_optical_frame";
          raw_msg_.encoding = bayer_encoding;
          raw_msg_.height = OutFrame.stFrameInfo.nHeight;
          raw_msg_.width = OutFrame.stFrameInfo.nWidth;
          raw_msg_.step = OutFrame.stFrameInfo.nWidth;  // Raw 步长 = 宽

          // 零拷贝 (尽可能)
          raw_msg_.data.resize(OutFrame.stFrameInfo.nFrameLen);
          memcpy(raw_msg_.data.data(), OutFrame.pBufAddr, OutFrame.stFrameInfo.nFrameLen);

          camera_info_msg_.header = raw_msg_.header;
          raw_pub_.publish(raw_msg_, camera_info_msg_);
        }

        // ---------------------------------------------------------
        // 步骤 2: 发布 RGB 图像 (/image_color) -> 给 RQT/人类
        // ---------------------------------------------------------
        // 只有当接收者有订阅时才转换，节省 CPU 资源 (Lazy publish)
        if (rgb_pub_.getNumSubscribers() > 0 || record_pub_->get_subscription_count() > 0) {
          rgb_msg_.header.stamp = stamp;
          rgb_msg_.header.frame_id = "camera_optical_frame";
          rgb_msg_.encoding = "rgb8";
          rgb_msg_.height = OutFrame.stFrameInfo.nHeight;
          rgb_msg_.width = OutFrame.stFrameInfo.nWidth;
          rgb_msg_.step = OutFrame.stFrameInfo.nWidth * 3;
          rgb_msg_.data.resize(rgb_msg_.height * rgb_msg_.step);

          // 填充转换结构体
          ConvertParam_.pDstBuffer = rgb_msg_.data.data();
          ConvertParam_.nDstBufferSize = rgb_msg_.data.size();
          ConvertParam_.pSrcData = OutFrame.pBufAddr;  // 源数据直接来自相机 buffer
          ConvertParam_.nSrcDataLen = OutFrame.stFrameInfo.nFrameLen;
          ConvertParam_.enSrcPixelType = OutFrame.stFrameInfo.enPixelType;

          // 调用 SDK 进行去马赛克转换
          MV_CC_ConvertPixelType(camera_handle_, &ConvertParam_);

          rgb_pub_.publish(rgb_msg_, camera_info_msg_);

          // 内录抽帧 (使用 RGB 图像)
          record_fps_control++;
          if (record_fps_control >= 7) {
            record_pub_->publish(rgb_msg_);
            record_fps_control = 0;
          }
        }

        MV_CC_FreeImageBuffer(camera_handle_, &OutFrame);
      } else {
        // 错误处理...
        fail_count++;
        if (fail_count >= 100) {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! Resetting...");
          MV_CC_StopGrabbing(camera_handle_);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          MV_CC_StartGrabbing(camera_handle_);
          fail_count = 0;
        }
      }
    }
  }

  // 参数声明与回调部分保持不变...
  void declareParams()
  {
    // ... (与你原代码一致)
    // 确保 TriggerMode 开启
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE fValue;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &fValue);
    param_desc.integer_range[0].from_value = fValue.fMin;
    param_desc.integer_range[0].to_value = fValue.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000.0, param_desc);
    nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);

    // Gain
    MV_CC_GetFloatValue(camera_handle_, "Gain", &fValue);
    param_desc.integer_range[0].from_value = fValue.fMin;
    param_desc.integer_range[0].to_value = fValue.fMax;
    double gain = this->declare_parameter("gain", fValue.fCurValue, param_desc);
    nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);

    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
    nRet = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", false);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    // ... (与你原代码一致)
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_double());
      } else if (param.get_name() == "gain") {
        MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
      }
    }
    return result;
  }
};
