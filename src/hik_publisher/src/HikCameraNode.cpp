/**
 * @file HikCameraNode.cpp
 * @brief 海康相机节点实现
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#include "hik_publisher/HikCameraNode.hpp"

#include <cstring>

#include "rclcpp_components/register_node_macro.hpp"

namespace hik_publisher
{

  // ============================================================================
  // 构造函数
  // ============================================================================

  HikCameraNode::HikCameraNode(const rclcpp::NodeOptions &options)
      : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    // 1. 枚举并打开设备
    MV_CC_DEVICE_INFO_LIST device_list;
    last_result_ = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);
    while (device_list.nDeviceNum == 0 && rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "未找到相机! 重试中...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      last_result_ = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);
    }
    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    MV_CC_OpenDevice(camera_handle_);

    // 2. 强制 BayerRG8 输出
    int temp_ret = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);
    if (temp_ret != MV_OK)
    {
      RCLCPP_WARN(this->get_logger(), "设置 PixelFormat 为 BayerRG8 失败");
    }

    MV_CC_GetImageInfo(camera_handle_, &img_info_);

    // 3. 预分配缓冲区（只在初始化时分配一次，热路径中不再 resize）
    size_t raw_size = img_info_.nHeightMax * img_info_.nWidthMax;
    raw_buffer_.resize(raw_size);
    rgb_msg_.data.resize(raw_size * 3);

    // 初始化像素转换参数
    convert_param_.nWidth = img_info_.nWidthMax;
    convert_param_.nHeight = img_info_.nHeightMax;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    // 5. 创建发布者
    raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS());

    save_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/save_frame", rclcpp::SensorDataQoS());

    declareParams();
    MV_CC_StartGrabbing(camera_handle_);

    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread(&HikCameraNode::captureLoop, this);
  }

  // ============================================================================
  // 析构函数
  // ============================================================================

  HikCameraNode::~HikCameraNode()
  {
    if (capture_thread_.joinable())
      capture_thread_.join();
    if (camera_handle_)
    {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode 已销毁!");
  }

  // ============================================================================
  // 图像抓取主循环
  // ============================================================================

  void HikCameraNode::captureLoop()
  {
    MV_FRAME_OUT out_frame;
    int fail_count = 0;

    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
    MV_CC_StartGrabbing(camera_handle_);

    while (rclcpp::ok())
    {
      last_result_ = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);

      if (last_result_ == MV_OK)
      {
        fail_count = 0;

        rclcpp::Time stamp = this->now();

        // 确定 Bayer 编码类型
        std::string bayer_encoding;
        bool is_bayer = true;

        switch (out_frame.stFrameInfo.enPixelType)
        {
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

        uint32_t frame_len = out_frame.stFrameInfo.nFrameLen;
        uint32_t height = out_frame.stFrameInfo.nHeight;
        uint32_t width = out_frame.stFrameInfo.nWidth;

        bool need_rgb = save_pub_->get_subscription_count() > 0;

        if (is_bayer)
        {
          // 保存 Raw 数据副本供 RGB 转换（在释放 SDK 缓冲区之前）
          if (need_rgb && frame_len <= raw_buffer_.size())
          {
            memcpy(raw_buffer_.data(), out_frame.pBufAddr, frame_len);
          }

          // 发布 /image_raw —— 使用 unique_ptr（兼容 intra-process）
          auto msg = std::make_unique<sensor_msgs::msg::Image>();
          msg->header.stamp = stamp;
          msg->header.frame_id = "camera_optical_frame";
          msg->encoding = bayer_encoding;
          msg->height = height;
          msg->width = width;
          msg->step = width;
          msg->data.resize(frame_len);
          memcpy(msg->data.data(), out_frame.pBufAddr, frame_len);
          raw_pub_->publish(std::move(msg));
        }

        // 释放 SDK 缓冲区 —— 相机可开始下一次曝光
        MV_CC_FreeImageBuffer(camera_handle_, &out_frame);

        // 发布 /save_frame（从已保存的 Raw 缓冲区进行 RGB 转换）
        if (need_rgb)
        {
          rgb_msg_.header.stamp = stamp;
          rgb_msg_.header.frame_id = "camera_optical_frame";
          rgb_msg_.encoding = "rgb8";
          rgb_msg_.height = height;
          rgb_msg_.width = width;
          rgb_msg_.step = width * 3;
          rgb_msg_.data.resize(height * width * 3);

          convert_param_.pDstBuffer = rgb_msg_.data.data();
          convert_param_.nDstBufferSize = rgb_msg_.data.size();
          convert_param_.pSrcData = raw_buffer_.data();
          convert_param_.nSrcDataLen = frame_len;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

          save_pub_->publish(rgb_msg_);
        }
      }
      else
      {
        fail_count++;
        if (fail_count >= 100)
        {
          RCLCPP_WARN(this->get_logger(), "获取图像缓冲区失败! 正在重置...");
          MV_CC_StopGrabbing(camera_handle_);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          MV_CC_StartGrabbing(camera_handle_);
          fail_count = 0;
        }
      }
    }
  }

  // ============================================================================
  // 参数管理
  // ============================================================================

  void HikCameraNode::declareParams()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE fValue;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // 曝光时间
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &fValue);
    param_desc.integer_range[0].from_value = fValue.fMin;
    param_desc.integer_range[0].to_value = fValue.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000.0, param_desc);
    last_result_ = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);

    // 增益
    MV_CC_GetFloatValue(camera_handle_, "Gain", &fValue);
    param_desc.integer_range[0].from_value = fValue.fMin;
    param_desc.integer_range[0].to_value = fValue.fMax;
    double gain = this->declare_parameter("gain", fValue.fCurValue, param_desc);
    last_result_ = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);

    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
    last_result_ = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", false);
  }

  rcl_interfaces::msg::SetParametersResult HikCameraNode::parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters)
    {
      if (param.get_name() == "exposure_time")
      {
        MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_double());
      }
      else if (param.get_name() == "gain")
      {
        MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
      }
    }
    return result;
  }

} // namespace hik_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(hik_publisher::HikCameraNode)
