#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include "autoaim_interfaces/msg/green_dot.hpp"
#include "cv_bridge/cv_bridge.h"
#include "detect_base/Dot.hpp"
#include "detect_base/GreenDotDetect.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <filesystem>

class GreenDotDetectNode : public rclcpp::Node
{
private:
  // 订阅者 & 算法对象
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<autoaim_interfaces::msg::GreenDot>::SharedPtr target_pub;
  std::shared_ptr<GreenDotDetect> detector_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  // 静态相机参数 (只读一次)
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // FPS 统计
  int fps_counter_ = 0;
  std::chrono::steady_clock::time_point last_fps_time_;
  //储存计时器
  std::chrono::steady_clock::time_point last_save_time_;

  std::chrono::milliseconds save_interval_{1000/60};


  // 调试开关
  bool debug_mode_ = false;
  bool save_images = false;


  DetectParams p;

public:
  GreenDotDetectNode(const rclcpp::NodeOptions & options) : Node("green_dot_detect_node", options)
  {
    last_fps_time_ = std::chrono::steady_clock::now();
    last_save_time_ = std::chrono::steady_clock::now();

    detector_ = std::make_shared<GreenDotDetect>();

    // ==========================================
    // 1. 声明并初始化相机内参 (Static)
    // ==========================================
    std::vector<double> default_k = {1500.0, 0.0, 640.0, 0.0, 1500.0, 360.0, 0.0, 0.0, 1.0};
    std::vector<double> default_d = {0.0, 0.0, 0.0, 0.0, 0.0};

    this->declare_parameter("camera.matrix", default_k);
    this->declare_parameter("camera.dist_coeffs", default_d);

    std::vector<double> k_data, d_data;
    this->get_parameter("camera.matrix", k_data);
    this->get_parameter("camera.dist_coeffs", d_data);

    camera_matrix_ = cv::Mat(3, 3, CV_64F, k_data.data()).clone();
    dist_coeffs_ = cv::Mat(d_data.size(), 1, CV_64F, d_data.data()).clone();

    // 初始化算法的相机部分
    detector_->init_camera(camera_matrix_, dist_coeffs_);
    RCLCPP_INFO(this->get_logger(), "Camera parameters loaded.");

    // ==========================================
    // 2. 声明动态参数 (对应 DetectParams)
    // ==========================================
    this->declare_parameter("debug_mode", true);
    this->declare_parameter("save_images", false);

    // 阈值与形态
    this->declare_parameter("detect.v_low", 100);
    this->declare_parameter("detect.min_area", 2.0);
    this->declare_parameter("detect.max_area", 200.0);
    this->declare_parameter("detect.min_aspect_ratio", 0.4);
    this->declare_parameter("detect.max_aspect_ratio", 2.5);
    this->declare_parameter("detect.min_circularity", 0.5);

    // 抗干扰
    this->declare_parameter("detect.min_gr_ratio", 2.3);
    this->declare_parameter("detect.min_gb_ratio", 0.8);
    this->declare_parameter("detect.search_strip_min_h", 20);

    // 物理场景
    this->declare_parameter("detect.camera_height", 0.0);  // mm
    this->declare_parameter("detect.target_height", 0.0);  // mm
    this->declare_parameter("detect.detect_scale", 10.0);  // deg
    this->declare_parameter("detect.distance", 25000.0);   // mm

    this->declare_parameter("detect.pix_offset", 0.0);

    // 首次同步参数
    refresh_params();

    // ==========================================
    // 3. 注册回调
    // ==========================================
    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&GreenDotDetectNode::parametersCallback, this, std::placeholders::_1));

    // 订阅 Raw 图像
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&GreenDotDetectNode::imageCallback, this, std::placeholders::_1));

    target_pub = this->create_publisher<autoaim_interfaces::msg::GreenDot>(
      "/detections/green_dots", rclcpp::SensorDataQoS().keep_last(1));

    RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for images...");
  }

private:
  void save_image_to_disk(const cv::Mat & image)
  {
      cv::Mat save_img = image.clone();
      
      // 1. 定义你的保存目录
      std::string save_dir = "/home/nvidia/saved_green_dots";

      // 2. 检查并创建目录 (如果目录不存在，就自动创建它)
      if (!std::filesystem::exists(save_dir)) {
          std::filesystem::create_directories(save_dir);
          // 如果是在 ROS 2 节点中，建议加一句日志打印：
          // RCLCPP_INFO(this->get_logger(), "Created directory: %s", save_dir.c_str());
      }

      auto now = std::chrono::system_clock::now();
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
      std::time_t t = std::chrono::system_clock::to_time_t(now);

      std::stringstream ss;
      ss << save_dir << "/DOT_" 
        << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << "_" 
        << std::setfill('0') << std::setw(3) << ms.count() << ".jpg";

      // 3. 写入并检查返回值
      bool success = cv::imwrite(ss.str(), save_img);
      if (!success) {
          // 如果还是失败（比如权限不足），至少你会知道
          //std::cerr << "Failed to save image to: " << ss.str() << std::endl;
          RCLCPP_ERROR(this->get_logger(), "Failed to save image to: %s", ss.str().c_str());
      }
  }
  // -----------------------------------------------------------------
  // 从 ROS 参数服务器读取最新值 -> 打包进 DetectParams -> 传给算法
  // -----------------------------------------------------------------
  void refresh_params()
  {

    // 读取基础控制
    this->get_parameter("debug_mode", debug_mode_);
    this->get_parameter("save_images", save_images);

    // 读取算法参数
    this->get_parameter("detect.v_low", p.v_low);
    this->get_parameter("detect.min_area", p.min_area);
    this->get_parameter("detect.max_area", p.max_area);
    this->get_parameter("detect.min_aspect_ratio", p.min_aspect_ratio);
    this->get_parameter("detect.max_aspect_ratio", p.max_aspect_ratio);
    this->get_parameter("detect.min_circularity", p.min_circularity);

    this->get_parameter("detect.min_gr_ratio", p.min_gr_ratio);
    this->get_parameter("detect.min_gb_ratio", p.min_gb_ratio);
    this->get_parameter("detect.search_strip_min_h", p.search_strip_min_h);

    this->get_parameter("detect.camera_height", p.camera_height);
    this->get_parameter("detect.target_height", p.target_height);
    this->get_parameter("detect.detect_scale", p.detect_scale);
    this->get_parameter("detect.distance", p.distance);

    this->get_parameter("detect.pix_offset", p.pix_offset);

    // 更新算法内部状态
    detector_->update_params(p);

    RCLCPP_INFO(
      this->get_logger(),
      "[Params Updated] Debug:%d | V_Low:%d | Area:%.1f-%.1f | Dist:%.0f | YawOff:%.1f",
      debug_mode_, p.v_low, p.min_area, p.max_area, p.distance, p.pix_offset);
  }

  // -----------------------------------------------------------------
  // 动态参数回调
  // -----------------------------------------------------------------
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    bool algo_params_changed = false;

    for (const auto & param : parameters) {
      const std::string & name = param.get_name();

      // 1. 基础控制
      if (name == "debug_mode") {
        debug_mode_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Debug Mode Set: %d", debug_mode_);
      }
      // 2. 算法参数匹配 (注意类型转换)
      else if (name == "detect.v_low") {
        p.v_low = param.as_int();
        algo_params_changed = true;
      } else if (name == "detect.min_area") {
        p.min_area = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.max_area") {
        p.max_area = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.min_aspect_ratio") {
        p.min_aspect_ratio = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.max_aspect_ratio") {
        p.max_aspect_ratio = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.min_circularity") {
        p.min_circularity = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.min_gr_ratio") {
        p.min_gr_ratio = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.min_gb_ratio") {
        p.min_gb_ratio = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.search_strip_min_h") {
        p.search_strip_min_h = param.as_int();
        algo_params_changed = true;
      } else if (name == "detect.distance") {
        p.distance = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.pix_offset") {
        p.pix_offset = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.detect_scale") {
        p.detect_scale = param.as_double();
        algo_params_changed = true;
      }
    }

    // 只有当算法相关的参数变化时，才调用 update_params
    if (algo_params_changed) {
      detector_->update_params(p);
      RCLCPP_INFO(
        this->get_logger(), "[Dynamic] Algorithm Params Updated! V_Low: %d, Area: %.1f",
        p.v_low, p.min_area);
    }

    return result;
  }

  // -----------------------------------------------------------------
  // 图像回调
  // -----------------------------------------------------------------
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat raw_frame;
    try {
      // 2. 格式转换
      if (
        msg->encoding.find("bayer") != std::string::npos ||
        msg->encoding == sensor_msgs::image_encodings::MONO8) {
        raw_frame = cv_bridge::toCvShare(msg, msg->encoding)->image;
      } else {
        cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
        std::vector<cv::Mat> chs;
        cv::split(bgr, chs);
        raw_frame = chs[1];  // Green Channel
      }
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // 3. 执行检测
    std::vector<Dot> dots;
    bool found = detector_->detect(raw_frame, dots, debug_mode_);

    // 4. 解算与输出
    if (found) {
      detector_->calculate_dots_yaw(dots);

      // 假设这是在 GreenDotDetect::detect 的末尾，此时您已经有了 std::vector<Dot> dots
      if (dots.size() > 1) {
        // 1. 获取相机内参和物理配置参数
        double fy = camera_matrix_.at<double>(1, 1);
        double cy = camera_matrix_.at<double>(1, 2);

        double delta_H = p.target_height - p.camera_height;
        double Z = p.distance;

        // 2. 计算理论上绿点应该出现的 Y 像素坐标
        // (如果 target 比 camera 高，图像上 Y 应该小于 cy，所以用减法)
        double expected_y_pixel = cy - fy * (delta_H / Z);

        // 3. 遍历所有的候选绿点，找到 Y 坐标最接近理论值的那个
        double min_y_diff = std::numeric_limits<double>::max();
        Dot best_dot;

        for (const auto & dot : dots) {
          // dot.center.y 是您前面代码算出并映射回原图的绝对像素坐标
          double y_diff = std::abs(dot.center.y - expected_y_pixel);

          if (y_diff < min_y_diff) {
            min_y_diff = y_diff;
            best_dot = dot;
          }
        }

        // 4. 清空原来的 dots 数组，只保留最完美匹配的那一个！
        dots.clear();
        dots.push_back(best_dot);
      }

      // 输出所有检测到的点
      autoaim_interfaces::msg::GreenDot target;
      auto & d = dots[0];
      // RCLCPP_INFO(this->get_logger(),
      //     "Target Pos:(%.2f, %.2f) Yaw:%.3f deg",
      //     d.center.x, d.center.y, d.yaw);

      target.header.stamp = msg->header.stamp;
      target.header.frame_id = msg->header.frame_id;
      target.x = d.center.x;
      target.y = d.center.y;
      target.angle_yaw = d.yaw;
      target.d_pixel = target.x - camera_matrix_.at<double>(0, 2) + p.pix_offset;  // 加上像素偏移补偿
      //RCLCPP_INFO(this->get_logger(), "%2.f", p.pix_offset);

      target_pub->publish(target);
    } else if (!found) {
      // RCLCPP_INFO(this->get_logger(),"No Target!");
      autoaim_interfaces::msg::GreenDot target;
      target.header.stamp = msg->header.stamp;
      target.header.frame_id = msg->header.frame_id;
      target.x = -1;
      target.y = -1;

      target_pub->publish(target);
    }

    auto now = std::chrono::steady_clock::now();
    fps_counter_++;

    // 计算时间差 (秒)
    auto fps_diff =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time_).count();
    if (fps_diff >= 1000) {  // 超过1000ms
      RCLCPP_INFO(this->get_logger(), "FPS: %d", fps_counter_);
      fps_counter_ = 0;
      last_fps_time_ = now;
    }

    auto save_diff =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_save_time_).count();
    if (save_images && save_diff >= save_interval_.count()) {
      save_image_to_disk(raw_frame);
      RCLCPP_INFO(this->get_logger(), "Image saved to disk.");
      last_save_time_ = now;
    }
  }
};
