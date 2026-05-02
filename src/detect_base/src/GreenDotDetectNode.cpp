/**
 * @file GreenDotDetectNode.cpp
 * @brief 绿色灯点检测节点实现
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#include "detect_base/GreenDotDetectNode.hpp"

#include <ctime>
#include <iomanip>
#include <sstream>

#include "rclcpp_components/register_node_macro.hpp"

namespace detect_base
{

  // ============================================================================
  // 构造函数
  // ============================================================================

  /// @brief
  /// @param options
  GreenDotDetectNode::GreenDotDetectNode(const rclcpp::NodeOptions &options)
      : Node("green_dot_detect_node", options)
  {
    last_fps_time_ = std::chrono::steady_clock::now();
    last_save_time_ = std::chrono::steady_clock::now();

    detector_ = std::make_shared<GreenDotDetect>();

    // ==========================================
    // 1. 声明并初始化相机内参 (静态参数)
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
    RCLCPP_INFO(this->get_logger(), "相机参数已加载");

    // ==========================================
    // 2. 声明动态参数 (对应 DetectParams)
    // ==========================================
    this->declare_parameter("debug_mode", true);
    this->declare_parameter("save_images", false);
    this->declare_parameter("save_fps", 60.0);
    this->declare_parameter("save_path", save_path_);

    // 阈值与形态学
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
    this->declare_parameter("detect.camera_height", 0.0);
    this->declare_parameter("detect.target_height", 0.0);
    this->declare_parameter("detect.detect_scale", 10.0);
    this->declare_parameter("detect.distance", 25000.0);

    this->declare_parameter("detect.calibrated_pixel_x", 0.0);

    // 首次同步参数
    refreshParams();

    // ==========================================
    // 3. 注册回调
    // ==========================================
    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&GreenDotDetectNode::parametersCallback, this, std::placeholders::_1));

    // 订阅 Raw 图像
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&GreenDotDetectNode::imageCallback, this, std::placeholders::_1));

    // 订阅比赛状态
    game_status_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/game_status", rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::UInt8::SharedPtr msg)
        {
          game_started_.store(msg->data == 4);
        });

    target_pub_ = this->create_publisher<autoaim_interfaces::msg::GreenDot>(
        "/detections/green_dots", rclcpp::SensorDataQoS().keep_last(1));

    RCLCPP_INFO(this->get_logger(), "节点已初始化，等待图像...");

    // 启动异步保存线程
    save_thread_ = std::thread(&GreenDotDetectNode::saveThreadLoop, this);
  }

  GreenDotDetectNode::~GreenDotDetectNode()
  {
    stop_save_thread_ = true;
    save_cv_.notify_one();
    if (save_thread_.joinable())
    {
      save_thread_.join();
    }
  }

  // ============================================================================
  // 参数管理
  // ============================================================================

  void GreenDotDetectNode::refreshParams()
  {
    // 读取基础控制参数
    this->get_parameter("debug_mode", debug_mode_);
    this->get_parameter("save_images", save_images_);
    this->get_parameter("save_fps", save_fps_);
    this->get_parameter("save_path", save_path_);
    save_interval_ = std::chrono::milliseconds(static_cast<int>(1000.0 / save_fps_));

    // 读取算法参数
    this->get_parameter("detect.v_low", detect_params_.v_low);
    this->get_parameter("detect.min_area", detect_params_.min_area);
    this->get_parameter("detect.max_area", detect_params_.max_area);
    this->get_parameter("detect.min_aspect_ratio", detect_params_.min_aspect_ratio);
    this->get_parameter("detect.max_aspect_ratio", detect_params_.max_aspect_ratio);
    this->get_parameter("detect.min_circularity", detect_params_.min_circularity);

    this->get_parameter("detect.min_gr_ratio", detect_params_.min_gr_ratio);
    this->get_parameter("detect.min_gb_ratio", detect_params_.min_gb_ratio);
    this->get_parameter("detect.search_strip_min_h", detect_params_.search_strip_min_h);

    this->get_parameter("detect.camera_height", detect_params_.camera_height);
    this->get_parameter("detect.target_height", detect_params_.target_height);
    this->get_parameter("detect.detect_scale", detect_params_.detect_scale);
    this->get_parameter("detect.distance", detect_params_.distance);

    this->get_parameter("detect.calibrated_pixel_x", detect_params_.calibrated_pixel_x);

    // 更新算法内部状态
    detector_->update_params(detect_params_);

    RCLCPP_INFO(
        this->get_logger(),
        "[参数已更新] Debug:%d | V_Low:%d | Area:%.1f-%.1f | Dist:%.0f | CalPixelX:%.1f",
        debug_mode_, detect_params_.v_low, detect_params_.min_area, detect_params_.max_area,
        detect_params_.distance, detect_params_.calibrated_pixel_x);
  }

  rcl_interfaces::msg::SetParametersResult GreenDotDetectNode::parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    bool algo_params_changed = false;

    for (const auto &param : parameters)
    {
      const std::string &name = param.get_name();

      // 基础控制参数
      if (name == "debug_mode")
      {
        debug_mode_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Debug 模式: %d", debug_mode_);
      }
      else if (name == "save_images")
      {
        save_images_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "保存图像: %d", save_images_);
      }
      else if (name == "save_fps")
      {
        save_fps_ = param.as_double();
        save_interval_ = std::chrono::milliseconds(static_cast<int>(1000.0 / save_fps_));
        RCLCPP_INFO(this->get_logger(), "保存帧率: %.1f", save_fps_);
      }
      else if (name == "save_path")
      {
        save_path_ = param.as_string();
        RCLCPP_INFO(this->get_logger(), "保存路径: %s", save_path_.c_str());
      }
      // 算法参数——标记为需要更新
      else if (name == "detect.v_low")
      {
        detect_params_.v_low = param.as_int();
        algo_params_changed = true;
      }
      else if (name == "detect.min_area")
      {
        detect_params_.min_area = param.as_double();
        algo_params_changed = true;
      }
      else if (name == "detect.max_area")
      {
        detect_params_.max_area = param.as_double();
        algo_params_changed = true;
      }
      else if (name == "detect.min_aspect_ratio")
      {
        detect_params_.min_aspect_ratio = param.as_double();
        algo_params_changed = true;
      }
      else if (name == "detect.max_aspect_ratio")
      {
        detect_params_.max_aspect_ratio = param.as_double();
        algo_params_changed = true;
      }
      else if (name == "detect.min_circularity")
      {
        detect_params_.min_circularity = param.as_double();
        algo_params_changed = true;
      }
      else if (name == "detect.min_gr_ratio")
      {
        detect_params_.min_gr_ratio = param.as_double();
        algo_params_changed = true;
      }
      else if (name == "detect.min_gb_ratio")
      {
        detect_params_.min_gb_ratio = param.as_double();
        algo_params_changed = true;
      }
      else if (name == "detect.search_strip_min_h")
      {
        detect_params_.search_strip_min_h = param.as_int();
        algo_params_changed = true;
      }
      else if (name == "detect.distance")
      {
        detect_params_.distance = param.as_double();
        algo_params_changed = true;
      }
      else if (name == "detect.calibrated_pixel_x")
      {
        detect_params_.calibrated_pixel_x = param.as_double();
        algo_params_changed = true;
      }
      else if (name == "detect.detect_scale")
      {
        detect_params_.detect_scale = param.as_double();
        algo_params_changed = true;
      }
    }

    // 只有当算法相关的参数变化时，才调用 update_params
    if (algo_params_changed)
    {
      detector_->update_params(detect_params_);
      RCLCPP_INFO(
          this->get_logger(), "[动态] 算法参数已更新! V_Low: %d, Area: %.1f",
          detect_params_.v_low, detect_params_.min_area);
    }

    return result;
  }

  // ============================================================================
  // 图像回调
  // ============================================================================

  void GreenDotDetectNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat raw_frame;
    try
    {
      if (
          msg->encoding.find("bayer") != std::string::npos ||
          msg->encoding == sensor_msgs::image_encodings::MONO8)
      {
        auto cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        raw_frame = cv_ptr->image;
      }
      else
      {
        auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat bgr = cv_ptr->image;
        std::vector<cv::Mat> chs;
        cv::split(bgr, chs);
        raw_frame = chs[1]; // 提取 Green Channel
      }
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
      return;
    }

    // 执行检测
    std::vector<Dot> dots;
    bool found = detector_->detect(raw_frame, dots, debug_mode_);

    // 解算与输出
    if (found)
    {
      detector_->calculate_dots_yaw(dots);

      // 多目标筛选：只保留 Y 坐标最接近理论值的那一个
      if (dots.size() > 1)
      {
        double fy = camera_matrix_.at<double>(1, 1);
        double cy = camera_matrix_.at<double>(1, 2);

        double delta_H = detect_params_.target_height - detect_params_.camera_height;
        double Z = detect_params_.distance;

        // 计算理论上绿点应该出现的 Y 像素坐标
        double expected_y_pixel = cy - fy * (delta_H / Z);

        // 遍历所有候选绿点，找到 Y 坐标最接近理论值的那个
        double min_y_diff = std::numeric_limits<double>::max();
        Dot best_dot;

        for (const auto &dot : dots)
        {
          double y_diff = std::abs(dot.center.y - expected_y_pixel);

          if (y_diff < min_y_diff)
          {
            min_y_diff = y_diff;
            best_dot = dot;
          }
        }

        // 清空原来的 dots 数组，只保留最匹配的那一个
        dots.clear();
        dots.push_back(best_dot);
      }

      // 发布检测到的目标
      autoaim_interfaces::msg::GreenDot target;
      auto &d = dots[0];

      target.header.stamp = msg->header.stamp;
      target.header.frame_id = msg->header.frame_id;
      target.x = d.center.x;
      target.y = d.center.y;
      target.angle_yaw = d.yaw;
      target.d_pixel = target.x - detect_params_.calibrated_pixel_x;

      target_pub_->publish(target);
    }
    else
    {
      // 未检测到目标，发布无效坐标 (-1, -1)
      autoaim_interfaces::msg::GreenDot target;
      target.header.stamp = msg->header.stamp;
      target.header.frame_id = msg->header.frame_id;
      target.x = -1;
      target.y = -1;

      target_pub_->publish(target);
    }

    // FPS 统计与图像保存
    auto now = std::chrono::steady_clock::now();

    if (debug_mode_)
    {
      fps_counter_++;
      auto fps_diff =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time_).count();
      if (fps_diff >= 1000)
      {
        RCLCPP_INFO(this->get_logger(), "FPS: %d", fps_counter_);
        fps_counter_ = 0;
        last_fps_time_ = now;
      }
    }

    auto save_diff =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_save_time_).count();
    if (save_images_ && game_started_.load() && save_diff >= save_interval_.count())
    {
      {
        std::lock_guard<std::mutex> lock(save_mutex_);
        if (save_queue_.size() >= 30)
          save_queue_.pop();
        save_queue_.push(raw_frame.clone());
      }
      save_cv_.notify_one();
      last_save_time_ = now;
    }
  }

  // ============================================================================
  // 异步保存线程
  // ============================================================================

  void GreenDotDetectNode::saveImageToDisk(const cv::Mat &image)
  {
    // 按节点启动时间创建子目录（只在第一次创建）
    std::string base_path = save_path_;
    static std::string save_dir = [base_path]()
    {
      auto now = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << base_path << "/"
         << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
      std::string dir = ss.str();
      std::filesystem::create_directories(dir);
      return dir;
    }();

    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << save_dir << "/DOT_"
       << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << "_"
       << std::setfill('0') << std::setw(3) << ms.count() << ".jpg";

    bool success = cv::imwrite(ss.str(), image);
    if (!success)
    {
      RCLCPP_ERROR(this->get_logger(), "图像保存失败: %s", ss.str().c_str());
    }
  }

  void GreenDotDetectNode::saveThreadLoop()
  {
    while (!stop_save_thread_)
    {
      cv::Mat frame;
      {
        std::unique_lock<std::mutex> lock(save_mutex_);
        save_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]()
                          { return !save_queue_.empty() || stop_save_thread_; });
        if (stop_save_thread_ && save_queue_.empty())
          break;
        if (save_queue_.empty())
          continue;
        frame = std::move(save_queue_.front());
        save_queue_.pop();
      }
      saveImageToDisk(frame);
    }
  }

} // namespace detect_base

RCLCPP_COMPONENTS_REGISTER_NODE(detect_base::GreenDotDetectNode)
