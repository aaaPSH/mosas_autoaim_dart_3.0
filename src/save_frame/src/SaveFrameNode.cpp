/**
 * @file SaveFrameNode.cpp
 * @brief 帧录制节点实现——rosbag2 录制、缓冲区管理与磁盘空间控制
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#include "save_frame/SaveFrameNode.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/serialization.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include <rosbag2_compression_zstd/zstd_compressor.hpp>
#include "rosbag2_compression/compression_options.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "rclcpp/parameter_client.hpp"

namespace save_frame
{

  SaveFrameNode::SaveFrameNode(const rclcpp::NodeOptions &options)
      : Node("save_frame_node", options),
        last_stat_time_(std::chrono::steady_clock::now())
  {
    RCLCPP_INFO(this->get_logger(), "Initializing SaveFrameNode...");

    // 初始化参数
    initParameters();

    // 初始化订阅器
    initSubscribers();

    // 初始化录制
    initRecording();

    RCLCPP_INFO(this->get_logger(), "SaveFrameNode initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Save path: %s", save_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Detection topic: %s", detection_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Game status topic: %s", game_status_topic_.c_str());
  }

  SaveFrameNode::~SaveFrameNode()
  {
    stop_thread_ = true;
    buffer_cv_.notify_all();

    if (deferred_start_timer_)
    {
      deferred_start_timer_->cancel();
    }

    if (recording_thread_.joinable())
    {
      recording_thread_.join();
    }

    if (is_recording_)
    {
      stopRecording();
    }

    RCLCPP_INFO(this->get_logger(), "SaveFrameNode destroyed");
  }

  void SaveFrameNode::initParameters()
  {
    // 基本配置
    save_path_ = this->declare_parameter("save_path", "./recordings");
    image_topic_ = this->declare_parameter("image_topic", "/save_frame");
    raw_image_topic_ = this->declare_parameter("raw_image_topic", "/image_raw");
    detection_topic_ = this->declare_parameter("detection_topic", "/detections/green_dots");
    game_status_topic_ = this->declare_parameter("game_status_topic", "/game_status");
    use_game_status_ = this->declare_parameter("use_game_status", true);

    record_images_ = this->declare_parameter("record_images", true);
    record_raw_images_ = this->declare_parameter("record_raw_images", true);
    record_detections_ = this->declare_parameter("record_detections", true);
    auto_start_ = this->declare_parameter("auto_start", false);

    // 性能配置
    buffer_size_ = this->declare_parameter("buffer_size", 1000);
    batch_size_ = this->declare_parameter("batch_size", 50);
    use_compression_ = this->declare_parameter("use_compression", true);

    // 间隔取帧配置
    frame_interval_ = this->declare_parameter("frame_interval", 1);                           // 每N帧保存一帧，1=每帧都保存
    time_interval_ms_ = this->declare_parameter("time_interval_ms", static_cast<int64_t>(0)); // 时间间隔（毫秒），0=不按时间间隔

    // 存储管理
    max_file_size_mb_ = this->declare_parameter("max_file_size_mb", 1024);            // 1GB
    max_total_size_gb_ = this->declare_parameter("max_total_size_gb", 50);            // 50GB
    disk_space_threshold_gb_ = this->declare_parameter("disk_space_threshold_gb", 5); // 5GB

    // 相机内参将从/camera_info话题获取，这里只声明默认值（用于初始化）
    camera_matrix_ = {1500.0, 0.0, 640.0, 0.0, 1500.0, 360.0, 0.0, 0.0, 1.0};
    distortion_coeffs_ = {0.0, 0.0, 0.0, 0.0, 0.0};
    distortion_model_ = "plumb_bob";

    // 初始化间隔取帧状态
    frame_counter_ = 0;
    last_saved_time_ = std::chrono::steady_clock::now();

    // 创建保存目录
    std::filesystem::create_directories(save_path_);
  }

  void SaveFrameNode::initSubscribers()
  {
    // 订阅相机信息话题
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::SensorDataQoS(),
        std::bind(&SaveFrameNode::cameraInfoCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to camera info topic: /camera_info");

    if (record_images_)
    {
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          image_topic_, rclcpp::SensorDataQoS(),
          std::bind(&SaveFrameNode::imageCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic_.c_str());
    }

    if (record_raw_images_)
    {
      raw_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          raw_image_topic_, rclcpp::SensorDataQoS(),
          std::bind(&SaveFrameNode::rawImageCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Subscribed to raw image topic: %s", raw_image_topic_.c_str());
    }

    if (record_detections_)
    {
      detection_sub_ = this->create_subscription<autoaim_interfaces::msg::GreenDot>(
          detection_topic_, rclcpp::SensorDataQoS(),
          std::bind(&SaveFrameNode::detectionCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Subscribed to detection topic: %s", detection_topic_.c_str());
    }

    // 比赛状态订阅
    if (use_game_status_)
    {
      game_status_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
          game_status_topic_, rclcpp::SensorDataQoS(),
          std::bind(&SaveFrameNode::gameStatusCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Subscribed to game status topic: %s", game_status_topic_.c_str());
    }
  }

  void SaveFrameNode::initRecording()
  {
    // 启动录制线程
    recording_thread_ = std::thread(&SaveFrameNode::recordingThread, this);

    // 打印 auto_start 参数实际值，辅助诊断
    bool auto_start = false;
    this->get_parameter_or("auto_start", auto_start, false);
    // RCLCPP_INFO(this->get_logger(), "auto_start parameter = %s (will try direct start and deferred start)",
    //             auto_start ? "true" : "false");

    // 方式1: 立即启动（适用于参数在构造时已生效的情况）
    if (auto_start)
    {
      startRecording();
    }

    // 方式2: 延迟启动（适用于参数在构造后才生效的情况）
    // 定时器在节点完全初始化后触发，此时参数和服务都已就绪
    deferred_start_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(300),
        [this]()
        {
          deferred_start_timer_->cancel();
          if (is_recording_)
          {
            return;
          }
          bool auto_start = false;
          this->get_parameter_or("auto_start", auto_start, false);
          if (auto_start)
          {
            // RCLCPP_INFO(this->get_logger(), "Deferred start triggered, attempting recording");
            startRecording();
          }
        });
  }

  void SaveFrameNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    frames_received_++;

    if (!is_recording_ || !record_images_)
    {
      return;
    }

    // 检查磁盘空间
    uint64_t available_space = getAvailableDiskSpace(save_path_);
    if (available_space < disk_space_threshold_gb_ * 1024ULL * 1024ULL * 1024ULL)
    {
      RCLCPP_WARN(this->get_logger(), "Low disk space: %lu MB available. Attempting to clean up old files...",
                  available_space / (1024 * 1024));
      cleanupOldFiles();
      available_space = getAvailableDiskSpace(save_path_);
      if (available_space < disk_space_threshold_gb_ * 1024ULL * 1024ULL * 1024ULL)
      {
        RCLCPP_WARN(this->get_logger(), "Still low disk space: %lu MB available after cleanup. Stopping recording.",
                    available_space / (1024 * 1024));
        stopRecording();
        return;
      }
    }

    // 间隔取帧逻辑
    bool should_save = true;

    // 基于帧间隔的过滤
    if (frame_interval_ > 1)
    {
      frame_counter_++;
      if (frame_counter_ % frame_interval_ != 0)
      {
        should_save = false;
      }
    }

    // 基于时间间隔的过滤（如果同时启用，取更严格的限制）
    if (time_interval_ms_ > 0)
    {
      auto now = std::chrono::steady_clock::now();
      auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - last_saved_time_)
                            .count();

      if (elapsed_ms < time_interval_ms_)
      {
        should_save = false;
      }
      else
      {
        last_saved_time_ = now;
      }
    }

    // 如果不满足保存条件，直接返回
    if (!should_save)
    {
      // 即使不保存，也更新性能统计
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_stat_time_).count();
      if (elapsed >= 5)
      {
        double fps = static_cast<double>(frames_received_) / elapsed;
        RCLCPP_INFO(this->get_logger(), "Image FPS: %.1f, Buffer size: %zu (filtered)", fps, message_buffer_.size());
        frames_received_ = 0;
        last_stat_time_ = now;
      }
      return;
    }

    // 序列化消息
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    rclcpp::SerializedMessage serialized_msg;
    serialization.serialize_message(msg.get(), &serialized_msg);

    // 添加到缓冲区
    addToBuffer(image_topic_, serialized_msg);

    // 性能统计
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_stat_time_).count();
    if (elapsed >= 5)
    {
      double fps = static_cast<double>(frames_received_) / elapsed;
      RCLCPP_INFO(this->get_logger(), "Image FPS: %.1f, Buffer size: %zu", fps, message_buffer_.size());
      frames_received_ = 0;
      last_stat_time_ = now;
    }
  }

  void SaveFrameNode::rawImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!is_recording_ || !record_raw_images_)
    {
      return;
    }

    // 检查磁盘空间
    uint64_t available_space = getAvailableDiskSpace(save_path_);
    if (available_space < disk_space_threshold_gb_ * 1024ULL * 1024ULL * 1024ULL)
    {
      RCLCPP_WARN(this->get_logger(), "Low disk space: %lu MB available. Attempting to clean up old files...",
                  available_space / (1024 * 1024));
      cleanupOldFiles();
      available_space = getAvailableDiskSpace(save_path_);
      if (available_space < disk_space_threshold_gb_ * 1024ULL * 1024ULL * 1024ULL)
      {
        RCLCPP_WARN(this->get_logger(), "Still low disk space: %lu MB available after cleanup. Stopping recording.",
                    available_space / (1024 * 1024));
        stopRecording();
        return;
      }
    }

    // 间隔取帧逻辑
    bool should_save = true;

    if (frame_interval_ > 1)
    {
      raw_frame_counter_++;
      if (raw_frame_counter_ % frame_interval_ != 0)
      {
        should_save = false;
      }
    }

    if (time_interval_ms_ > 0)
    {
      auto now = std::chrono::steady_clock::now();
      auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - last_raw_saved_time_)
                            .count();

      if (elapsed_ms < time_interval_ms_)
      {
        should_save = false;
      }
      else
      {
        last_raw_saved_time_ = now;
      }
    }

    if (!should_save)
    {
      return;
    }

    // 序列化消息
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    rclcpp::SerializedMessage serialized_msg;
    serialization.serialize_message(msg.get(), &serialized_msg);

    // 添加到缓冲区
    addToBuffer(raw_image_topic_, serialized_msg);
  }

  void SaveFrameNode::detectionCallback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg)
  {
    if (!is_recording_ || !record_detections_)
    {
      return;
    }

    // 序列化消息
    rclcpp::Serialization<autoaim_interfaces::msg::GreenDot> serialization;
    rclcpp::SerializedMessage serialized_msg;
    serialization.serialize_message(msg.get(), &serialized_msg);

    // 添加到缓冲区
    addToBuffer(detection_topic_, serialized_msg);
  }

  void SaveFrameNode::gameStatusCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    uint8_t status = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received game status: %u", status);

    if (status == 4) // IN_GAME
    {
      if (!is_recording_)
      {
        RCLCPP_INFO(this->get_logger(), "Game started, starting recording");
        startRecording();
      }
    }
    else if (status == 5) // END_GAME
    {
      if (is_recording_)
      {
        RCLCPP_INFO(this->get_logger(), "Game ended, stopping recording");
        stopRecording();
      }
    }
  }

  void SaveFrameNode::addToBuffer(const std::string &topic_name, const rclcpp::SerializedMessage &msg)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    // 如果缓冲区已满，丢弃最旧的消息
    if (message_buffer_.size() >= buffer_size_)
    {
      message_buffer_.pop_front();
      messages_dropped_++;

      if (messages_dropped_ % 100 == 0)
      {
        RCLCPP_WARN(this->get_logger(), "Buffer full, dropped %zu messages", messages_dropped_);
      }
    }

    // 添加新消息到缓冲区
    BufferedMessage buffered_msg;
    buffered_msg.timestamp = this->now();
    buffered_msg.topic_name = topic_name;
    buffered_msg.serialized_msg = std::make_shared<rclcpp::SerializedMessage>(msg);

    message_buffer_.push_back(buffered_msg);

    // 通知录制线程
    buffer_cv_.notify_one();
  }

  void SaveFrameNode::recordingThread()
  {
    while (!stop_thread_)
    {
      std::unique_lock<std::mutex> lock(buffer_mutex_);

      // 等待缓冲区有数据或停止信号
      buffer_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]()
                          { return !message_buffer_.empty() || stop_thread_; });

      if (stop_thread_)
      {
        break;
      }

      if (!message_buffer_.empty() && is_recording_)
      {
        lock.unlock(); // 释放 buffer 锁，让 processBufferBatch 自己获取
        processBufferBatch();
      }
      else
      {
        lock.unlock();
      }

      // 检查是否需要创建新文件
      if (is_recording_ && current_file_size_ > max_file_size_mb_ * 1024ULL * 1024ULL)
      {
        createNewBagFile();
      }
    }
  }

  void SaveFrameNode::processBufferBatch()
  {
    size_t processed = 0;
    std::vector<BufferedMessage> batch;

    // 获取一批消息
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      size_t count = std::min(batch_size_, message_buffer_.size());
      batch.reserve(count);

      for (size_t i = 0; i < count; ++i)
      {
        batch.push_back(std::move(message_buffer_.front()));
        message_buffer_.pop_front();
      }
    }

    // 写入消息（在 writer_mutex_ 保护下）
    {
      std::lock_guard<std::mutex> wlock(writer_mutex_);
      if (!bag_writer_)
      {
        return;
      }
      for (auto &msg : batch)
      {
        try
        {
          rosbag2_storage::SerializedBagMessage serialized_bag_msg;
          serialized_bag_msg.serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
              const_cast<rcutils_uint8_array_t *>(&msg.serialized_msg->get_rcl_serialized_message()),
              [](rcutils_uint8_array_t * /* data */) {});

          serialized_bag_msg.topic_name = msg.topic_name;
          serialized_bag_msg.time_stamp = msg.timestamp.nanoseconds();

          bag_writer_->write(std::make_shared<rosbag2_storage::SerializedBagMessage>(serialized_bag_msg));

          // 更新文件大小估计（粗略估计）
          current_file_size_ += msg.serialized_msg->size();
          total_recorded_size_ += msg.serialized_msg->size();
          frames_recorded_++;

          processed++;
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to write message: %s", e.what());
        }
      }
    }

    if (processed > 0)
    {
      // 定期清理旧文件
      if (total_recorded_size_ > max_total_size_gb_ * 1024ULL * 1024ULL * 1024ULL)
      {
        cleanupOldFiles();
        total_recorded_size_ = 0;
      }
    }
  }

  void SaveFrameNode::createRecordingDir()
  {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;

    std::stringstream ss;
    ss << "match_" << current_match_id_ << "_"
       << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(3) << ms.count();

    recording_dir_ = (std::filesystem::path(save_path_) / ss.str()).string();
    std::filesystem::create_directories(recording_dir_);

    RCLCPP_INFO(this->get_logger(), "Created recording directory: %s", recording_dir_.c_str());
  }

  void SaveFrameNode::saveDetectParams()
  {
    // 检查节点是否已完全构造（被 shared_ptr 持有）
    // 在构造函数中调用时 weak_from_this 是 expired，直接返回，避免阻塞
    auto weak_this = this->weak_from_this();
    if (weak_this.expired())
    {
      RCLCPP_INFO(this->get_logger(), "Node not yet owned by shared_ptr, skipping param save during init");
      return;
    }

    try
    {
      // 创建参数客户端连接到green_dot_detect_node
      auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
          weak_this.lock(), "green_dot_detect_node");

      // 等待服务可用
      if (!param_client->wait_for_service(std::chrono::seconds(2)))
      {
        RCLCPP_WARN(this->get_logger(), "green_dot_detect_node parameter service not available, skipping detect params save");
        return;
      }

      // 获取所有detect参数
      std::vector<std::string> param_names = {
          "detect.v_low", "detect.min_area", "detect.max_area",
          "detect.min_aspect_ratio", "detect.max_aspect_ratio", "detect.min_circularity",
          "detect.min_gr_ratio", "detect.min_gb_ratio", "detect.search_strip_min_h",
          "detect.camera_height", "detect.target_height", "detect.detect_scale",
          "detect.distance", "detect.calibrated_pixel_x",
          "camera.matrix", "camera.dist_coeffs"};

      auto params = param_client->get_parameters(param_names);

      // 验证参数是否有效
      size_t valid_count = 0;
      for (const auto &param : params)
      {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
        {
          valid_count++;
        }
        else if (!param.get_name().empty())
        {
          RCLCPP_WARN(this->get_logger(), "Parameter '%s' returned NOT_SET", param.get_name().c_str());
        }
      }
      if (valid_count == 0)
      {
        RCLCPP_WARN(this->get_logger(), "All parameters returned empty - parameter service may not be available");
      }

      // 写入yaml文件
      std::string yaml_path = (std::filesystem::path(recording_dir_) / "detect_params.yaml").string();
      std::ofstream yaml_file(yaml_path);

      if (!yaml_file.is_open())
      {
        RCLCPP_WARN(this->get_logger(), "Failed to open detect_params.yaml for writing");
        return;
      }

      // 获取当前时间
      auto now = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(now);
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) %
                1000;

      yaml_file << "# 录制时的detect参数" << std::endl;
      yaml_file << "# recorded_at: "
                << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
                << "." << std::setfill('0') << std::setw(3) << ms.count()
                << std::endl;
      yaml_file << "detect:" << std::endl;

      for (const auto &param : params)
      {
        const std::string &name = param.get_name();
        // 只保存detect.*参数
        if (name.find("detect.") == 0)
        {
          std::string short_name = name.substr(7); // 去掉"detect."前缀
          yaml_file << "  " << short_name << ": ";

          switch (param.get_type())
          {
          case rclcpp::ParameterType::PARAMETER_INTEGER:
            yaml_file << param.as_int();
            break;
          case rclcpp::ParameterType::PARAMETER_DOUBLE:
            yaml_file << param.as_double();
            break;
          case rclcpp::ParameterType::PARAMETER_BOOL:
            yaml_file << (param.as_bool() ? "true" : "false");
            break;
          case rclcpp::ParameterType::PARAMETER_STRING:
            yaml_file << "\"" << param.as_string() << "\"";
            break;
          default:
            yaml_file << "null";
            break;
          }
          yaml_file << std::endl;
        }
      }

      // 保存相机参数
      yaml_file << "camera:" << std::endl;
      for (const auto &param : params)
      {
        const std::string &name = param.get_name();
        if (name == "camera.matrix")
        {
          yaml_file << "  matrix: [";
          auto values = param.as_double_array();
          for (size_t i = 0; i < values.size(); ++i)
          {
            if (i > 0)
              yaml_file << ", ";
            yaml_file << values[i];
          }
          yaml_file << "]" << std::endl;
        }
        else if (name == "camera.dist_coeffs")
        {
          yaml_file << "  dist_coeffs: [";
          auto values = param.as_double_array();
          for (size_t i = 0; i < values.size(); ++i)
          {
            if (i > 0)
              yaml_file << ", ";
            yaml_file << values[i];
          }
          yaml_file << "]" << std::endl;
        }
      }

      yaml_file.close();
      RCLCPP_INFO(this->get_logger(), "Saved detect params to: %s", yaml_path.c_str());
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to save detect params: %s", e.what());
    }
  }

  void SaveFrameNode::startRecording()
  {
    if (is_recording_)
    {
      RCLCPP_WARN(this->get_logger(), "Already recording");
      return;
    }

    try
    {
      // 录制开始前清理旧文件，为新录制腾出空间
      cleanupOldFiles();

      // 创建按时间命名的录制目录
      createRecordingDir();

      // 保存当前detect参数到录制目录
      saveDetectParams();

      // 重置统计 (createNewBagFile 会自增 file_counter_, 先置0)
      file_counter_ = 0;
      frames_recorded_ = 0;
      messages_dropped_ = 0;
      current_file_size_ = 0;
      frame_counter_ = 0;
      raw_frame_counter_ = 0;
      last_saved_time_ = std::chrono::steady_clock::now();
      last_raw_saved_time_ = std::chrono::steady_clock::now();

      // 创建bag文件在录制目录中
      createNewBagFile();
      is_recording_ = true;

      RCLCPP_INFO(this->get_logger(), "Started recording to: %s", recording_dir_.c_str());
      RCLCPP_INFO(this->get_logger(), "Match ID: %s", current_match_id_.c_str());
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", e.what());
    }
  }

  void SaveFrameNode::stopRecording()
  {
    if (!is_recording_)
    {
      return;
    }

    is_recording_ = false;

    // 处理缓冲区中剩余的消息
    processBufferBatch();

    {
      std::lock_guard<std::mutex> wlock(writer_mutex_);
      if (bag_writer_)
      {
        bag_writer_.reset();
      }
    }

    RCLCPP_INFO(this->get_logger(), "Stopped recording");
    RCLCPP_INFO(this->get_logger(), "Total frames recorded: %zu", frames_recorded_);
    RCLCPP_INFO(this->get_logger(), "Messages dropped: %zu", messages_dropped_);
    RCLCPP_INFO(this->get_logger(), "Buffer estimate: %.1f MB, compression=%s",
                current_file_size_.load() / (1024.0 * 1024.0),
                use_compression_ ? "zstd" : "none");
  }

  void SaveFrameNode::createNewBagFile()
  {
    std::lock_guard<std::mutex> wlock(writer_mutex_);
    if (bag_writer_)
    {
      bag_writer_.reset(); // 关闭旧文件，触发 FILE 模式压缩
    }

    std::string filename = generateFilename();
    std::string full_path = (std::filesystem::path(recording_dir_) / filename).string();

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = full_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    if (use_compression_)
    {
      try
      {
        rosbag2_compression::CompressionOptions compression_options;
        compression_options.compression_mode = rosbag2_compression::CompressionMode::FILE;
        compression_options.compression_format = "zstd";

        auto compression_writer = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(compression_options);
        bag_writer_ = std::move(compression_writer);

        bag_writer_->open(storage_options, converter_options);

        RCLCPP_INFO(this->get_logger(), "[WRITER] compression=zstd(mode:FILE) uri=%s", full_path.c_str());
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(this->get_logger(), "[WRITER] compression init failed: %s", e.what());
        RCLCPP_WARN(this->get_logger(), "[WRITER] falling back to non-compression writer");
        bag_writer_.reset();
        use_compression_ = false;
      }
    }

    if (!bag_writer_)
    {
      try
      {
        bag_writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
        bag_writer_->open(storage_options, converter_options);
        RCLCPP_INFO(this->get_logger(), "[WRITER] compression=none uri=%s", full_path.c_str());
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "[WRITER] non-compression open failed: %s", e.what());
        bag_writer_.reset();
        throw; // 重新抛出，让 startRecording 的 catch 处理
      }
    }

    // 注册话题（失败不阻止录制，只记录警告）
    if (record_images_)
    {
      try
      {
        bag_writer_->create_topic(
            {image_topic_, "sensor_msgs/msg/Image", "cdr", ""});
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to create image topic: %s", e.what());
      }
    }

    if (record_raw_images_)
    {
      try
      {
        bag_writer_->create_topic(
            {raw_image_topic_, "sensor_msgs/msg/Image", "cdr", ""});
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to create raw image topic: %s", e.what());
      }
    }

    if (record_detections_)
    {
      try
      {
        bag_writer_->create_topic(
            {detection_topic_, "autoaim_interfaces/msg/GreenDot", "cdr", ""});
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to create detection topic: %s", e.what());
      }
    }

    current_file_size_ = 0;
    file_counter_++;

    RCLCPP_INFO(this->get_logger(), "Created new bag file: %s", full_path.c_str());
  }

  std::string SaveFrameNode::generateFilename() const
  {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;

    std::stringstream ss;
    ss << "match_" << current_match_id_ << "_"
       << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(3) << ms.count()
       << "_part" << std::setfill('0') << std::setw(3) << file_counter_
       << ".db3";

    return ss.str();
  }

  void SaveFrameNode::cleanupOldFiles()
  {
    try
    {
      std::vector<std::filesystem::path> files;

      // 递归收集所有bag文件
      for (const auto &entry : std::filesystem::recursive_directory_iterator(save_path_))
      {
        if (entry.is_regular_file() && entry.path().extension() == ".db3")
        {
          files.push_back(entry.path());
        }
      }

      // 按修改时间排序（最旧的在前面）
      std::sort(files.begin(), files.end(),
                [](const std::filesystem::path &a, const std::filesystem::path &b)
                {
                  return std::filesystem::last_write_time(a) < std::filesystem::last_write_time(b);
                });

      // 计算当前总大小
      uint64_t total_size = 0;
      for (const auto &file : files)
      {
        total_size += std::filesystem::file_size(file);
      }

      // 删除最旧的文件直到满足大小限制
      size_t deleted_count = 0;
      while (!files.empty() && total_size > max_total_size_gb_ * 1024ULL * 1024ULL * 1024ULL)
      {
        const auto &oldest_file = files.front();
        uint64_t file_size = std::filesystem::file_size(oldest_file);

        std::filesystem::remove(oldest_file);
        total_size -= file_size;
        files.erase(files.begin());
        deleted_count++;
      }

      if (deleted_count > 0)
      {
        RCLCPP_INFO(this->get_logger(), "Cleaned up %zu old bag files", deleted_count);
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to cleanup old files: %s", e.what());
    }
  }

  uint64_t SaveFrameNode::getAvailableDiskSpace(const std::string &path) const
  {
    try
    {
      std::filesystem::space_info space = std::filesystem::space(path);
      return space.available;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get disk space: %s", e.what());
      return UINT64_MAX;
    }
  }

  void SaveFrameNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // 提取相机内参矩阵
    if (msg->k.size() == 9)
    {
      camera_matrix_.clear();
      camera_matrix_.reserve(9);
      for (size_t i = 0; i < 9; ++i)
      {
        camera_matrix_.push_back(msg->k[i]);
      }
    }

    // 提取畸变系数
    distortion_coeffs_.clear();
    distortion_coeffs_.reserve(msg->d.size());
    for (const auto &coeff : msg->d)
    {
      distortion_coeffs_.push_back(coeff);
    }

    // 提取畸变模型
    distortion_model_ = msg->distortion_model;

    if (!camera_info_received_)
    {
      camera_info_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Received camera info: model=%s, matrix_size=%zu, dist_size=%zu",
                  distortion_model_.c_str(), camera_matrix_.size(), distortion_coeffs_.size());
    }
  }

} // namespace save_frame

RCLCPP_COMPONENTS_REGISTER_NODE(save_frame::SaveFrameNode)
