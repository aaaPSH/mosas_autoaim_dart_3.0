/**
 * @file SaveFrameNode.hpp
 * @brief 图像与检测结果录制节点，基于 rosbag2 实现数据记录
 *
 * 订阅 /save_frame (RGB 图像) 和 /detections/green_dots (检测结果)，
 * 通过 rosbag2 写入 ZSTD 压缩的 bag 文件，支持缓冲区批处理和磁盘空间管理。
 *
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "autoaim_interfaces/msg/green_dot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_compression/compression_options.hpp"
#include "rosbag2_compression_zstd/zstd_compressor.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "rclcpp/parameter_client.hpp"

namespace save_frame
{

  /** @brief 缓冲消息结构体，存储序列化后的消息及其元数据 */
  struct BufferedMessage
  {
    rclcpp::Time timestamp;
    std::string topic_name;
    std::shared_ptr<rclcpp::SerializedMessage> serialized_msg;
  };

  /**
   * @brief 帧录制节点
   *
   * 订阅多个话题，通过 rosbag2 将数据写入 ZSTD 压缩的 bag 文件。
   * 支持：
   * - 可配置的录制路径、缓冲区大小、文件大小限制
   * - 自动磁盘空间管理和旧文件清理
   * - 间隔取帧控制（按帧数或时间间隔）
   * - 相机内参自动从 /camera_info 话题获取并记录
   */
  class SaveFrameNode : public rclcpp::Node
  {
  public:
    explicit SaveFrameNode(const rclcpp::NodeOptions &options);
    ~SaveFrameNode();

  private:
    /** @brief 初始化 ROS 参数 */
    void initParameters();

    /** @brief 初始化话题订阅 */
    void initSubscribers();

    /** @brief 初始化 rosbag2 录制器 */
    void initRecording();

    // --- 回调函数 ---
    /** @brief 图像话题回调 */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /** @brief 原始图像话题回调 */
    void rawImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /** @brief 检测结果话题回调 */
    void detectionCallback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg);

    /** @brief 比赛状态话题回调 */
    void gameStatusCallback(const std_msgs::msg::UInt8::SharedPtr msg);

    /** @brief 相机内参话题回调 */
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    // --- 录制控制 ---
    /** @brief 开始录制 */
    void startRecording();

    /** @brief 停止录制 */
    void stopRecording();

    /** @brief 录制线程主循环 */
    void recordingThread();

    // --- 缓冲区管理 ---
    /** @brief 将消息加入写缓冲区 */
    void addToBuffer(const std::string &topic_name, const rclcpp::SerializedMessage &msg);

    /** @brief 批量处理缓冲区中的消息，写入 bag 文件 */
    void processBufferBatch();

    // --- 文件管理 ---
    /** @brief 创建新 bag 文件 */
    void createNewBagFile();

    /** @brief 清理超出大小限制的旧文件 */
    void cleanupOldFiles();

    /** @brief 生成带时间戳的文件名 */
    std::string generateFilename() const;

    /** @brief 获取指定路径的可用磁盘空间 */
    uint64_t getAvailableDiskSpace(const std::string &path) const;

    // --- 录制目录管理 ---
    /** @brief 创建录制输出目录 */
    void createRecordingDir();

    /** @brief 保存当前检测参数到录制目录 */
    void saveDetectParams();

    std::string recording_dir_;

    // --- 配置参数 ---
    std::string save_path_;
    std::string image_topic_;
    std::string raw_image_topic_;
    std::string detection_topic_;
    std::string game_status_topic_;
    bool use_game_status_;
    bool record_images_;
    bool record_raw_images_;
    bool record_detections_;
    bool auto_start_;
    size_t buffer_size_;
    size_t batch_size_;
    size_t max_file_size_mb_;
    size_t max_total_size_gb_;
    size_t disk_space_threshold_gb_;
    bool use_compression_;

    // --- 间隔取帧参数 ---
    size_t frame_interval_;
    size_t frame_counter_;
    size_t raw_frame_counter_;
    int64_t time_interval_ms_;
    std::chrono::steady_clock::time_point last_saved_time_;
    std::chrono::steady_clock::time_point last_raw_saved_time_;

    // --- 相机内参（从话题获取） ---
    std::vector<double> camera_matrix_;
    std::vector<double> distortion_coeffs_;
    std::string distortion_model_;
    std::atomic<bool> camera_info_received_{false};

    // --- 状态变量 ---
    std::atomic<bool> is_recording_{false};
    std::atomic<bool> stop_thread_{false};
    std::atomic<size_t> current_file_size_{0};
    std::atomic<size_t> total_recorded_size_{0};
    size_t file_counter_{0};
    std::string current_match_id_{"default"};

    // --- 线程和同步 ---
    std::thread recording_thread_;
    std::mutex buffer_mutex_;
    std::mutex writer_mutex_;
    std::condition_variable buffer_cv_;
    std::deque<BufferedMessage> message_buffer_;

    // --- ROS 组件 ---
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
    rclcpp::Subscription<autoaim_interfaces::msg::GreenDot>::SharedPtr detection_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr game_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> bag_writer_;
    rclcpp::TimerBase::SharedPtr deferred_start_timer_;

    // --- 性能统计 ---
    std::chrono::steady_clock::time_point last_stat_time_;
    size_t frames_received_{0};
    size_t frames_recorded_{0};
    size_t messages_dropped_{0};
  };

} // namespace save_frame
