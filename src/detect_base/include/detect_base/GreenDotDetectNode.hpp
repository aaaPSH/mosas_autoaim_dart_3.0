/**
 * @file GreenDotDetectNode.hpp
 * @brief 绿色灯点检测 ROS 2 节点，封装 GreenDotDetect 算法，提供图像订阅和检测结果发布
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#pragma once

#include <chrono>
#include <limits>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "autoaim_interfaces/msg/green_dot.hpp"
#include "cv_bridge/cv_bridge.h"
#include "detect_base/Dot.hpp"
#include "detect_base/GreenDotDetect.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <filesystem>

namespace detect_base
{

  /**
   * @brief 绿色灯点检测节点
   *
   * 订阅相机的 Bayer Raw 图像，通过 GreenDotDetect 算法检测绿色 LED 灯点，
   * 将检测结果以 GreenDot 消息发布。支持调试可视化和图像保存。
   */
  class GreenDotDetectNode : public rclcpp::Node
  {
  public:
    explicit GreenDotDetectNode(const rclcpp::NodeOptions &options);
    ~GreenDotDetectNode() override;

  private:
    /** @brief ROS 参数更新回调 */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

    /** @brief 从参数服务器读取最新参数并更新算法 */
    void refreshParams();

    /** @brief 图像订阅回调 */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /** @brief 将图像保存到磁盘（由异步线程调用） */
    void saveImageToDisk(const cv::Mat &image);

    /** @brief 异步保存线程主循环 */
    void saveThreadLoop();

    // --- 订阅者 & 发布者 ---
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr game_status_sub_;
    rclcpp::Publisher<autoaim_interfaces::msg::GreenDot>::SharedPtr target_pub_;

    // --- 比赛状态 ---
    std::atomic<bool> game_started_{false};

    // --- 检测算法实例 ---
    std::shared_ptr<GreenDotDetect> detector_;

    // --- 参数回调句柄 ---
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // --- 相机内参 (只读一次) ---
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // --- 检测参数 ---
    DetectParams detect_params_;

    // --- FPS 统计 ---
    int fps_counter_ = 0;
    std::chrono::steady_clock::time_point last_fps_time_;

    // --- 保存控制 ---
    std::chrono::steady_clock::time_point last_save_time_;
    double save_fps_ = 60.0;
    std::chrono::milliseconds save_interval_{1000 / 60};
    std::string save_path_ = "/home/nvidia/saved_green_dots";

    // --- 调试开关 ---
    bool debug_mode_ = false;
    bool save_images_ = false;
    bool use_game_status_ = true;

    // --- 异步保存线程 ---
    std::thread save_thread_;
    std::mutex save_mutex_;
    std::condition_variable save_cv_;
    std::queue<cv::Mat> save_queue_;
    std::atomic<bool> stop_save_thread_{false};
  };

} // namespace detect_base
