/**
 * @file SystemMonitorNode.hpp
 * @brief 系统监控节点，记录 CAN 通信状态和绿灯检测状态的日志
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#pragma once

#include <fstream>
#include <memory>
#include <string>

#include "autoaim_interfaces/msg/green_dot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace system_monitor
{

  /**
   * @brief 系统监控节点
   *
   * 订阅 CAN 硬件状态和绿灯检测结果，定期将状态信息写入带时间戳的日志文件。
   * 支持 CAN 通信超时检测和绿灯丢失检测。
   */
  class SystemMonitorNode : public rclcpp::Node
  {
  public:
    explicit SystemMonitorNode(const rclcpp::NodeOptions &options);
    ~SystemMonitorNode();

  private:
    /** @brief 绿灯检测结果回调 */
    void greenDotCallback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg);

    /** @brief CAN 硬件状态回调 */
    void canHwStateCallback(const std_msgs::msg::UInt8::SharedPtr msg);

    /** @brief 定时器回调——检查超时并写入日志 */
    void timerCallback();

    /** @brief 写入日志消息 */
    void writeLog(const std::string &message);

    /** @brief 检查状态并构建日志条目 */
    void checkAndWriteLog();

    /** @brief 生成带时间戳的日志文件名 */
    std::string generateLogFilename();

    /** @brief 将 CAN 控制器状态码转换为可读字符串 */
    std::string hwStateToString(uint8_t state);

    // --- 订阅者 ---
    rclcpp::Subscription<autoaim_interfaces::msg::GreenDot>::SharedPtr green_dot_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr can_hw_state_sub_;

    // --- 定时器 ---
    rclcpp::TimerBase::SharedPtr timer_;

    // --- 状态变量 ---
    uint8_t last_can_hw_state_;
    bool green_dot_detected_;
    double green_dot_x_;
    double green_dot_y_;
    bool can_hw_state_received_;
    bool green_dot_received_;

    // --- 日志文件 ---
    std::ofstream log_file_;
    std::string log_path_;

    // --- 参数 ---
    int check_interval_ms_;
    int can_timeout_ms_;

    // --- 时间跟踪 ---
    rclcpp::Time startup_time_;
    rclcpp::Time last_can_hw_state_time_;
    rclcpp::Time last_green_dot_time_;
    rclcpp::Time last_log_time_;
  };

} // namespace system_monitor
