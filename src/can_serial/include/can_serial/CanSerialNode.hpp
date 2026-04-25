/**
 * @file CanSerialNode.hpp
 * @brief CAN 串行通信 ROS 2 节点，负责与下位机通过 CAN 总线交换控制指令和状态
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>
#include <vector>

#include "autoaim_interfaces/msg/green_dot.hpp"
#include "can_serial/CanSerialCore.hpp"
#include "can_serial/ScrewPid.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace can_serial
{

/**
 * @brief CAN 串行通信节点
 *
 * 接收来自检测节点的 GreenDot 消息，通过 PID 控制器计算云台速度指令，
 * 经 CAN 总线发送给下位机。同时接收下位机状态信息并上报 CAN 硬件状态。
 */
class CanSerialNode : public rclcpp::Node
{
public:
  explicit CanSerialNode(const rclcpp::NodeOptions & options);

  /**
   * @brief 向下位机发送速度与开火指令
   * @param speed 云台旋转速度
   * @param fire 是否开火
   */
  void send_command();

  /**
   * @brief 处理接收到的 CAN 帧
   * @param frame 收到的 CAN 帧
   */
  void handle_can_frame(const can_frame & frame);

private:
  /** @brief 收到 GreenDot 检测结果时的回调 */
  void green_dots_callback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg);

  /** @brief 将 uint8_t 转换为 8 位二进制字符串（调试用） */
  std::string to_binary_string(uint8_t value);

  /** @brief 动态参数更新回调 */
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  /** @brief 定时上报 CAN 硬件状态 */
  void publish_can_hw_state();

  /** @brief 发送探测帧（不受 calibrated 限制） */
  void send_probe();


  // --- 订阅者与发布者 ---
  rclcpp::Subscription<autoaim_interfaces::msg::GreenDot>::SharedPtr green_dots_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr can_hw_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_status_pub_;

  // --- 定时器 ---
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr can_state_timer_;
  rclcpp::TimerBase::SharedPtr probe_timer_;

  // --- CAN 通信 ---
  std::unique_ptr<CanSerial> can_core_;
  can_frame frame_;
  std::mutex data_mutex_;

  // --- 游戏状态枚举 ---
  /** @brief 比赛阶段 */
  enum class GameStatus : uint8_t {
    PRE_PREPARATION = 0,   ///< 比赛未开始
    PREPARATION     = 1,   ///< 准备阶段
    SELF_CHECK      = 2,   ///< 自检阶段
    START_COUNTDOWN = 3,   ///< 5s 倒计时
    IN_GAME         = 4,   ///< 比赛中
    END_GAME        = 5,   ///< 比赛结束
  };

  /** @brief 瞄准状态机 */
  enum class AimState { TRACKING, VERIFYING, LOCKED, LOST };

  // --- 状态变量 ---
  AimState current_state_ = AimState::LOST;
  std::vector<double> history_x_;
  int lost_frames_count_ = 0;
  int16_t last_valid_speed_ = 0;
  bool first_run_ = true;

  //下位机发送的变量
  struct send_command
  {
    bool can_shoot = false;
    double speed = 0.0;
    bool detected = false;
  } s_command_;

  struct get_command
  {
    bool calibrated = false;
    GameStatus current_game_status = GameStatus::PRE_PREPARATION;
  } g_command_;
  

  // --- 参数回调句柄 ---
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // --- CAN 通信常量 ---
  static constexpr uint32_t CAN_TX_ID = 0x106;
  static constexpr uint32_t CAN_RX_ID = 0x100;
  static constexpr uint8_t CAN_FRAME_DLC = 8;
  static constexpr uint8_t CAN_MIN_RX_DLC = 7;
  static constexpr uint8_t FIRE_ON = 0x01;
  static constexpr uint8_t FIRE_OFF = 0x00;

  // --- 控制逻辑常量 ---
  static constexpr double SCALE = 100;
  static constexpr double DEADZONE_MULTIPLIER = 1.5;

  // --- 容错参数 ---
  static constexpr int MAX_LOST_TOLERANCE = 5;
  static constexpr size_t VERIFY_FRAMES = 5;

  // --- PID 控制器 ---
  ScrewPID my_pid_{0.8, 0.1, 0.05, 5.0, 100.0};

  /// @brief PID 参数集（可从 ROS 参数服务器动态更新）
  struct PIDParams
  {
    double Kp;
    double Ki;
    double Kd;
    double max_speed;
    double deadzone;
  } pid_params_;

  rclcpp::Time last_time_;
  rclcpp::Duration dt_ = rclcpp::Duration::from_seconds(0);

  // --- 下位机在线检测 ---
  bool slave_alive_{false};
  rclcpp::Time last_slave_rx_time_;
};

}  // namespace can_serial
