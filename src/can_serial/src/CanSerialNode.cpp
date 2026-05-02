/**
 * @file CanSerialNode.cpp
 * @brief CAN 串行通信节点实现——状态机、CAN 收发与 PID 控制
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#include "can_serial/CanSerialNode.hpp"

#include <chrono>
#include <cstring>
#include <sstream>

namespace can_serial
{

// ============================================================================
// 辅助函数
// ============================================================================

std::string CanSerialNode::to_binary_string(uint8_t value)
{
  std::ostringstream oss;
  for (int i = 7; i >= 0; --i) {
    oss << ((value >> i) & 1);
  }
  return oss.str();
}

// ============================================================================
// 构造函数
// ============================================================================

CanSerialNode::CanSerialNode(const rclcpp::NodeOptions & options)
: Node("can_serial_node", options)
{
  // 初始化 CAN 帧数据为 0
  std::memset(&frame_, 0, sizeof(frame_));
  frame_.can_id = CAN_TX_ID;
  frame_.can_dlc = CAN_FRAME_DLC;

  // ================= [1. 声明并读取参数] =================
  this->declare_parameter("debug_level", 1);
  this->declare_parameter("Kp", 0.01);
  this->declare_parameter("Ki", 0.0);
  this->declare_parameter("Kd", 0.0);
  this->declare_parameter("max_speed", 100.0);
  this->declare_parameter("deadzone", 1.0);

  pid_params_.Kp = this->get_parameter("Kp").as_double();
  pid_params_.Ki = this->get_parameter("Ki").as_double();
  pid_params_.Kd = this->get_parameter("Kd").as_double();
  pid_params_.max_speed = this->get_parameter("max_speed").as_double();
  pid_params_.deadzone = this->get_parameter("deadzone").as_double();

  my_pid_.set_params(
    pid_params_.Kp, pid_params_.Ki, pid_params_.Kd,
    pid_params_.deadzone, pid_params_.max_speed);

  // ================= [2. 注册动态参数回调] =================
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CanSerialNode::parameters_callback, this, std::placeholders::_1));

  // ================= [3. 初始化 CAN 驱动] =================
  can_core_ = std::make_unique<CanSerial>("can0");
  try {
    can_core_->init();
    can_core_->set_frame_callback(
      std::bind(&CanSerialNode::handle_can_frame, this, std::placeholders::_1));
    can_core_->async_read();
    can_core_->start_io_service();
    std::cout << "Boost.Asio 线程已启动" << std::endl;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "CAN 初始化失败: %s", e.what());
    throw;
  }

  // 发布 CAN 硬件状态，每 500ms 上报一次
  can_hw_state_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/can_hardware_state", 10);
  game_status_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/game_status", 10);
  can_state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CanSerialNode::publish_can_hw_state, this));

  // ================= [4. 初始化订阅者] =================
  green_dots_sub_ = this->create_subscription<autoaim_interfaces::msg::GreenDot>(
    "/detections/green_dots", rclcpp::SensorDataQoS(),
    std::bind(&CanSerialNode::green_dots_callback, this, std::placeholders::_1));
}

// ============================================================================
// 动态参数回调
// ============================================================================

rcl_interfaces::msg::SetParametersResult CanSerialNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Parameters updated successfully";

  for (const auto & param : parameters) {
    if (param.get_name() == "Kp") {
      pid_params_.Kp = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Kp 已更新为: %.4f", pid_params_.Kp);
    } else if (param.get_name() == "Ki") {
      pid_params_.Ki = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Ki 已更新为: %.4f", pid_params_.Ki);
    } else if (param.get_name() == "Kd") {
      pid_params_.Kd = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Kd 已更新为: %.4f", pid_params_.Kd);
    } else if (param.get_name() == "max_speed") {
      pid_params_.max_speed = param.as_double();
      RCLCPP_INFO(this->get_logger(), "max_speed 已更新为: %.1f", pid_params_.max_speed);
    } else if (param.get_name() == "deadzone") {
      pid_params_.deadzone = param.as_double();
      RCLCPP_INFO(this->get_logger(), "deadzone 已更新为: %.2f", pid_params_.deadzone);
    }
  }

  my_pid_.set_params(
    pid_params_.Kp, pid_params_.Ki, pid_params_.Kd,
    pid_params_.deadzone, pid_params_.max_speed);

  return result;
}

// ============================================================================
// CAN 接收与解析
// ============================================================================


void CanSerialNode::handle_can_frame(const can_frame & frame)
{
  if (frame.can_id == CAN_RX_ID && frame.can_dlc >= CAN_MIN_RX_DLC) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    this->g_command_.calibrated = frame.data[3];
    this->g_command_.current_game_status = static_cast<GameStatus>(frame.data[5]);

    auto msg = std_msgs::msg::UInt8();
    msg.data = frame.data[5];
    game_status_pub_->publish(msg);
  }
}

// ============================================================================
// CAN 发送
// ============================================================================

void CanSerialNode::send_command()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  int16_t speed_int = static_cast<int16_t>(this->s_command_.speed * SCALE);
  frame_.data[0] = this->s_command_.detected;
  frame_.data[1] = (speed_int >> 8) & 0xFF;
  frame_.data[2] = speed_int & 0xFF;
  frame_.data[4] = this->s_command_.can_shoot ? FIRE_ON : FIRE_OFF;
  try {
    if(this->g_command_.calibrated){
      can_core_->send_frame(frame_);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "CAN 发送异常: %s", e.what());
  }
}

// ============================================================================
// GreenDot 检测结果回调——瞄准状态机
// ============================================================================

void CanSerialNode::green_dots_callback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg)
{
  rclcpp::Time current_time = msg->header.stamp;
  double current_x = msg->x;
  double current_error = msg->d_pixel;

  if (first_run_) {
    last_time_ = current_time;
    first_run_ = false;
    return;
  }

  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  // ==========================================
  // [全局守卫] 无论在什么状态，优先处理目标丢失
  // ==========================================
  if (current_x < 0.0) {
    lost_frames_count_++;

    // 还在容忍范围内，保持最后一次的速度（惯性滑行）
    if (lost_frames_count_ <= MAX_LOST_TOLERANCE) {
      send_command();
      return;
    } else {
      current_state_ = AimState::LOST;
    }
  } else {
    // 看到目标，清零丢帧计数器
    lost_frames_count_ = 0;
  }

  // ==========================================
  // 动静分离状态机
  // ==========================================
  switch (current_state_) {
    case AimState::TRACKING: {
      this->s_command_.speed = my_pid_.compute(current_error, dt);
      this->s_command_.can_shoot = false;
      this->s_command_.detected = true;

      if (std::abs(current_error) < pid_params_.deadzone) {
        RCLCPP_INFO(this->get_logger(), "状态切换: TRACKING -> VERIFYING");
        current_state_ = AimState::VERIFYING;
        history_x_.clear();
        history_x_.push_back(current_error);
      } 
      last_valid_speed_ = this->s_command_.speed;
      send_command();
      break;
    }

    case AimState::VERIFYING: {
      this->s_command_.speed = 0;
      this->s_command_.can_shoot = false;
      this->s_command_.detected = true;
      history_x_.push_back(current_error);

      if (history_x_.size() >= VERIFY_FRAMES) {
        double sum = 0;
        for (double x : history_x_) sum += x;
        double avg_x = sum / VERIFY_FRAMES;

        if (std::abs(avg_x) <= pid_params_.deadzone) {
          current_state_ = AimState::LOCKED;
          RCLCPP_INFO(this->get_logger(), "完美锁定! (均值 %.2f)", avg_x);
        } else {
          current_state_ = AimState::TRACKING;
          history_x_.clear();
          my_pid_.reset();
          RCLCPP_WARN(this->get_logger(), "假锁定 (均值 %.2f)", avg_x);
        }
      }
      send_command();
      break;
    }

    case AimState::LOCKED: {
      this->s_command_.speed = 0;
      this->s_command_.can_shoot = true;
      this->s_command_.detected = true;
      if (std::abs(current_error) > pid_params_.deadzone * DEADZONE_MULTIPLIER) {
        RCLCPP_INFO(this->get_logger(), "状态切换: LOCKED -> TRACKING");
        current_state_ = AimState::TRACKING;
        my_pid_.reset();
      }
      send_command();
      break;
    }

    case AimState::LOST: {
      if (current_x >= 0.0) {
        RCLCPP_INFO(this->get_logger(), "状态切换: LOST -> TRACKING");
        current_state_ = AimState::TRACKING;
        my_pid_.reset();
        this->s_command_.speed = my_pid_.compute(current_error, dt);
        this->s_command_.can_shoot = false;
        this->s_command_.detected = true;
        send_command();
        last_valid_speed_ = this->s_command_.speed;
      } else {
        this->s_command_.speed = 0;
        this->s_command_.can_shoot = false;
        this->s_command_.detected = false;
        send_command();
      }
      break;
    }
  }
}

// ============================================================================
// CAN 硬件状态上报
// ============================================================================

void CanSerialNode::publish_can_hw_state()
{
  auto msg = std_msgs::msg::UInt8();

  if (!can_core_->is_interface_up()) {
    msg.data = 4;  // INTERFACE_DOWN
  } else {
    msg.data = can_core_->get_controller_state();  // 0~3
  }
  can_hw_state_pub_->publish(msg);
}

}  // namespace can_serial

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(can_serial::CanSerialNode)
