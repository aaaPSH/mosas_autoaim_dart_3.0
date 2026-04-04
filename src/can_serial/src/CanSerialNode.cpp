#include "can_serial/CanSerialNode.hpp"

#include <cstring>

namespace can_serial
{
CanSerialNode::CanSerialNode(const rclcpp::NodeOptions & options)
: Node("can_serial_node", options)
{
  // ================= [ 1. 声明并读取参数 ] =================
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
    pid_params_.Kp, pid_params_.Ki, pid_params_.Kd, pid_params_.deadzone, pid_params_.max_speed);

  // ================= [ 2. 注册动态参数回调 ] =================
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CanSerialNode::parameters_callback, this, std::placeholders::_1));

  // ================= [ 3. 初始化can ] =================
  // 初始化CAN驱动
  can_core_ = std::make_unique<CanSerial>("can0");
  try {
    can_core_->init();
    can_core_->set_frame_callback(
      std::bind(&CanSerialNode::handle_can_frame, this, std::placeholders::_1));
    can_core_->async_read();
    // 启动Boost.Asio事件循环线程
    can_core_->start_io_service();
    std::cout << "Boost.Asio线程已启动" << std::endl;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "CAN初始化失败: %s", e.what());
    throw;
  }

  // ================= [ 4. 初始化订阅者 ] =================
  green_dots_sub_ = this->create_subscription<autoaim_interfaces::msg::GreenDot>(
    "/detections/green_dots", rclcpp::SensorDataQoS(),
    std::bind(&CanSerialNode::green_dots_callback, this, std::placeholders::_1));

}
// ==========================================================
// 动态参数回调处理函数
// ==========================================================
rcl_interfaces::msg::SetParametersResult CanSerialNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Parameters updated successfully";

  for (const auto & param : parameters) {
    if (param.get_name() == "Kp") {
      pid_params_.Kp = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Kp updated to: %.4f", pid_params_.Kp);
    } else if (param.get_name() == "Ki") {
      pid_params_.Ki = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Ki updated to: %.4f", pid_params_.Ki);
    } else if (param.get_name() == "Kd") {
      pid_params_.Kd = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Kd updated to: %.4f", pid_params_.Kd);
    } else if (param.get_name() == "max_speed") {
      pid_params_.max_speed = param.as_double();
      RCLCPP_INFO(this->get_logger(), "max_speed updated to: %.1f", pid_params_.max_speed);
    } else if (param.get_name() == "deadzone") {
      pid_params_.deadzone = param.as_double();
      RCLCPP_INFO(this->get_logger(), "deadzone updated to: %.2f", pid_params_.deadzone);
    }
  }

  my_pid_.set_params(
    pid_params_.Kp, pid_params_.Ki, pid_params_.Kd, pid_params_.deadzone, pid_params_.max_speed);

  return result;
}
void CanSerialNode::parse_received_data()
{
  
}

void CanSerialNode::handle_can_frame(const can_frame & frame){

  // RCLCPP_INFO(this->get_logger(),"已经收到消息");
  // RCLCPP_INFO(get_logger(), "收到CAN帧 - ID: 0x%x, 长度: %d", frame.can_id, frame.can_dlc);
  //   // 仅处理ID为0xA0的帧
  //   if (frame.can_id == CAN_RX_ID && frame.can_dlc >= CAN_MIN_RX_DLC)
  //   {
  //     std::stringstream ss;
  //     for (size_t i = 0; i < frame.can_dlc; ++i) {
  //         ss << std::setw(2) << std::setfill('0') << std::hex 
  //           << static_cast<int>(frame.data[i]) << " ";
  //     }
  //     RCLCPP_INFO(this->get_logger(), "CAN Received -> %s", ss.str().c_str());
  //   }
}

void CanSerialNode::send_command(can_frame& frame, double speed, bool fire)
{
  frame.can_id = CAN_TX_ID;
  frame.can_dlc = CAN_FRAME_DLC;
  
  std::memset(frame.data, 0, sizeof(frame.data));
  int16_t speed_int = static_cast<int16_t>(speed);

  frame.data[0] = fire ? FIRE_ON : FIRE_OFF;
  frame.data[1] = (speed_int >> 8) & 0xFF;
  frame.data[2] = speed_int & 0xFF;
  

  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  try {
    can_core_->send_frame(frame);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "CAN 发送异常: %s", e.what());
  }
}


void CanSerialNode::green_dots_callback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg)
{
  can_frame frame;
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
  // [全局守卫]：无论在什么状态，优先处理目标丢失
  // ==========================================
  if (current_x < 0.0) {
    lost_frames_count_++;
    
    // 还在容忍范围内，保持最后一次的速度（惯性滑行）
    if (lost_frames_count_ <= MAX_LOST_TOLERANCE) {
      send_command(frame, last_valid_speed_, false);
      return; 
    } 
    else {
      RCLCPP_INFO(this->get_logger(), "状态切换: -> LOST");
      current_state_ = AimState::LOST;

    }
  } else {
    // 看到目标了，清零丢帧计数器
    lost_frames_count_ = 0;
  }

  // ==========================================
  // 3. 动静分离状态机 (只有在看到目标，或被判定为 LOST 时才会执行)
  // ==========================================
  switch (current_state_) {
    case AimState::TRACKING: {
      double target_speed = my_pid_.compute(current_error, dt);

      if (std::abs(current_error) == ZERO_ERROR_THRESHOLD) {
        send_command(frame, 0, false);
        RCLCPP_INFO(this->get_logger(), "状态切换: TRACKING -> VERIFYING");
        current_state_ = AimState::VERIFYING;
        history_x_.clear();
        history_x_.push_back(current_x); // 把当前的合格数据放进去
      } else {
        send_command(frame, target_speed, false);
        last_valid_speed_ = target_speed;
      }
      break;
    }

    case AimState::VERIFYING: {
      send_command(frame, 0, false);
      history_x_.push_back(current_x);

      if (history_x_.size() >= VERIFY_FRAMES) {
        double sum = 0;
        for (double x : history_x_) sum += x;
        double avg_x = sum / VERIFY_FRAMES;
        
        if (std::abs(avg_x) <= pid_params_.deadzone) {
          current_state_ = AimState::LOCKED;
          RCLCPP_INFO(this->get_logger(), "完美锁定!(均值%.2f)", avg_x);
        } else {
          current_state_ = AimState::TRACKING;
          history_x_.clear();
          my_pid_.reset();
          RCLCPP_WARN(this->get_logger(), "假锁定(均值%.2f)", avg_x);
        }
      }
      break;
    }

    case AimState::LOCKED: {
      send_command(frame, 0, true);
      if (std::abs(current_error) > pid_params_.deadzone * DEADZONE_MULTIPLIER) {
        RCLCPP_INFO(this->get_logger(), "状态切换: LOCKED -> TRACKING");
        current_state_ = AimState::TRACKING;
        my_pid_.reset();
      }
      break;
    }

    case AimState::LOST: {
      if (current_x >= 0.0) {
        // 目标重新出现了！
        RCLCPP_INFO(this->get_logger(), "状态切换: LOST -> TRACKING");
        current_state_ = AimState::TRACKING;
        my_pid_.reset();
        double target_speed = my_pid_.compute(current_error, dt);
        send_command(frame, target_speed, false);
        last_valid_speed_ = target_speed;
      } else {
        // 真的没看见，发 0 原地等待
        send_command(frame, 0, false);
        last_valid_speed_ = 0.0; 
      }
      break;
    }
  }
}

}  // namespace uart_serial

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(can_serial::CanSerialNode)
