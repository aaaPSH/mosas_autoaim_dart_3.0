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

  // timer_ = this->create_wall_timer(
  //   std::chrono::milliseconds(10), std::bind(&CanSerialNode::handle_can_frame, this));
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
// void CanSerialNode::parse_received_data()
// {
//   const size_t FRAME_LENGTH = 6; 

//   while (rx_buffer_.size() >= FRAME_LENGTH) {
//     if (rx_buffer_[0] == 0x55 && rx_buffer_[1] == 0xAA) {
//       uint8_t checksum = 0;
//       for (size_t i = 0; i < FRAME_LENGTH - 1; i++) {
//         checksum += rx_buffer_[i];
//       }

//       if (checksum == rx_buffer_[FRAME_LENGTH - 1]) {
//         // 校验成功，提取数据并处理






//         rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + FRAME_LENGTH);
//       } else {
//         // 校验和失败：可能是误判的帧头，丢弃当前字节（或者整帧），继续往后找
//         RCLCPP_WARN(this->get_logger(), "串口接收校验和错误！");
//         rx_buffer_.erase(rx_buffer_.begin()); // 弹出第一个字节，重新对齐
//       }
//     } else {
//       // 当前头部不是 0x55，说明数据错位，弹出一个字节，继续寻找正确帧头
//       rx_buffer_.erase(rx_buffer_.begin());
//     }
//   }
// }

void CanSerialNode::handle_can_frame(const can_frame & frame){

  RCLCPP_INFO(this->get_logger(),"已经收到消息");
  RCLCPP_INFO(get_logger(), "收到CAN帧 - ID: 0x%x, 长度: %d", frame.can_id, frame.can_dlc);
    // 仅处理ID为0xA0的帧
    if (frame.can_id == 0x100 && frame.can_dlc >= 7)
    {
      std::stringstream ss;
      for (size_t i = 0; i < frame.can_dlc; ++i) {
          ss << std::setw(2) << std::setfill('0') << std::hex 
            << static_cast<int>(frame.data[i]) << " ";
      }
      RCLCPP_INFO(this->get_logger(), "CAN Received -> %s", ss.str().c_str());
    }
}

void CanSerialNode::send_command(can_frame& frame, double speed, bool fire)
{
  std::memset(frame.data, 0, sizeof(frame.data));
  int16_t speed_int = static_cast<int16_t>(speed);

  frame.data[0] = 0xAA;
  frame.data[1] = 0x55;
  frame.data[2] = fire ? 0x01 : 0x00;
  

  frame.data[3] = speed_int & 0xFF;
  frame.data[4] = (speed_int >> 8) & 0xFF;
  frame.data[5] = 0x00;
  

  uint8_t checksum = 0;
  for (int i = 0; i < frame.can_dlc-2; i++) {
    checksum += frame.data[i];
  }
  frame.data[6] = checksum;
  frame.data[7] = 0x01;

  try {
    can_core_->send_frame(frame);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "CAN 发送异常: %s", e.what());
  }
}


void CanSerialNode::green_dots_callback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg)
{

  can_frame frame;
  frame.can_id = 0x106;
  frame.can_dlc = 8;

  rclcpp::Time current_time = msg->header.stamp;
  double current_x = msg->x;
  double current_error = msg->d_pixel;

  if (first_run_) {
    last_time_ = current_time;
    first_run_ = false;
    return;
  }

  // ==========================================
  // 丢帧拦截与惯性滑行机制
  // ==========================================
  if (current_x < 0.0) {
    lost_frames_count_++;
    if (lost_frames_count_ <= MAX_LOST_TOLERANCE) {
      RCLCPP_INFO(
        this->get_logger(), "丢帧：（%d/%d) %.2d", lost_frames_count_, MAX_LOST_TOLERANCE,
        last_valid_speed_);
      send_command(frame, last_valid_speed_, false);
    } else {
      send_command(frame, 0, false);
      //test_count++;
      RCLCPP_INFO(this->get_logger(), "NO TARGET!");
      my_pid_.reset();
      current_state_ = AimState::TRACKING;
    }
    return;
  }

  // ==========================================
  // 2.自动吸收丢帧时间
  // ==========================================
  lost_frames_count_ = 0;
  double dt = (current_time - last_time_).seconds();

  if (dt > 0.5) {
    dt = 0.014;
    my_pid_.reset();
  }

  last_time_ = current_time;

  // ==========================================
  // 3.动静分离状态机
  // ==========================================
  switch (current_state_) {
    case AimState::TRACKING: {
      double target_speed = my_pid_.compute(current_error, dt);

      if (target_speed == 0.0) {
        send_command(frame, 0, false);
        current_state_ = AimState::VERIFYING;
        history_x_.clear();
        RCLCPP_INFO(this->get_logger(), "进入死区，刹车并开始多帧核实...");
      } else {
        send_command(frame, target_speed, false);
        last_valid_speed_ = target_speed;
      }
      break;
    }

    case AimState::VERIFYING: {
      history_x_.push_back(current_x);

      if (history_x_.size() >= VERIFY_FRAMES) {
        double sum = 0;
        for (double x : history_x_) sum += x;
        double avg_x = sum / VERIFY_FRAMES;
        if (std::abs(avg_x) <= pid_params_.deadzone) {
          current_state_ = AimState::LOCKED;
          RCLCPP_INFO(this->get_logger(), "完美锁定！真理均值坐标: %.2f", avg_x);
        } else {
          current_state_ = AimState::TRACKING;
          history_x_.clear();
          my_pid_.reset();
          RCLCPP_WARN(this->get_logger(), "假锁定(均值%.2f)，打回 TRACKING 继续微调", avg_x);
        }
      }
      break;
    }

    case AimState::LOCKED: {
      send_command(frame, 0, true);
      if (std::abs(current_error) > 3.0) {
        current_state_ = AimState::TRACKING;
        my_pid_.reset();
        RCLCPP_WARN(this->get_logger(), "重新瞄准");
      }
      break;
    }
  }
}

}  // namespace uart_serial

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(can_serial::CanSerialNode)