#ifndef UART_SERIAL_NODE_HPP_
#define UART_SERIAL_NODE_HPP_

#include <serial/serial.h>

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "autoaim_interfaces/msg/green_dot.hpp"
#include "uart_serial/ScrewPid.hpp"

namespace uart_serial
{
class UartSerialNode : public rclcpp::Node
{
public:
  explicit UartSerialNode(const rclcpp::NodeOptions & options);
  bool send_command(double speed, bool fire);
  bool handle_serial_frame();
  ~UartSerialNode();

private:
  void green_dots_callback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Subscription<autoaim_interfaces::msg::GreenDot>::SharedPtr green_dots_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<uint8_t> serial_buffer_;
  void parse_received_data();

  serial::Serial serial_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  enum class AimState { TRACKING, VERIFYING, LOCKED };
  AimState current_state_ = AimState::TRACKING;
  std::vector<double> history_x_;

  int lost_frames_count_ = 0;
  int16_t last_valid_speed_ = 0;

  const int SCALE = 100;
  const int MAX_LOST_TOLERANCE = 5;
  const size_t VERIFY_FRAMES = 5;

  ScrewPID my_pid_{0.8, 0.1, 0.05, 5.0, 100.0};

  struct
  {
    double Kp;
    double Ki;
    double Kd;
    double max_speed;
    double deadzone;
  } pid_params_;

  rclcpp::Time last_time_;
  rclcpp::Duration dt = rclcpp::Duration::from_seconds(0);
  bool first_run_ = true;
};
}  // namespace uart_serial
#endif  // UART_SERIAL_NODE_HPP_