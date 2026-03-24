#ifndef CAN_SERIAL__CAN_SERIAL_NODE_HPP_
#define CAN_SERIAL__CAN_SERIAL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "autoaim_interfaces/msg/green_dot.hpp"
#include "can_serial/ScrewPid.hpp"
#include "CanSerialCore.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace can_serial
{
class CanSerialNode : public rclcpp::Node
{
public:
  explicit CanSerialNode(const rclcpp::NodeOptions & options);
  void send_command(can_frame& frame, double speed, bool fire);
  void handle_can_frame(const can_frame & frame);

private:
  void green_dots_callback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg);
   std::string to_binary_string(uint8_t value);

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Subscription<autoaim_interfaces::msg::GreenDot>::SharedPtr green_dots_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  
  std::unique_ptr<CanSerial> can_core_;
  //void parse_received_data();

  //serial::Serial serial_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  enum class AimState { TRACKING, VERIFYING, LOCKED };
  AimState current_state_ = AimState::TRACKING;
  std::vector<double> history_x_;

  int lost_frames_count_ = 0;
  int16_t last_valid_speed_ = 0;

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


  //int test_count = 0;
};
// 辅助函数：将 uint8_t 转换为二进制字符串
std::string CanSerialNode::to_binary_string(uint8_t value)
{
  std::ostringstream oss;
  for (int i = 7; i >= 0; --i)  // 从最高位到最低位
  {
    oss << ((value >> i) & 1);  // 提取第 i 位
  }
  return oss.str();
}
} 
#endif 