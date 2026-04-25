/**
 * @file SystemMonitorNode.cpp
 * @brief 系统监控节点实现——状态收集、超时检测与日志记录
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#include "system_monitor/SystemMonitorNode.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace system_monitor
{

  SystemMonitorNode::SystemMonitorNode(const rclcpp::NodeOptions &options)
      : Node("system_monitor_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing SystemMonitorNode...");

    // 声明参数
    log_path_ = this->declare_parameter("log_path", "./logs");
    check_interval_ms_ = this->declare_parameter("check_interval_ms", 1000);
    can_timeout_ms_ = this->declare_parameter("can_timeout_ms", 2000);

    // 初始化状态
    last_can_hw_state_ = 0;
    green_dot_detected_ = false;
    green_dot_x_ = 0.0;
    green_dot_y_ = 0.0;
    can_hw_state_received_ = false;
    green_dot_received_ = false;

    // 初始化时间跟踪，避免首次回调时未定义行为
    startup_time_ = this->now();
    last_can_hw_state_time_ = this->now();
    last_green_dot_time_ = this->now();

    // 生成带时间戳的日志文件名
    std::string log_filename = generateLogFilename();
    std::filesystem::create_directories(log_path_);

    // 打开日志文件
    log_file_.open(log_filename, std::ios::app);
    if (!log_file_.is_open())
    {
      RCLCPP_WARN(this->get_logger(), "Failed to open log file: %s", log_filename.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Log file opened: %s", log_filename.c_str());
      writeLog("=== System Monitor Started ===");
    }

    // 订阅绿灯检测话题
    green_dot_sub_ = this->create_subscription<autoaim_interfaces::msg::GreenDot>(
        "/detections/green_dots", rclcpp::SensorDataQoS(),
        std::bind(&SystemMonitorNode::greenDotCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to /detections/green_dots");

    // 订阅CAN硬件状态话题
    can_hw_state_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/can_hardware_state", 10,
        std::bind(&SystemMonitorNode::canHwStateCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to /can_hardware_state");

    // 创建定时器，定期检查状态并写入日志
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(check_interval_ms_),
        std::bind(&SystemMonitorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "SystemMonitorNode initialized successfully");
  }

  SystemMonitorNode::~SystemMonitorNode()
  {
    if (log_file_.is_open())
    {
      writeLog("=== System Monitor Stopped ===");
      log_file_.close();
    }
    RCLCPP_INFO(this->get_logger(), "SystemMonitorNode destroyed");
  }

  void SystemMonitorNode::greenDotCallback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg)
  {
    green_dot_received_ = true;
    last_green_dot_time_ = this->now();

    // 判断是否检测到绿灯（x >= 0 表示检测到目标）
    if (msg->x >= 0.0)
    {
      green_dot_detected_ = true;
      green_dot_x_ = msg->x;
      green_dot_y_ = msg->y;
    }
    else
    {
      green_dot_detected_ = false;
      green_dot_x_ = 0.0;
      green_dot_y_ = 0.0;
    }
  }

  void SystemMonitorNode::canHwStateCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    uint8_t state = msg->data;
    // 验证状态值是否在已知范围内，未知状态视为未收到有效数据
    bool valid = (state <= 5 || state == 255);
    if (!valid)
    {
      RCLCPP_WARN(this->get_logger(), "收到未知 CAN 状态码: %u, 忽略",
                   static_cast<unsigned>(state));
      return;
    }
    can_hw_state_received_ = true;
    last_can_hw_state_time_ = this->now();
    last_can_hw_state_ = state;
    RCLCPP_DEBUG(this->get_logger(), "CAN hw state received: %u", static_cast<unsigned>(state));
  }

  void SystemMonitorNode::timerCallback()
  {
    // 启动宽限期：前3秒跳过CAN超时检测，等待DDS连接建立
    auto uptime_ms = (this->now() - startup_time_).seconds() * 1000.0;
    bool in_startup_grace = uptime_ms < 3000.0;

    // 检查CAN硬件状态超时（如果超过can_timeout_ms_没有收到状态更新，认为CanSerialNode挂了）
    if (can_hw_state_received_ && !in_startup_grace)
    {
      auto now = this->now();
      auto elapsed_ms = (now - last_can_hw_state_time_).seconds() * 1000.0;
      if (elapsed_ms > can_timeout_ms_)
      {
        last_can_hw_state_ = 255; // 自定义：超时未更新
      }
    }

    // 检查绿灯检测超时
    if (green_dot_received_)
    {
      auto now = this->now();
      auto elapsed_ms = (now - last_green_dot_time_).seconds() * 1000.0;
      if (elapsed_ms > 2000)
      {
        green_dot_detected_ = false;
      }
    }

    // 写入日志
    checkAndWriteLog();
  }

  void SystemMonitorNode::checkAndWriteLog()
  {
    // 构建CAN状态字符串
    std::string can_status_str;
    if (can_hw_state_received_)
    {
      can_status_str = hwStateToString(last_can_hw_state_);
      if (can_status_str == "UNKNOWN")
      {
        RCLCPP_WARN(this->get_logger(), "CAN hardware state UNKNOWN (raw=%u), possible initialization issue",
                     static_cast<unsigned>(last_can_hw_state_));
      }
    }
    else
    {
      can_status_str = "NO_DATA";
    }

    // 构建绿灯状态字符串
    std::string green_dot_status;
    if (!green_dot_received_)
    {
      green_dot_status = "NO_DATA";
    }
    else if (green_dot_detected_)
    {
      std::ostringstream oss;
      oss << "DETECTED (x=" << std::fixed << std::setprecision(1)
          << green_dot_x_ << ", y=" << green_dot_y_ << ")";
      green_dot_status = oss.str();
    }
    else
    {
      green_dot_status = "LOST";
    }

    std::string log_message = "CAN: " + can_status_str + " | GREEN_DOT: " + green_dot_status;

    // 写入日志
    writeLog(log_message);

    // 同时输出到ROS日志
    // RCLCPP_INFO(this->get_logger(), "%s", log_message.c_str());
  }

  std::string SystemMonitorNode::hwStateToString(uint8_t state)
  {
    switch (state)
    {
    case 0:
      return "ONLINE";
    case 1:
      return "WARNING";
    case 2:
      return "PASSIVE";
    case 3:
      return "BUS_OFF";
    case 4:
      return "IF_DOWN";
    case 5:
      return "NO_SLAVE";
    case 255:
      return "TIMEOUT";
    default:
      return "UNKNOWN";
    }
  }

  std::string SystemMonitorNode::generateLogFilename()
  {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;

    std::stringstream ss;
    ss << log_path_ << "/system_monitor_"
       << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(3) << ms.count()
       << ".log";
    return ss.str();
  }

  void SystemMonitorNode::writeLog(const std::string &message)
  {
    if (!log_file_.is_open())
    {
      return;
    }

    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;

    // 格式化时间戳
    std::stringstream ss;
    ss << "[" << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
       << "." << std::setfill('0') << std::setw(3) << ms.count() << "] "
       << message;

    // 写入文件
    log_file_ << ss.str() << std::endl;
    log_file_.flush();
  }

} // namespace system_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(system_monitor::SystemMonitorNode)
