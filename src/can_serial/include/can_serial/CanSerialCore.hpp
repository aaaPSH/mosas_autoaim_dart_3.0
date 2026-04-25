/**
 * @file CanSerialCore.hpp
 * @brief CAN 总线底层通信类，基于 Boost.Asio 封装 Linux SocketCAN 接口
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#pragma once

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/netlink.h>
#include <linux/can/raw.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <atomic>
#include <cstring>
#include <functional>
#include <iostream>
#include <string>
#include <thread>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

namespace can_serial
{

/**
 * @brief CAN 总线底层 I/O 类
 *
 * 封装 Linux SocketCAN 的创建、绑定、异步读写和错误处理。
 * 使用 Boost.Asio 进行异步 I/O 事件驱动。
 */
class CanSerial
{
public:
  /** @brief CAN 帧回调函数类型 */
  using FrameCallback = std::function<void(const can_frame &)>;

  /**
   * @brief 构造函数
   * @param interface CAN 网络接口名，默认 "can0"
   */
  explicit CanSerial(const std::string & interface = "can0");
  ~CanSerial();

  /** @brief 初始化 CAN 套接字，创建并绑定 SocketCAN 接口 */
  void init();

  /** @brief 启动一次异步读取，收到数据后自动递归调用 */
  void async_read();

  /** @brief 启动 Boost.Asio 事件循环（在独立线程中运行） */
  void start_io_service();

  /** @brief 发送 CAN 帧 */
  void send_frame(const can_frame & frame);

  /** @brief 设置接收回调函数 */
  void set_frame_callback(FrameCallback callback);

  /**
   * @brief 查询 CAN 接口是否已启动
   * @return true 表示接口 UP 且 RUNNING
   */
  bool is_interface_up();

  /**
   * @brief 获取 CAN 控制器状态
   * @return 0=ACTIVE, 1=WARNING, 2=PASSIVE, 3=BUS_OFF, 4=INTERFACE_DOWN
   */
  int get_controller_state() const { return controller_state_; }

  /** @brief 是否检测到 ACK 错误（总线上无设备响应） */
  bool get_no_ack() const { return no_ack_detected_.load(); }

  /** @brief 清除 ACK 错误标志 */
  void clear_no_ack() { no_ack_detected_.store(false); }

  /** @brief Boost.Asio I/O 线程 */
  std::thread io_thread_;

private:
  /**
   * @brief 处理收到的 CAN 帧
   * @param ec Boost 错误码
   * @param bytes 实际收到的字节数
   */
  void handle_received(const boost::system::error_code & ec, size_t bytes);

  /**
   * @brief 处理 CAN 错误帧
   * @param frame 收到的错误帧
   */
  void handle_error_frame(const can_frame & frame);

  std::string can_interface_;
  int sock_ = -1;
  boost::asio::io_service io_service_;
  boost::asio::posix::basic_stream_descriptor<> stream_;
  can_frame recv_frame_;
  FrameCallback frame_callback_;

  /** @brief CAN 控制器状态：0=ACTIVE, 1=WARNING, 2=PASSIVE, 3=BUS_OFF, 4=INTERFACE_DOWN */
  int controller_state_ = 0;

  /** @brief 是否检测到 ACK 错误（发送后无设备响应） */
  std::atomic<bool> no_ack_detected_{false};
};

}  // namespace can_serial
