/**
 * @file CanSerialCore.cpp
 * @brief CAN 总线底层通信类实现
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#include "can_serial/CanSerialCore.hpp"

namespace can_serial
{

CanSerial::CanSerial(const std::string & interface)
: can_interface_(interface), stream_(io_service_)
{
}

CanSerial::~CanSerial()
{
  if (sock_ >= 0) close(sock_);
  io_service_.stop();
  if (io_thread_.joinable()) {
    io_thread_.join();
    std::cout << "Boost.Asio 线程已停止" << std::endl;
  }
}

void CanSerial::init()
{
  // 创建 CAN 套接字
  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) throw std::runtime_error("socket failed");

  // 关闭本地回环模式
  int loopback = 0;
  setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  // 启用 CAN 错误帧上报，接收硬件层总线错误
  can_err_mask_t err_mask = CAN_ERR_MASK;
  setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

  // 绑定 CAN 接口
  ifreq ifr{};
  strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ);
  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
    throw std::runtime_error("获取接口索引失败: " + std::string(strerror(errno)));

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    throw std::runtime_error("绑定失败: " + std::string(strerror(errno)));

  // 将套接字分配给 Boost.Asio 流
  stream_.assign(sock_);
}

void CanSerial::async_read()
{
  stream_.async_read_some(
    boost::asio::buffer(&recv_frame_, sizeof(can_frame)),
    [this](auto &&... args) { handle_received(args...); });
}

void CanSerial::handle_received(const boost::system::error_code & ec, size_t bytes)
{
  if (!ec && bytes == sizeof(can_frame)) {
    if (recv_frame_.can_id & CAN_ERR_FLAG) {
      handle_error_frame(recv_frame_);
      async_read();
      return;
    }
    if (frame_callback_) frame_callback_(recv_frame_);
    async_read();
  } else if (ec) {
    if (ec.value() == ENODEV || ec.value() == ENETDOWN) {
      controller_state_ = 4;
    }
    std::cout << "读取错误: " << ec.message() << std::endl;
  }
}

void CanSerial::handle_error_frame(const can_frame & frame)
{
  if (frame.can_id & CAN_ERR_CRTL) {
    // data[1] 低 2 位是控制器状态 (0-3)，高位是溢出等错误标志
    // 只取低 2 位避免将溢出标志误认为状态值
    int raw = frame.data[1] & 0x03;
    if (raw <= 3) {
      controller_state_ = raw;
    }
  }
  if (frame.can_id & CAN_ERR_BUSOFF) {
    controller_state_ = 3;
  }
  if (frame.can_id & CAN_ERR_ACK) {
    no_ack_detected_.store(true);
  }
}

bool CanSerial::is_interface_up()
{
  struct ifreq ifr{};
  strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ);
  if (ioctl(sock_, SIOCGIFFLAGS, &ifr) < 0) return false;
  return (ifr.ifr_flags & IFF_UP) && (ifr.ifr_flags & IFF_RUNNING);
}

void CanSerial::start_io_service()
{
  io_thread_ = std::thread([this]() {
    std::cout << "启动 Boost.Asio 事件循环..." << std::endl;
    io_service_.run();
    std::cout << "Boost.Asio 事件循环已停止" << std::endl;
  });
}

void CanSerial::send_frame(const can_frame & frame)
{
  stream_.write_some(boost::asio::buffer(&frame, sizeof(frame)));
}

void CanSerial::set_frame_callback(FrameCallback callback)
{
  frame_callback_ = std::move(callback);
}

}  // namespace can_serial
