/**
 * @file HikCameraNode.hpp
 * @brief 海康威视工业相机 ROS 2 驱动节点
 *
 * 通过海康 MVS SDK 控制相机，发布 Bayer Raw 图像话题 /image_raw，
 * 同时将 Bayer 转换为 RGB 发布到 /save_frame 供录制使用。
 * 使用零拷贝 Loaned Message 机制发布 Raw 图像以减少内存拷贝。
 *
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#pragma once

#include <algorithm>
#include <iostream>
#include <thread>
#include <vector>

#include "MvCameraControl.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace hik_publisher
{

  /**
   * @brief 海康相机 ROS 2 节点
   *
   * 封装海康 MVS SDK，实现相机枚举、打开、参数配置、图像抓取和发布。
   * 支持动态参数设置曝光时间和增益。
   */
  class HikCameraNode : public rclcpp::Node
  {
  public:
    explicit HikCameraNode(const rclcpp::NodeOptions &options);
    ~HikCameraNode();

  private:
    /** @brief 图像抓取主循环（运行在独立线程） */
    void captureLoop();

    /** @brief 声明并初始化相机参数 */
    void declareParams();

    /** @brief 动态参数回调 */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

    // --- 发布者 ---
    /// @brief Raw Bayer 图像发布者（零拷贝） */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;

    /// @brief RGB 录制图像发布者 */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr save_pub_;

    // --- 图像缓冲区 ---
    sensor_msgs::msg::Image rgb_msg_;
    std::vector<uint8_t> raw_buffer_;

    // --- 相机句柄与参数 ---
    void *camera_handle_ = nullptr;
    MV_IMAGE_BASIC_INFO img_info_{};
    MV_CC_PIXEL_CONVERT_PARAM convert_param_{};
    int last_result_ = MV_OK;

    // --- 线程 ---
    std::thread capture_thread_;

    // --- 参数回调句柄 ---
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  };

} // namespace hik_publisher
