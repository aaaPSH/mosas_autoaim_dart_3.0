#pragma once
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

#include "detect_dl_base/Dot.hpp"

// ==========================================
// 数据结构定义
// ==========================================

/**
 * @brief 检测参数配置结构体
 * 针对 25m 距离 55mm 目标给出了建议默认值
 */
struct DetectParams
{
  // --- 几何与ROI参数 ---
  double target_height = 600.0;  // 目标离地高度 (mm)
  double camera_height = 0.0;    // 相机离地高度 (mm)
  double distance = 2500.0;      // 预估距离 (mm)
  double detect_scale = 5.0;     // 搜索条带的垂直视场角范围 (度)
  int search_strip_min_h = 30;   // 搜索条带最小像素高度 (防止过窄)

  // --- 阈值与形态学 ---
  int v_low = 210;  // 绿色通道二值化阈值 (需根据曝光调整)

  // --- 几何筛选 (针对半分辨率图) ---
  double min_area = 5.0;   // 最小面积 (对应原图上的像素数，代码中会 /4 处理)
  double max_area = 30.0;  // 最大面积
  double min_aspect_ratio = 0.5;  // 最小长宽比
  double max_aspect_ratio = 2.0;  // 最大长宽比
  double min_circularity = 0.6;  // 最小圆形度 (小目标像素化严重，圆形度不会太高)

  // --- 颜色比率 (Bayer域) ---
  double min_gr_ratio = 3.0;  // Green/Red 能量比
  double min_gb_ratio = 1.5;  // Green/Blue 能量比

  // --- 标定修正 ---
  double yaw_offset = 0.0;  // 偏航角补偿 (度)
};

/**
 * @brief 检测结果结构体
 */

// ==========================================
// 核心检测类
// ==========================================

class GreenDotDetect
{
public:
  GreenDotDetect() = default;
  ~GreenDotDetect() = default;

  /**
     * @brief 初始化相机内参
     * @param cameraMatrix 3x3 内参矩阵
     * @param distCoeffs 畸变系数
     */
  void init_camera(const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs);

  /**
     * @brief 更新检测参数
     */
  void update_params(const DetectParams & new_params);

  /**
     * @brief 执行检测
     * @param raw_image 输入图像 (必须是单通道 Bayer Raw, 假设为 BayerRG 格式)
     * @param dots 输出检测到的灯点列表
     * @param debug 是否开启可视化调试窗口
     * @return true 检测到至少一个目标
     */
  bool detect(const cv::Mat & raw_image, std::vector<Dot> & dots, bool debug = false);

  /**
     * @brief 计算所有目标的偏航角
     * 结果直接写入 dots 结构体中的 yaw 字段
     */
  bool calculate_dots_yaw(std::vector<Dot> & dots);

private:
  // --- 内部成员变量 ---
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  DetectParams params;

  // --- 内存复用缓存 (优化性能) ---
  cv::Mat half_green;  // 用于存储降维后的绿色通道

  // --- 内部辅助函数 ---
  /**
     * @brief 计算亚像素质心 (灰度重心法)
     */
  cv::Point2f computeSubPixelCenter(const cv::Mat & roi_img, const cv::Rect & local_rect);
};