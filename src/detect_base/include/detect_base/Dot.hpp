#pragma once
#include <opencv2/opencv.hpp>
struct Dot
{
  cv::Point2f center;  // 原始分辨率下的质心坐标
  cv::Rect roi;        // 原始分辨率下的外接矩形
  double area;         // 轮廓面积
  double yaw;          // 计算出的偏航角 (度)
};