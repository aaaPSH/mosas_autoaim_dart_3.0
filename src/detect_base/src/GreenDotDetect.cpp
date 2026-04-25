/**
 * @file GreenDotDetect.cpp
 * @brief 绿色灯点检测算法实现
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#include "detect_base/GreenDotDetect.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace detect_base
{

  /** @brief 角度转弧度常数函数，替代宏定义 */
  constexpr double to_radian(double x) { return x * CV_PI / 180.0; }

  // ==========================================
  // 1. 初始化与参数更新
  // ==========================================

  void GreenDotDetect::init_camera(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs)
  {
    this->camera_matrix_ = camera_matrix.clone();
    this->dist_coeffs_ = dist_coeffs.clone();
  }

  void GreenDotDetect::update_params(const DetectParams &new_params)
  {
    this->params_ = new_params;
  }

  // ==========================================
  // 2. 亚像素质心计算
  // ==========================================

  cv::Point2f GreenDotDetect::compute_sub_pixel_center(
      const cv::Mat &roi_img, const cv::Rect &local_rect)
  {
    // 确保 ROI 不越界
    cv::Rect safe_rect = local_rect & cv::Rect(0, 0, roi_img.cols, roi_img.rows);

    // 如果交集为空，返回几何中心
    if (safe_rect.empty())
    {
      return cv::Point2f(local_rect.width / 2.0f, local_rect.height / 2.0f);
    }

    cv::Mat roi = roi_img(safe_rect);
    cv::Moments mu = cv::moments(roi, false);

    // 防止除零（全黑区域）
    if (mu.m00 < DBL_EPSILON)
    {
      return cv::Point2f(safe_rect.x + safe_rect.width / 2.0f, safe_rect.y + safe_rect.height / 2.0f);
    }

    return cv::Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
  }

  // ==========================================
  // 3. [核心] Raw 数据检测
  // ==========================================

  bool GreenDotDetect::detect(const cv::Mat &raw_image, std::vector<Dot> &dots, bool debug)
  {
    // 基础检查：图像必须非空且为单通道
    if (raw_image.empty() || raw_image.channels() != 1)
      return false;
    dots.clear();

    // --- 步骤 1: 降维 (Bayer -> Green Energy) ---
    int w = raw_image.cols;
    int h = raw_image.rows;

    // 安全性：确保高度为偶数，防止 ptr(y+1) 越界
    if (h % 2 != 0)
      h -= 1;

    // 半分辨率图像，复用成员缓存避免每帧分配
    half_green_.create(h / 2, w / 2, CV_8UC1);
    half_green_.setTo(cv::Scalar(0));

    // 快速提取绿色通道能量 (BayerRG 格式下，偶数行是 R G，奇数行是 G B)
    // 取相邻两行的 G 分量平均值作为该点的亮度
    for (int y = 0; y < h; y += 2)
    {
      const uchar *r0 = raw_image.ptr<uchar>(y);
      const uchar *r1 = raw_image.ptr<uchar>(y + 1);
      uchar *d = half_green_.ptr<uchar>(y / 2);
      for (int x = 0; x < w; x += 2)
      {
        // RGGB 布局: r0[x+1] 是 G, r1[x] 是 G
        d[x / 2] = static_cast<uchar>((static_cast<int>(r0[x + 1]) + static_cast<int>(r1[x])) >> 1);
      }
    }

    // --- 步骤 2: 物理 ROI 计算 (基于相机高度和目标高度) ---
    // 注意：所有物理量使用毫米单位，相机内参以像素为单位
    double fy_half = camera_matrix_.at<double>(1, 1) / 2.0; // 半分辨率下的垂直焦距 (像素)
    double fx_half = camera_matrix_.at<double>(0, 0) / 2.0; // 半分辨率下的水平焦距 (像素)
    double cy_half = camera_matrix_.at<double>(1, 2) / 2.0; // 半分辨率下的垂直光心 (像素)

    // 高度差 (毫米)
    double delta_h = params_.target_height - params_.camera_height;

    // 投影偏移计算：像素偏移 = (像素焦距 × 物理高度差) / 物理距离
    double projected_offset = (fy_half * delta_h) / params_.distance;
    double expected_y = cy_half - projected_offset;

    // 搜索宽度计算：像素宽度 = 像素焦距 × tan(搜索角度)
    double search_w = std::abs(fx_half * std::tan(to_radian(params_.detect_scale)));
    double expected_x = params_.calibrated_pixel_x / 2.0;

    int roi_y = static_cast<int>(expected_y - params_.search_strip_min_h / 2);
    int roi_x = static_cast<int>(expected_x - search_w);

    // 边界对齐
    if (roi_y % 2 != 0)
      roi_y -= 1;
    if (roi_x % 2 != 0)
      roi_x -= 1;

    cv::Rect roi_rect(roi_x, roi_y, static_cast<int>(2 * search_w), params_.search_strip_min_h);
    cv::Rect valid_roi = roi_rect & cv::Rect(0, 0, half_green_.cols, half_green_.rows);

    // 合理范围内的 ROI 条带
    cv::Rect strip_roi = roi_rect & valid_roi;
    if (strip_roi.empty())
      strip_roi = cv::Rect(0, 0, half_green_.cols, half_green_.rows);

    // 在半分辨率图上截取搜索条带
    cv::Mat roi_img = half_green_(strip_roi);

    // --- 步骤 3: 预处理 (二值化 + 膨胀) ---
    if (mask_.size() != roi_img.size())
      mask_.create(roi_img.size(), CV_8UC1);
    cv::threshold(roi_img, mask_, params_.v_low, 255, cv::THRESH_BINARY);
    cv::dilate(mask_, mask_, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // ==========================================
    // 可视化准备：画布与 Chip 容器
    // ==========================================
    cv::Mat debug_vis_full;
    std::vector<cv::Mat> magnified_chips;

    if (debug)
    {
      // 底图使用完整的 half_green 转彩色
      cv::cvtColor(half_green_, debug_vis_full, cv::COLOR_GRAY2BGR);

      // 标示搜索范围
      cv::rectangle(debug_vis_full, strip_roi, cv::Scalar(255, 0, 255), 2);
      cv::circle(
          debug_vis_full, cv::Point(static_cast<int>(expected_x), static_cast<int>(expected_y)), 1,
          cv::Scalar(255, 0, 255), -1);
      cv::putText(
          debug_vis_full, "ROI Search Strip", strip_roi.tl() - cv::Point(0, 5), cv::FONT_HERSHEY_PLAIN,
          1.0, cv::Scalar(255, 0, 255), 1);
    }

    // --- 步骤 4: 筛选循环 ---
    for (const auto &cnt : contours)
    {
      cv::Rect r = cv::boundingRect(cnt); // ROI 坐标系下的外接矩形

      // 计算半分辨率图像上的全局坐标
      cv::Rect global_r = r;
      global_r.x += strip_roi.x;
      global_r.y += strip_roi.y;
      cv::Point global_pt_draw(global_r.x, global_r.y);

      // --- 筛选 1: 面积 ---
      double area = cv::contourArea(cnt);
      if (area < params_.min_area / 4.0 || area > params_.max_area / 4.0)
      {
        if (debug)
        {
          cv::rectangle(debug_vis_full, global_r, cv::Scalar(0, 255, 255), 1); // 黄色 = 面积过滤
          cv::putText(
              debug_vis_full, cv::format("%.1f", area * 4.0), global_pt_draw, cv::FONT_HERSHEY_PLAIN,
              0.8, cv::Scalar(0, 255, 255), 1);
        }
        continue;
      }

      // --- 筛选 2: 长宽比 ---
      double aspect_ratio = static_cast<double>(r.width) / r.height;
      if (aspect_ratio < params_.min_aspect_ratio || aspect_ratio > params_.max_aspect_ratio)
      {
        if (debug)
        {
          cv::rectangle(debug_vis_full, global_r, cv::Scalar(0, 255, 255), 1);
        }
        continue;
      }

      // --- 筛选 3: 圆形度 ---
      double perimeter = cv::arcLength(cnt, true);
      double circularity = (perimeter > 0) ? (4 * CV_PI * area) / (perimeter * perimeter) : 0;
      if (area > 5.0 && circularity < params_.min_circularity)
      {
        if (debug)
        {
          cv::rectangle(debug_vis_full, global_r, cv::Scalar(255, 0, 0), 1); // 蓝色 = 圆形度过滤
        }
        continue;
      }

      // --- 筛选 4: 亮度峰值 ---
      double minVal, maxVal;
      cv::minMaxLoc(roi_img(r), &minVal, &maxVal);
      if (maxVal < params_.v_low + 10)
      {
        if (debug)
        {
          cv::rectangle(debug_vis_full, global_r, cv::Scalar(255, 0, 0), 1); // 蓝色 = 亮度过滤
          cv::putText(
              debug_vis_full, cv::format("%.1f", maxVal), global_pt_draw, cv::FONT_HERSHEY_PLAIN, 0.8,
              cv::Scalar(255, 0, 0), 1);
        }
        continue;
      }

      // --- 筛选 5: 颜色比率检查 (RAW 采样) ---
      double sumR = 0, sumB = 0, sumG = 0;
      int pixel_count = 0;
      {
        // 在小 ROI 内绘制掩码
        cv::Mat local_mask = cv::Mat::zeros(r.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> temp_cnt = {cnt};
        for (auto &pt : temp_cnt[0])
          pt -= r.tl();
        cv::drawContours(local_mask, temp_cnt, 0, cv::Scalar(255), -1);

        for (int iy = 0; iy < r.height; ++iy)
        {
          // 映射回原始 RAW 图像的坐标
          int raw_y_even = (strip_roi.y + r.y + iy) * 2;
          if (raw_y_even + 1 >= raw_image.rows)
            break;

          const uchar *ptr_half_g = roi_img.ptr<uchar>(r.y + iy);
          const uchar *ptr_mask = local_mask.ptr<uchar>(iy);
          const uchar *raw_r_row = raw_image.ptr<uchar>(raw_y_even);
          const uchar *raw_b_row = raw_image.ptr<uchar>(raw_y_even + 1);

          for (int ix = 0; ix < r.width; ++ix)
          {
            if (ptr_mask[ix] > 0)
            {
              sumG += ptr_half_g[r.x + ix]; // 这里的 G 已经是平均后的
              int raw_x_even = (strip_roi.x + r.x + ix) * 2;
              sumR += raw_r_row[raw_x_even];     // BayerRG: R 在偶数行偶数列
              sumB += raw_b_row[raw_x_even + 1]; // BayerRG: B 在奇数行奇数列
              pixel_count++;
            }
          }
        }
      }

      double ratio_gr = 0;
      double ratio_gb = 0;
      if (pixel_count > 0)
      {
        double avgG = sumG / pixel_count;
        double avgR = sumR / pixel_count;
        double avgB = sumB / pixel_count;
        ratio_gr = avgG / (avgR + 1.0);
        ratio_gb = avgG / (avgB + 1.0);

        if (ratio_gr < params_.min_gr_ratio || ratio_gb < params_.min_gb_ratio)
        {
          if (debug)
          {
            cv::rectangle(debug_vis_full, global_r, cv::Scalar(0, 0, 255), 1); // 红色 = 颜色过滤
            cv::putText(
                debug_vis_full, cv::format("%.1f,%.1f", ratio_gr, ratio_gb), global_pt_draw,
                cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(0, 0, 255), 1);
          }
          continue;
        }
      }

      // --- 筛选 6: 通过检测 (PASS) ---
      cv::Point2f local_sub_center = compute_sub_pixel_center(roi_img, r);

      Dot dot;
      // 坐标映射回原图分辨率 (x2)
      dot.center.x = (strip_roi.x + r.x + local_sub_center.x) * 2.0 + 0.5;
      dot.center.y = (strip_roi.y + r.y + local_sub_center.y) * 2.0 + 0.5;

      // 使用内参去畸变，映射到归一化平面坐标
      std::vector<cv::Point2f> srcPoints;
      srcPoints.push_back(dot.center);
      std::vector<cv::Point2f> dstPoints;
      cv::undistortPoints(
          srcPoints, dstPoints, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_);

      dot.center = dstPoints[0];
      dots.push_back(dot);

      // [Debug] 绘制成功结果 & 制作放大图
      if (debug)
      {
        // A. 全图绘制
        cv::rectangle(debug_vis_full, global_r, cv::Scalar(0, 255, 0), 2); // 亮绿色 = 通过
        cv::Point2f global_center = cv::Point2f(global_r.x, global_r.y) + local_sub_center;
        cv::drawMarker(debug_vis_full, global_center, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 15, 2);
        cv::putText(
            debug_vis_full, cv::format("GR=%.1f GB=%.1f", ratio_gr, ratio_gb), global_pt_draw - cv::Point(0, 5),
            cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(0, 255, 0), 1);

        // B. 制作放大图 (Chip) - 统一高度为 64 像素
        cv::Mat chip = roi_img(r).clone();
        cv::cvtColor(chip, chip, cv::COLOR_GRAY2BGR);

        int target_chip_h = 64;
        float scale = static_cast<float>(target_chip_h) / std::max(1, chip.rows);

        // 保持比例缩放
        cv::resize(chip, chip, cv::Size(), scale, scale, cv::INTER_NEAREST);

        // 绘制绿色边框
        cv::rectangle(chip, cv::Rect(0, 0, chip.cols, chip.rows), cv::Scalar(0, 255, 0), 1);
        cv::putText(
            chip, std::to_string(dots.size()), cv::Point(2, 12), cv::FONT_HERSHEY_PLAIN, 0.8,
            cv::Scalar(0, 255, 0), 1);

        magnified_chips.push_back(chip);
      }
    }

    // ==========================================
    // 可视化最终输出：Dashboard & 独立 Mask 窗口
    // ==========================================
    if (debug)
    {
      // ------------------------------------------------------------------
      // 窗口 1: GreenDot Inspector (全图 + 锁定目标拼接图)
      // ------------------------------------------------------------------

      // A. 全图 (含光心标记)
      double cx_half = camera_matrix_.at<double>(0, 2) / 2.0;
      double cy_half_val = camera_matrix_.at<double>(1, 2) / 2.0;
      cv::drawMarker(
          debug_vis_full, cv::Point2f(cx_half, cy_half_val), cv::Scalar(255, 0, 0), cv::MARKER_CROSS,
          15, 1);

      // B. 锁定目标放大图拼接 (Chips)
      cv::Mat vis_chips_strip;
      if (magnified_chips.empty())
      {
        vis_chips_strip = cv::Mat::zeros(64, debug_vis_full.cols, CV_8UC3);
        cv::putText(
            vis_chips_strip, "No Targets Detected", cv::Point(10, 35), cv::FONT_HERSHEY_PLAIN, 1.2,
            cv::Scalar(100, 100, 100), 1);
      }
      else
      {
        // 横向拼接
        cv::hconcat(magnified_chips, vis_chips_strip);

        // 宽度适配 (跟全图一样宽)
        if (vis_chips_strip.cols > debug_vis_full.cols)
        {
          vis_chips_strip = vis_chips_strip(cv::Rect(0, 0, debug_vis_full.cols, vis_chips_strip.rows));
        }
        else if (vis_chips_strip.cols < debug_vis_full.cols)
        {
          cv::Mat black_bg = cv::Mat::zeros(vis_chips_strip.rows, debug_vis_full.cols, CV_8UC3);
          vis_chips_strip.copyTo(black_bg(cv::Rect(0, 0, vis_chips_strip.cols, vis_chips_strip.rows)));
          vis_chips_strip = black_bg;
        }
        cv::putText(
            vis_chips_strip, "Locked Targets (RGB)", cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1.0,
            cv::Scalar(0, 255, 0), 1);
      }

      // C. 垂直拼接 (全图 + Chips)
      std::vector<cv::Mat> v_imgs;
      v_imgs.push_back(debug_vis_full);
      v_imgs.push_back(vis_chips_strip);

      cv::Mat dashboard;
      cv::vconcat(v_imgs, dashboard);

      // 缩放主窗口以适应屏幕
      if (dashboard.rows > 950)
      {
        cv::resize(dashboard, dashboard, cv::Size(), 0.7, 0.7);
      }
      cv::namedWindow("GreenDot Inspector", cv::WINDOW_NORMAL | cv::WINDOW_GUI_EXPANDED);
      cv::imshow("GreenDot Inspector", dashboard);

      // ------------------------------------------------------------------
      // 窗口 2: ROI Binary Mask (独立窗口，放大显示)
      // ------------------------------------------------------------------
      if (!mask_.empty())
      {
        cv::Mat mask_display;

        // 放大 3 倍显示，使用 INTER_NEAREST 保持二值图边缘锐利
        double scale_factor = 3.0;
        cv::resize(mask_, mask_display, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);

        // 图上标注
        cv::putText(
            mask_display, "ROI Binary Mask (x3)", cv::Point(5, 25), cv::FONT_HERSHEY_PLAIN, 1.5,
            cv::Scalar(128), 2);
        cv::namedWindow("ROI Mask (Magnified)", cv::WINDOW_NORMAL | cv::WINDOW_GUI_EXPANDED);
        cv::imshow("ROI Mask (Magnified)", mask_display);
      }

      cv::waitKey(1);
    }

    return !dots.empty();
  }

  // ==========================================
  // 4. 偏航角计算
  // ==========================================

  bool GreenDotDetect::calculate_dots_yaw(std::vector<Dot> &dots)
  {
    if (dots.empty())
      return false;

    for (size_t i = 0; i < dots.size(); ++i)
    {
      // 偏航角计算：yaw = arctan(像素差 / 焦距) × (180/π)
      // dots[i].center.x: 去畸变后的归一化平面坐标
      // params_.calibrated_pixel_x: 校准的像素点 x 坐标
      // camera_matrix_.at<double>(0, 0): 水平焦距 (像素)
      dots[i].yaw =
          std::atan(
              (dots[i].center.x - params_.calibrated_pixel_x) / camera_matrix_.at<double>(0, 0)) *
          (180.0 / CV_PI);
    }
    return true;
  }

} // namespace detect_base
