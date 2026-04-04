#include "detect_base/GreenDotDetect.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

#define TO_RADIAN(x) ((x)*CV_PI / 180.0)

// ==========================================
// 1. 初始化与参数更新
// ==========================================
void GreenDotDetect::init_camera(const cv::Mat & cameraMatrix, const cv::Mat & distCoffs)
{
  this->cameraMatrix = cameraMatrix.clone();
  this->distCoeffs = distCoffs.clone();
}

void GreenDotDetect::update_params(const DetectParams & new_params)
{
  this->params = new_params;
  // std::cout << "[GreenDot] Params Updated." << std::endl;
}

// ==========================================
// 2. 亚像素质心计算
// ==========================================
cv::Point2f GreenDotDetect::computeSubPixelCenter(
  const cv::Mat & roi_img, const cv::Rect & local_rect)
{
  // 确保 ROI 不越界
  cv::Rect safe_rect = local_rect & cv::Rect(0, 0, roi_img.cols, roi_img.rows);

  // 如果交集为空，返回几何中心
  if (safe_rect.empty()) {
    return cv::Point2f(
      local_rect.width / 2.0f, local_rect.height / 2.0f);
  }

  cv::Mat roi = roi_img(safe_rect);
  cv::Moments mu = cv::moments(roi, false);

  // 防止除零（全黑区域）
  if (mu.m00 < DBL_EPSILON) {
    return cv::Point2f(safe_rect.x + safe_rect.width / 2.0f, safe_rect.y + safe_rect.height / 2.0f);
  }

  return cv::Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
}

// ==========================================
// 3. [核心] Raw 数据检测
// ==========================================
bool GreenDotDetect::detect(const cv::Mat & raw_image, std::vector<Dot> & dots, bool debug)
{
  // 1. 基础检查
  if (raw_image.empty() || raw_image.channels() != 1) return false;
  dots.clear();

  // --- 步骤 1: 降维 (Bayer -> Green Energy) ---
  int w = raw_image.cols;
  int h = raw_image.rows;

  // 安全性：确保高度为偶数，防止 ptr(y+1) 越界
  if (h % 2 != 0) h -= 1;

  // 底图分辨率是 Raw 的一半
  cv::Mat half_green = cv::Mat::zeros(h / 2, w / 2, CV_8UC1);

  // 快速提取绿色通道能量 (BayerRG 格式下，偶数行是 R G，奇数行是 G B)
  // 这里取相邻两行的 G 分量平均值作为该点的亮度
  for (int y = 0; y < h; y += 2) {
    const uchar * r0 = raw_image.ptr<uchar>(y);
    const uchar * r1 = raw_image.ptr<uchar>(y + 1);
    uchar * d = half_green.ptr<uchar>(y / 2);
    for (int x = 0; x < w; x += 2) {
      // RGGB 布局: r0[x+1] 是 G, r1[x] 是 G
      d[x / 2] = (uchar)(((int)r0[x + 1] + (int)r1[x]) >> 1);
    }
  }

  // --- 步骤 2: 物理 ROI 计算 (基于相机高度和目标高度) ---
  double fy_half = cameraMatrix.at<double>(1, 1) / 2.0;
  double fx_half = cameraMatrix.at<double>(0, 0) / 2.0;
  double cy_half = cameraMatrix.at<double>(1, 2) / 2.0;
  double cx_half = cameraMatrix.at<double>(0, 2) / 2.0;

  double delta_h = params.target_height - params.camera_height;
  double projected_offset = (fy_half * delta_h) / params.distance;
  double expected_y = cy_half - projected_offset;

  double search_w = std::abs(fx_half * std::tan(TO_RADIAN(params.detect_scale)));
  double expected_x = cx_half;

  int roi_y = static_cast<int>(expected_y - params.search_strip_min_h / 2);
  int roi_x = static_cast<int>(expected_x - search_w);

  // 边界检查
  if (roi_y % 2 != 0) roi_y -= 1;
  if (roi_x % 2 != 0) roi_x -= 1;

  cv::Rect roi_rect(roi_x, roi_y, static_cast<int>(2 * search_w), params.search_strip_min_h);
  cv::Rect valid_roi = roi_rect & cv::Rect(0, 0, half_green.cols, half_green.rows);

  cv::Rect strip_roi = roi_rect & valid_roi;//合理范围内的roi条带
  if (strip_roi.empty()) strip_roi = cv::Rect(0, 0, half_green.cols, half_green.rows);

  // 截取搜索条带
  cv::Mat roi_img = half_green(strip_roi);//在半分辨率图上地搜索条带

  // --- 步骤 3: 预处理 (二值化 + 膨胀) ---
  cv::Mat mask;
  cv::threshold(roi_img, mask, params.v_low, 255, cv::THRESH_BINARY);
  cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // ==========================================
  // Visual Setup: 准备全图画布和 Chip 容器
  // ==========================================
  cv::Mat debug_vis_full;
  std::vector<cv::Mat> magnified_chips;

  if (debug) {
    // 1. 底图使用完整的 half_green 转彩色
    cv::cvtColor(half_green, debug_vis_full, cv::COLOR_GRAY2BGR);

    // 2. 标示搜索范围
    cv::rectangle(debug_vis_full, strip_roi, cv::Scalar(255, 0, 255), 2);
    cv::circle(
      debug_vis_full, cv::Point(static_cast<int>(expected_x), static_cast<int>(expected_y)), 1,
      cv::Scalar(255, 0, 255), -1);
    cv::putText(
      debug_vis_full, "ROI Search Strip", strip_roi.tl() - cv::Point(0, 5), cv::FONT_HERSHEY_PLAIN,
      1.0, cv::Scalar(255, 0, 255), 1);
  }

  // --- 步骤 4: 筛选循环 ---
  for (const auto & cnt : contours) {
    cv::Rect r = cv::boundingRect(cnt);//roi坐标系下的外接矩

    // 计算全图坐标用于绘制
    cv::Rect global_r = r;//在半分辨率图像上的坐标
    global_r.x += strip_roi.x;
    global_r.y += strip_roi.y;
    cv::Point global_pt_draw(global_r.x, global_r.y);

    // --- 1. 面积筛选 ---
    double area = cv::contourArea(cnt);
    if (area < params.min_area / 4.0 || area > params.max_area / 4.0) {
      if (debug) {
        cv::rectangle(debug_vis_full, global_r, cv::Scalar(0, 255, 255), 1);  // 黄色代表被过滤
        cv::putText(
          debug_vis_full, cv::format("%.1f", area * 4.0), global_pt_draw, cv::FONT_HERSHEY_PLAIN,
          0.8, cv::Scalar(0, 255, 255), 1);
      }
      continue;
    }

    // --- 2. 长宽比筛选 ---
    double aspect_ratio = (double)r.width / r.height;
    if (aspect_ratio < params.min_aspect_ratio || aspect_ratio > params.max_aspect_ratio) {
      if (debug) {
        cv::rectangle(debug_vis_full, global_r, cv::Scalar(0, 255, 255), 1);
      }
      continue;
    }

    // --- 3. 圆形度筛选 ---
    double perimeter = cv::arcLength(cnt, true);
    double circularity = (perimeter > 0) ? (4 * CV_PI * area) / (perimeter * perimeter) : 0;
    if (area > 5.0 && circularity < params.min_circularity) {
      if (debug) {
        cv::rectangle(debug_vis_full, global_r, cv::Scalar(255, 0, 0), 1);  // 蓝色
      }
      continue;
    }

    // --- 4. 亮度峰值筛选 ---
    double minVal, maxVal;
    cv::minMaxLoc(roi_img(r), &minVal, &maxVal);
    if (maxVal < params.v_low + 10) {
      if (debug) {
        cv::rectangle(debug_vis_full, global_r, cv::Scalar(255, 0, 0), 1);  // 蓝色
        cv::putText(
          debug_vis_full, cv::format("%.1f", maxVal), global_pt_draw, cv::FONT_HERSHEY_PLAIN, 0.8,
          cv::Scalar(255, 0, 0), 1);
      }
      continue;
    }

    // --- 5. 颜色比率检查 (RAW 采样) ---
    double sumR = 0, sumB = 0, sumG = 0;
    int pixel_count = 0;
    {
      // 在小 ROI 内绘制掩码
      cv::Mat local_mask = cv::Mat::zeros(r.size(), CV_8UC1);
      std::vector<std::vector<cv::Point>> temp_cnt = {cnt};
      for (auto & pt : temp_cnt[0]) pt -= r.tl();
      cv::drawContours(local_mask, temp_cnt, 0, cv::Scalar(255), -1);

      for (int iy = 0; iy < r.height; ++iy) {
        // 映射回原始 RAW 图像的坐标
        int raw_y_even = (strip_roi.y + r.y + iy) * 2;
        if (raw_y_even + 1 >= raw_image.rows) break;

        const uchar * ptr_half_g = roi_img.ptr<uchar>(r.y + iy);
        const uchar * ptr_mask = local_mask.ptr<uchar>(iy);
        const uchar * raw_r_row = raw_image.ptr<uchar>(raw_y_even);
        const uchar * raw_b_row = raw_image.ptr<uchar>(raw_y_even + 1);

        for (int ix = 0; ix < r.width; ++ix) {
          if (ptr_mask[ix] > 0) {
            sumG += ptr_half_g[r.x + ix];  // 这里的G已经是平均后的
            int raw_x_even = (strip_roi.x + r.x + ix) * 2;
            sumR += raw_r_row[raw_x_even];      // BayerRG: R 在偶数行偶数列
            sumB += raw_b_row[raw_x_even + 1];  // BayerRG: B 在奇数行奇数列
            pixel_count++;
          }
        }
      }
    }

    double ratio_gr = 0;
    if (pixel_count > 0) {
      double avgG = sumG / pixel_count;
      double avgR = sumR / pixel_count;
      double avgB = sumB / pixel_count;
      ratio_gr = avgG / (avgR + 1.0);
      double ratio_gb = avgG / (avgB + 1.0);

      if (ratio_gr < params.min_gr_ratio || ratio_gb < params.min_gb_ratio) {
        if (debug) {
          cv::rectangle(debug_vis_full, global_r, cv::Scalar(0, 0, 255), 1);  // 红色
          cv::putText(
            debug_vis_full, cv::format("%.1f,%.1f", ratio_gr, ratio_gb), global_pt_draw,
            cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(0, 0, 255), 1);
        }
        continue;
      }
    }

    // --- 6. 通过检测 (PASS) ---
    cv::Point2f local_sub_center = computeSubPixelCenter(roi_img, r);

    Dot dot;
    // 坐标映射回原图分辨率 (x2)
    dot.center.x = (strip_roi.x + r.x + local_sub_center.x) * 2.0 + 0.5;
    dot.center.y = (strip_roi.y + r.y + local_sub_center.y) * 2.0 + 0.5;

    std::vector<cv::Point2f> srcPoints;
    srcPoints.push_back(dot.center);
    std::vector<cv::Point2f> dstPoints;
    // 使用内参去畸变点，计算物理平面坐标
    cv::undistortPoints(
      srcPoints, dstPoints, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

    dot.center = dstPoints[0];
    dot.roi = cv::Rect((strip_roi.x + r.x) * 2, (strip_roi.y + r.y) * 2, r.width * 2, r.height * 2);
    dot.area = area * 4;
    dots.push_back(dot);

    // [Debug] 绘制成功结果 & 制作放大图
    if (debug) {
      // A. 全图绘制
      cv::rectangle(debug_vis_full, global_r, cv::Scalar(0, 255, 0), 2);  // 亮绿色
      cv::Point2f global_center =
        cv::Point2f(global_r.x, global_r.y) + local_sub_center;
      cv::drawMarker(debug_vis_full, global_center, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 15, 2);
      cv::putText(
        debug_vis_full, cv::format("%.1f", ratio_gr), global_pt_draw - cv::Point(0, 5),
        cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(0, 255, 0), 1);

      // B. 制作放大图 (Chip) - 关键修复：统一高度
      cv::Mat chip = roi_img(r).clone();
      cv::cvtColor(chip, chip, cv::COLOR_GRAY2BGR);

      // 设定一个固定的显示高度 (64像素)
      int target_chip_h = 64;
      float scale = (float)target_chip_h / std::max(1, chip.rows);  // 避免除0

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
  // Visual Final: Dashboard & Separate Mask
  // ==========================================
  if (debug) {
    // ------------------------------------------------------------------
    // 窗口 1: GreenDot Inspector (全图 + 锁定目标拼接图)
    // ------------------------------------------------------------------

    // A. Top: 全图 (含光心)
    double cx_half = cameraMatrix.at<double>(0, 2) / 2.0;
    double cy_half_val = cameraMatrix.at<double>(1, 2) / 2.0;
    cv::drawMarker(
      debug_vis_full, cv::Point2f(cx_half, cy_half_val), cv::Scalar(255, 0, 0), cv::MARKER_CROSS,
      15, 1);

    // B. Bottom: 锁定目标放大图拼接 (Chips)
    cv::Mat vis_chips_strip;
    if (magnified_chips.empty()) {
      vis_chips_strip = cv::Mat::zeros(64, debug_vis_full.cols, CV_8UC3);
      cv::putText(
        vis_chips_strip, "No Targets Detected", cv::Point(10, 35), cv::FONT_HERSHEY_PLAIN, 1.2,
        cv::Scalar(100, 100, 100), 1);
    } else {
      // 横向拼接
      cv::hconcat(magnified_chips, vis_chips_strip);

      // 宽度适配 (跟全图一样宽)
      if (vis_chips_strip.cols > debug_vis_full.cols) {
        vis_chips_strip =
          vis_chips_strip(cv::Rect(0, 0, debug_vis_full.cols, vis_chips_strip.rows));
      } else if (vis_chips_strip.cols < debug_vis_full.cols) {
        cv::Mat black_bg = cv::Mat::zeros(vis_chips_strip.rows, debug_vis_full.cols, CV_8UC3);
        vis_chips_strip.copyTo(
          black_bg(cv::Rect(0, 0, vis_chips_strip.cols, vis_chips_strip.rows)));
        vis_chips_strip = black_bg;
      }
      cv::putText(
        vis_chips_strip, "Locked Targets (RGB)", cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1.0,
        cv::Scalar(0, 255, 0), 1);
    }

    // C. 垂直拼接 (只拼 全图 和 Chips，去掉中间的 Mask)
    std::vector<cv::Mat> v_imgs;
    v_imgs.push_back(debug_vis_full);
    v_imgs.push_back(vis_chips_strip);

    cv::Mat dashboard;
    cv::vconcat(v_imgs, dashboard);

    // 缩放主窗口以适应屏幕
    if (dashboard.rows > 950) {
      cv::resize(dashboard, dashboard, cv::Size(), 0.7, 0.7);
    }
    cv::namedWindow("GreenDot Inspector", cv::WINDOW_NORMAL | cv::WINDOW_GUI_EXPANDED);
    cv::imshow("GreenDot Inspector", dashboard);

    // ------------------------------------------------------------------
    // 窗口 2: ROI Binary Mask (独立窗口，放大显示)
    // ------------------------------------------------------------------
    if (!mask.empty()) {
      cv::Mat mask_display;

      // 放大倍数 (例如放大 3 倍，看着更清楚)
      double scale_factor = 3.0;

      // 使用 INTER_NEAREST 保持二值图的边缘锐利，不模糊
      cv::resize(mask, mask_display, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);

      // 在图上写字提示
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

bool GreenDotDetect::calculate_dots_yaw(std::vector<Dot> & dots)
{
  if (dots.empty()) return false;
  for (size_t i = 0; i < dots.size(); ++i) {
    // atan(x) 得到的是归一化平面上的角度 (Assuming z=1)
    dots[i].yaw =
      std::atan(
        (dots[i].center.x - cameraMatrix.at<double>(0, 2)) / cameraMatrix.at<double>(0, 0)) *
        (180.0 / CV_PI) +
      params.yaw_offset;
  }
  return true;
}