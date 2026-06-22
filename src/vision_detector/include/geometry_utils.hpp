#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

// 对四个点排序：[左上, 右上, 右下, 左下]
inline std::vector<cv::Point2f>
orderPoints(const std::vector<cv::Point2f> &pts) {
  std::vector<cv::Point2f> rect(4);
  std::vector<float> sum(4), diff(4);
  for (int i = 0; i < 4; ++i) {
    sum[i] = pts[i].x + pts[i].y;
    diff[i] = pts[i].y - pts[i].x;
  }
  auto min_sum = std::min_element(sum.begin(), sum.end());
  auto max_sum = std::max_element(sum.begin(), sum.end());
  rect[0] = pts[std::distance(sum.begin(), min_sum)]; // 左上
  rect[2] = pts[std::distance(sum.begin(), max_sum)]; // 右下

  auto min_diff = std::min_element(diff.begin(), diff.end());
  auto max_diff = std::max_element(diff.begin(), diff.end());
  rect[1] = pts[std::distance(diff.begin(), min_diff)]; // 右上
  rect[3] = pts[std::distance(diff.begin(), max_diff)]; // 左下
  return rect;
}

// 将输入图像中由四边形包围的区域透视变换为正方形
// poly: 四个点（任意顺序，内部会调用 orderPoints 排序）
// output_size: 输出正方形边长
inline cv::Mat warpPolygonToSquare(const cv::Mat &img,
                                   const std::vector<cv::Point2f> &poly,
                                   int output_size = 512) {
  auto src = orderPoints(poly);
  std::vector<cv::Point2f> dst = {{0.0f, 0.0f},
                                  {static_cast<float>(output_size - 1), 0.0f},
                                  {static_cast<float>(output_size - 1),
                                   static_cast<float>(output_size - 1)},
                                  {0.0f, static_cast<float>(output_size - 1)}};
  cv::Mat M = cv::getPerspectiveTransform(src, dst);
  cv::Mat warped;
  cv::warpPerspective(img, warped, M, cv::Size(output_size, output_size));
  return warped;
}

// 将轮廓近似为四边形，失败则回退到最小外接矩形
inline std::vector<cv::Point2f>
approxToQuad(const std::vector<cv::Point> &contour) {
  double peri = cv::arcLength(contour, true);
  std::vector<cv::Point> approx;
  // 尝试多个 epsilon 因子
  for (double factor : {0.02, 0.04, 0.08, 0.12}) {
    cv::approxPolyDP(contour, approx, factor * peri, true);
    if (approx.size() == 4) {
      std::vector<cv::Point2f> quad;
      for (auto &p : approx)
        quad.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
      return quad;
    }
  }
  // 回退：用 minAreaRect 取四个角点
  cv::RotatedRect rect = cv::minAreaRect(contour);
  std::vector<cv::Point2f> box(4);
  cv::boxPoints(rect, box);
  return box;
}
