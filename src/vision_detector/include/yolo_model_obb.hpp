#pragma once

#include "yolo_model_interface.hpp"
#include <string>
#include <vector>

class YoloOnnxObb : public YoloOnnxProcessor<YoloOnnxObb> {
  friend class YoloOnnxProcessor<YoloOnnxObb>;

public:
  struct ObbDetection {
    int classId = -1;
    std::string className;
    float score = 0.0f;
    // 旋转矩形参数（原图坐标系）
    cv::Point2f center;
    cv::Size2f size; // width, height
    float angle;     // 角度（度），通常模型输出为弧度需转换为度
    // 也可以直接存储四个角点
    std::vector<cv::Point2f> corners; // 4个点，可由 RotatedRect 生成
  };

  YoloOnnxObb(const std::filesystem::path &modelPath,
              const std::vector<std::string> &classNames)
      : YoloOnnxProcessor<YoloOnnxObb>(modelPath), classNames_(classNames) {}

  // 绘制旋转框
  static void drawObb(cv::Mat &image,
                      const std::vector<ObbDetection> &detections,
                      const cv::Scalar &color = cv::Scalar(0, 255, 0)) {
    for (const auto &det : detections) {
      std::vector<cv::Point> pts;
      for (const auto &p : det.corners) {
        pts.push_back(cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)));
      }
      cv::polylines(image, pts, true, color, 2);
      // 标签
      std::string label = det.className + ": " + cv::format("%.2f", det.score);
      cv::putText(image, label, pts[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, color,
                  2);
    }
  }

private:
  std::vector<ObbDetection> processImpl(const cv::Mat &frame,
                                         float confThres) {
    float ratio, dw, dh;
    auto inputTensorValues = preprocess(frame, ratio, dw, dh);
    auto outputs = runInference(inputTensorValues);

    if (outputs.empty() || !outputs[0].IsTensor())
      return {};
    auto outShape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    // 期望 [1, N, 7]：cx, cy, w, h, angle, conf, class_id
    if (outShape.size() != 3 || outShape[0] != 1 || outShape[2] < 7)
      return {};

    const float *outData = outputs[0].GetTensorData<float>();
    const size_t numDet = static_cast<size_t>(outShape[1]);
    const size_t detLen = static_cast<size_t>(outShape[2]);

    std::vector<ObbDetection> results;
    results.reserve(numDet);

    for (size_t i = 0; i < numDet; ++i) {
      float score = outData[i * detLen + 5]; // 第5列（0开始）
      if (score < confThres)
        continue;

      int cls = static_cast<int>(std::round(outData[i * detLen + 6]));
      cls =
          std::max(0, std::min(cls, static_cast<int>(classNames_.size()) - 1));

      // 从 letterbox 空间逆映射
      float cx = (outData[i * detLen + 0] - dw) / ratio;
      float cy = (outData[i * detLen + 1] - dh) / ratio;
      float w = outData[i * detLen + 2] / ratio;
      float h = outData[i * detLen + 3] / ratio;
      float angle = outData[i * detLen + 4]; // 可能是弧度，需根据模型转换

      // 如果模型输出弧度，转换为度（YOLOv8-obb 导出为弧度，OpenCV 的
      // RotatedRect 用度） 假设模型输出弧度，转换为度
      const float angle_deg = angle * 180.0f / CV_PI;

      // 构建旋转矩形（OpenCV 的 angle 定义：顺时针，x轴逆时针旋转的角度）
      cv::RotatedRect rrect(cv::Point2f(cx, cy), cv::Size2f(w, h), angle_deg);

      // 获取四个角点
      std::vector<cv::Point2f> corners(4);
      cv::boxPoints(rrect, corners);

      ObbDetection det;
      det.classId = cls;
      det.className = classNames_[cls];
      det.score = score;
      det.center = cv::Point2f(cx, cy);
      det.size = cv::Size2f(w, h);
      det.angle = angle_deg;
      det.corners = corners;
      results.push_back(det);
    }
    return results;
  }

  std::vector<std::string> classNames_;
};
