#pragma once

#include <onnxruntime_cxx_api.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "yolo_model_interface.hpp"

class YoloOnnxDetector : public YoloOnnxProcessor<YoloOnnxDetector> {
  friend class YoloOnnxProcessor<YoloOnnxDetector>;

public:
  struct Detection {
    int classId = -1;
    std::string className;
    float score = 0.0f;
    cv::Rect box;

    cv::Point center() const {
      return cv::Point(box.x + box.width / 2, box.y + box.height / 2);
    }
  };

  YoloOnnxDetector(const std::filesystem::path &modelPath,
                   const std::vector<std::string> &classNames)
      : YoloOnnxProcessor<YoloOnnxDetector>(modelPath),
        classNames_(classNames) {}

  // 按类别过滤
  static std::vector<Detection>
  getByClass(const std::vector<Detection> &dets, int classId) {
    std::vector<Detection> result;
    for (const auto &d : dets)
      if (d.classId == classId)
        result.push_back(d);
    return result;
  }

  // 找距画面中心最近的检测框
  static const Detection *
  nearestToCenter(const std::vector<Detection> &dets,
                  const cv::Size &frameSize) {
    if (dets.empty())
      return nullptr;
    const cv::Point imgCenter(frameSize.width / 2, frameSize.height / 2);
    const Detection *best = &dets[0];
    double bestDist = cv::norm(best->center() - imgCenter);
    for (size_t i = 1; i < dets.size(); ++i) {
      double dist = cv::norm(dets[i].center() - imgCenter);
      if (dist < bestDist) {
        bestDist = dist;
        best = &dets[i];
      }
    }
    return best;
  }

  // 绘制检测结果
  static void drawDetections(cv::Mat &image,
                             const std::vector<Detection> &detections) {
    for (const auto &det : detections) {
      cv::rectangle(image, det.box, cv::Scalar(0, 255, 0), 2);
      std::string label = det.className + ": " + cv::format("%.2f", det.score);
      cv::putText(image, label,
                  cv::Point(det.box.x, std::max(0, det.box.y - 8)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
  }

private:
  std::vector<Detection> processImpl(const cv::Mat &frame, float confThres) {
    float ratio, dw, dh;
    auto inputTensorValues = preprocess(frame, ratio, dw, dh);
    auto outputs = runInference(inputTensorValues);

    if (outputs.empty() || !outputs[0].IsTensor())
      return {};
    auto outShape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    // [1, N, 6]
    if (outShape.size() != 3 || outShape[0] != 1 || outShape[2] < 6)
      return {};

    const float *outData = outputs[0].GetTensorData<float>();
    const size_t numDet = static_cast<size_t>(outShape[1]);
    const size_t detLen = static_cast<size_t>(outShape[2]);

    std::vector<Detection> dets;
    for (size_t i = 0; i < numDet; ++i) {
      float score = outData[i * detLen + 4];
      if (score < confThres)
        continue;

      int cls = static_cast<int>(std::round(outData[i * detLen + 5]));
      cls =
          std::max(0, std::min(cls, static_cast<int>(classNames_.size()) - 1));

      float x1 = (outData[i * detLen + 0] - dw) / ratio;
      float y1 = (outData[i * detLen + 1] - dh) / ratio;
      float x2 = (outData[i * detLen + 2] - dw) / ratio;
      float y2 = (outData[i * detLen + 3] - dh) / ratio;

      int ix1 = std::max(
          0, std::min(static_cast<int>(std::round(x1)), frame.cols - 1));
      int iy1 = std::max(
          0, std::min(static_cast<int>(std::round(y1)), frame.rows - 1));
      int ix2 = std::max(
          0, std::min(static_cast<int>(std::round(x2)), frame.cols - 1));
      int iy2 = std::max(
          0, std::min(static_cast<int>(std::round(y2)), frame.rows - 1));

      if (ix2 <= ix1 || iy2 <= iy1)
        continue;
      dets.push_back({cls, classNames_[cls], score,
                      cv::Rect(cv::Point(ix1, iy1), cv::Point(ix2, iy2))});
    }
    return dets;
  }

  std::vector<std::string> classNames_;
};
