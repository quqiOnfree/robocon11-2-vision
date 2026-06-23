#pragma once

#include "yolo_model_interface.hpp"

class YoloOnnxPose : public YoloOnnxProcessor<YoloOnnxPose> {
  friend class YoloOnnxProcessor<YoloOnnxPose>;

public:
  struct PoseResult {
    cv::Rect box;
    float score;
    std::vector<cv::Point2f> keypoints; // 原图坐标
    std::vector<float> visibility;
  };

  YoloOnnxPose(const std::filesystem::path &modelPath, int numKeypoints)
      : YoloOnnxProcessor<YoloOnnxPose>(modelPath), numKpts_(numKeypoints) {}

  static void drawPose(cv::Mat &image, const std::vector<PoseResult> &poses) {
    for (const auto &pose : poses) {
      cv::rectangle(image, pose.box, cv::Scalar(0, 255, 0), 2);
      for (const auto &kp : pose.keypoints) {
        cv::circle(image, kp, 3, cv::Scalar(0, 0, 255), -1);
      }
    }
  }

private:
  std::vector<PoseResult> processImpl(const cv::Mat &frame, float confThres) {
    float ratio, dw, dh;
    auto inputTensor = preprocess(frame, ratio, dw, dh);
    auto outputs = runInference(inputTensor);
    if (outputs.empty() || !outputs[0].IsTensor())
      return {};

    auto outShape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    // [1, N, 5 + K*3]
    if (outShape.size() != 3 || outShape[0] != 1)
      return {};
    size_t numDet = static_cast<size_t>(outShape[1]);
    size_t detLen = static_cast<size_t>(outShape[2]);

    const float *outData = outputs[0].GetTensorData<float>();
    std::vector<PoseResult> results;
    for (size_t i = 0; i < numDet; ++i) {
      float score = outData[i * detLen + 4];
      if (score < confThres)
        continue;

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

      PoseResult pr;
      pr.box = cv::Rect(cv::Point(ix1, iy1), cv::Point(ix2, iy2));
      pr.score = score;

      // 关键点逆映射
      for (int k = 0; k < numKpts_; ++k) {
        float kx = (outData[i * detLen + 5 + k * 3 + 0] - dw) / ratio;
        float ky = (outData[i * detLen + 5 + k * 3 + 1] - dh) / ratio;
        float vis = outData[i * detLen + 5 + k * 3 + 2];
        kx = std::max(0.0f, std::min(kx, static_cast<float>(frame.cols - 1)));
        ky = std::max(0.0f, std::min(ky, static_cast<float>(frame.rows - 1)));
        pr.keypoints.emplace_back(kx, ky);
        pr.visibility.push_back(vis);
      }
      results.push_back(pr);
    }
    return results;
  }

  int numKpts_;
};
