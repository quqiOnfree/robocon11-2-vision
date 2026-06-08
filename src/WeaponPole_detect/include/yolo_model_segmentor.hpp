#pragma once

#include "yolo_model_interface.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

class YoloOnnxSegmentor : public YoloOnnxProcessor<YoloOnnxSegmentor> {
  friend class YoloOnnxProcessor<YoloOnnxSegmentor>;

public:
  struct SegmentedFace {
    cv::Mat binary_mask; // 全图尺寸的二值 mask
    cv::Rect bbox;       // 原图上的检测框
    int class_id = -1;
    std::string class_name;
    float score = 0.0f;
  };

  YoloOnnxSegmentor(const std::filesystem::path &modelPath,
                    const std::vector<std::string> &classNames = {})
      : YoloOnnxProcessor<YoloOnnxSegmentor>(modelPath),
        classNames_(classNames) {}

  static void drawInstances(cv::Mat &image,
                            const std::vector<SegmentedFace> &faces) {
    for (const auto &face : faces) {
      // 从二值掩码提取轮廓
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(face.binary_mask, contours, cv::RETR_EXTERNAL,
                       cv::CHAIN_APPROX_SIMPLE);
      if (contours.empty())
        continue;

      // 取最大轮廓并绘制（也可以画所有轮廓，但通常最大轮廓就是该面）
      auto biggest = std::max_element(
          contours.begin(), contours.end(), [](const auto &a, const auto &b) {
            return cv::contourArea(a) < cv::contourArea(b);
          });

      // 绘制多边形轮廓
      cv::polylines(image, *biggest, true, cv::Scalar(0, 255, 0), 2);

      // 绘制类别标签
      std::string label = face.class_name.empty()
                              ? "class " + std::to_string(face.class_id)
                              : face.class_name;
      label += ": " + cv::format("%.2f", face.score);
      // 标签位置选在 bbox 左上角
      cv::putText(image, label, face.bbox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 2);
    }
  }

private:
  std::vector<SegmentedFace> processImpl(const cv::Mat &frame,
                                         float confThres) {
    float ratio, dw, dh;
    auto inputTensor = preprocess(frame, ratio, dw, dh);
    // 多输出推理（需要两个输出：output0 和 output1）
    auto outputs = runInferenceAll(inputTensor);

    if (outputs.size() < 2 || !outputs[0].IsTensor() || !outputs[1].IsTensor())
      return {};

    auto shape0 = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    if (shape0.size() != 3 || shape0[0] != 1 || shape0[2] < 38)
      return {};

    auto shape1 = outputs[1].GetTensorTypeAndShapeInfo().GetShape();
    if (shape1.size() != 4 || shape1[0] != 1 || shape1[1] < 1)
      return {};

    const float *out0 = outputs[0].GetTensorData<float>();
    const float *out1 = outputs[1].GetTensorData<float>();

    const size_t numDet = static_cast<size_t>(shape0[1]);
    const size_t detLen = static_cast<size_t>(shape0[2]);
    const int numProto = static_cast<int>(shape1[1]);
    const int protoH = static_cast<int>(shape1[2]);
    const int protoW = static_cast<int>(shape1[3]);

    std::vector<SegmentedFace> results;

    for (size_t i = 0; i < numDet; ++i) {
      try {
        const float *det = out0 + i * detLen;
        float conf = det[4];
        if (conf < confThres)
          continue;

        int cls = static_cast<int>(std::round(det[5]));
        if (!classNames_.empty()) {
          cls = std::max(
              0, std::min(cls, static_cast<int>(classNames_.size()) - 1));
        }

        // letterbox → 原图 bbox 变换
        float ox1 = (det[0] - dw) / ratio;
        float oy1 = (det[1] - dh) / ratio;
        float ox2 = (det[2] - dw) / ratio;
        float oy2 = (det[3] - dh) / ratio;

        int ix1 = std::max(
            0, std::min(static_cast<int>(std::round(ox1)), frame.cols - 1));
        int iy1 = std::max(
            0, std::min(static_cast<int>(std::round(oy1)), frame.rows - 1));
        int ix2 = std::max(
            0, std::min(static_cast<int>(std::round(ox2)), frame.cols - 1));
        int iy2 = std::max(
            0, std::min(static_cast<int>(std::round(oy2)), frame.rows - 1));

        if (ix2 <= ix1 || iy2 <= iy1)
          continue;

        // 生成实例 mask
        cv::Mat mask160(protoH, protoW, CV_32FC1, cv::Scalar(0.0f));
        float *mData = reinterpret_cast<float *>(mask160.data);

        const float *coeffs = det + 6; // mask 系数偏移
        for (int c = 0; c < numProto; ++c) {
          float coeff = coeffs[c];
          if (coeff == 0.0f)
            continue;
          const float *pData = out1 + c * protoH * protoW;
          for (int p = 0; p < protoH * protoW; ++p) {
            mData[p] += coeff * pData[p];
          }
        }
        // sigmoid
        for (int p = 0; p < protoH * protoW; ++p) {
          mData[p] = 1.0f / (1.0f + std::exp(-mData[p]));
        }

        // letterbox bbox 映射到 160x160 空间
        float s = static_cast<float>(protoW) / static_cast<float>(inputW_);
        int mx1 =
            std::max(0, std::min(static_cast<int>(det[0] * s), protoW - 1));
        int my1 =
            std::max(0, std::min(static_cast<int>(det[1] * s), protoH - 1));
        int mx2 =
            std::max(0, std::min(static_cast<int>(det[2] * s), protoW - 1));
        int my2 =
            std::max(0, std::min(static_cast<int>(det[3] * s), protoH - 1));

        if (mx2 <= mx1 || my2 <= my1)
          continue;

        cv::Rect cropR(mx1, my1, mx2 - mx1, my2 - my1);
        cv::Mat croppedMask = mask160(cropR).clone();
        int bboxW = ix2 - ix1, bboxH = iy2 - iy1;
        cv::Mat resizedMask;
        cv::resize(croppedMask, resizedMask, cv::Size(bboxW, bboxH), 0, 0,
                   cv::INTER_LINEAR);

        cv::Mat binMask;
        cv::threshold(resizedMask, binMask, 0.5, 1.0, cv::THRESH_BINARY);
        binMask.convertTo(binMask, CV_8UC1, 255);

        cv::Mat fullMask = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);
        binMask.copyTo(fullMask(cv::Rect(ix1, iy1, bboxW, bboxH)));

        results.push_back(
            {fullMask, cv::Rect(ix1, iy1, bboxW, bboxH), cls,
             classNames_.empty() ? std::to_string(cls) : classNames_[cls],
             conf});
      } catch (const std::exception &e) {
        // 处理单个检测异常，继续处理其他检测
        std::cerr << "Error processing detection " << i << ": " << e.what()
                  << std::endl;
      }
    }
    return results;
  }

  std::vector<std::string> classNames_;
};
