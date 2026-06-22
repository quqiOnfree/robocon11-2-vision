#pragma once

#include "yolo_model_interface.hpp"

class YoloOnnxClassifier : public YoloOnnxProcessor<YoloOnnxClassifier> {
  friend class YoloOnnxProcessor<YoloOnnxClassifier>;

public:
  struct ClassResult {
    int classId;
    std::string className;
    float confidence;
  };

  YoloOnnxClassifier(const std::filesystem::path &modelPath,
                     const std::vector<std::string> &classNames)
      : YoloOnnxProcessor<YoloOnnxClassifier>(modelPath, false),
        classNames_(classNames) {}

private:
  ClassResult processImpl(const cv::Mat &frame, float /*confThres unused*/) {
    float ratio, dw, dh;
    auto inputTensor = preprocess(frame, ratio, dw, dh);
    auto outputs = runInference(inputTensor);

    if (outputs.empty() || !outputs[0].IsTensor())
      return {};
    auto outShape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    if (outShape.size() != 2 || outShape[0] != 1)
      return {};

    const float *outData = outputs[0].GetTensorData<float>();
    int numClasses = static_cast<int>(outShape[1]);
    auto maxIt = std::max_element(outData, outData + numClasses);
    int classId = static_cast<int>(std::distance(outData, maxIt));
    float conf = *maxIt; // 如果输出是 logits，可加 softmax

    return {classId, classNames_[classId], conf};
  }

  std::vector<std::string> classNames_;
};
