#pragma once

#include <onnxruntime_cxx_api.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

// 封装 ONNXRuntime + YOLO 前后处理
template <typename Derived> class YoloOnnxProcessor {
public:
  // letterbox 结果：
  // image: 缩放+填充后的图
  // ratio: 缩放比例（原图 -> 网络输入图）
  // dw/dh: 左右与上下方向的填充偏移（用于把框映射回原图）
  struct LetterboxResult {
    cv::Mat image;
    float ratio;
    float dw;
    float dh;
  };

  int inputH() const { return inputH_; }
  int inputW() const { return inputW_; }
  const std::string &inputName() const { return inputName_; }

  // 对单帧图像做检测
  auto process(const cv::Mat &frame, float confThres = 0.25f) {
    return static_cast<Derived *>(this)->processImpl(frame, confThres);
  }

protected:
  // 构造函数：加载模型、读取输入输出信息
  // useLetterbox: true=检测/分割的letterbox, false=分类的resize+centercrop
  explicit YoloOnnxProcessor(const std::filesystem::path &modelPath,
                             bool useLetterbox = true)
      : env_(ORT_LOGGING_LEVEL_WARNING, "yolo-onnx-cpp"), sessionOptions_(),
        session_(nullptr), memoryInfo_(Ort::MemoryInfo::CreateCpu(
                               OrtArenaAllocator, OrtMemTypeDefault)),
        useLetterbox_(useLetterbox) {
    // 打开图优化，可减少推理耗时
    sessionOptions_.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_ALL);
    // 线程数可按设备调整；嵌入式常见做法是先设 1，再按性能调参
    sessionOptions_.SetIntraOpNumThreads(1);

    session_ = Ort::Session(env_, modelPath.c_str(), sessionOptions_);

    Ort::AllocatorWithDefaultOptions allocator;
    auto inputNameAllocated = session_.GetInputNameAllocated(0, allocator);
    auto outputNameAllocated = session_.GetOutputNameAllocated(0, allocator);
    inputName_ = inputNameAllocated.get();
    outputName_ = outputNameAllocated.get();

    // 一般是 [1, 3, H, W]
    auto inputInfo = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
    const std::vector<int64_t> inputShape = inputInfo.GetShape();
    inputH_ = (inputShape.size() >= 4 && inputShape[2] > 0)
                  ? static_cast<int>(inputShape[2])
                  : 640;
    inputW_ = (inputShape.size() >= 4 && inputShape[3] > 0)
                  ? static_cast<int>(inputShape[3])
                  : 640;
  }

  ~YoloOnnxProcessor() = default;

  // letterbox：保持长宽比缩放后，再用灰边填充到固定尺寸
  // 这样能减少几何变形，通常比直接拉伸更稳
  static LetterboxResult
  letterbox(const cv::Mat &src, int newH, int newW,
            const cv::Scalar &color = cv::Scalar(114, 114, 114)) {
    const int h = src.rows;
    const int w = src.cols;
    const float r = std::min(static_cast<float>(newH) / static_cast<float>(h),
                             static_cast<float>(newW) / static_cast<float>(w));

    const int nw = static_cast<int>(std::round(w * r));
    const int nh = static_cast<int>(std::round(h * r));

    const float dw = (newW - nw) / 2.0f;
    const float dh = (newH - nh) / 2.0f;

    cv::Mat resized;
    if (w != nw || h != nh) {
      cv::resize(src, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    } else {
      resized = src.clone();
    }

    const int top = static_cast<int>(std::round(dh - 0.1f));
    const int bottom = static_cast<int>(std::round(dh + 0.1f));
    const int left = static_cast<int>(std::round(dw - 0.1f));
    const int right = static_cast<int>(std::round(dw + 0.1f));

    cv::Mat out;
    cv::copyMakeBorder(resized, out, top, bottom, left, right,
                       cv::BORDER_CONSTANT, color);
    return {out, r, dw, dh};
  }

  // 预处理（检测/分割用 letterbox，分类用 resize+centercrop）
  std::vector<float> preprocess(const cv::Mat &bgr, float &ratio, float &dw,
                                float &dh) const {
    cv::Mat rgb;
    if (useLetterbox_) {
      const LetterboxResult lb = letterbox(bgr, inputH_, inputW_);
      ratio = lb.ratio;
      dw = lb.dw;
      dh = lb.dh;
      cv::cvtColor(lb.image, rgb, cv::COLOR_BGR2RGB);
    } else {
      // 分类模式：短边缩放到 input_size（保持宽高比），长边中心裁剪
      int h = bgr.rows, w = bgr.cols;
      float scale =
          static_cast<float>(inputW_) / static_cast<float>(std::min(h, w));
      int nh = static_cast<int>(std::round(h * scale));
      int nw = static_cast<int>(std::round(w * scale));
      cv::Mat resized;
      cv::resize(bgr, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
      int top = (nh - inputH_) / 2;
      int left = (nw - inputW_) / 2;
      top = std::max(0, std::min(top, nh - inputH_));
      left = std::max(0, std::min(left, nw - inputW_));
      cv::Mat cropped = resized(cv::Rect(left, top, inputW_, inputH_)).clone();
      cv::cvtColor(cropped, rgb, cv::COLOR_BGR2RGB);
      ratio = 1.0f;  // 分类不需要反向映射，ratio/dw/dh 仅占位
      dw = 0.0f;
      dh = 0.0f;
    }

    cv::Mat f32;
    rgb.convertTo(f32, CV_32F, 1.0 / 255.0);

    // HWC -> CHW
    std::vector<cv::Mat> channels(3);
    cv::split(f32, channels);

    const size_t planeSize =
        static_cast<size_t>(inputH_) * static_cast<size_t>(inputW_);
    std::vector<float> inputTensorValues(3ULL * planeSize);
    for (int c = 0; c < 3; ++c) {
      std::memcpy(inputTensorValues.data() + c * planeSize, channels[c].data,
                  planeSize * sizeof(float));
    }

    return inputTensorValues;
  }

  std::vector<Ort::Value> runInference(const std::vector<float> &inputData) {
    std::array<int64_t, 4> tensorShape = {1, 3, inputH_, inputW_};
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo_, const_cast<float *>(inputData.data()), inputData.size(),
        tensorShape.data(), tensorShape.size());

    std::array<const char *, 1> inputNames = {inputName_.c_str()};
    std::array<const char *, 1> outputNames = {outputName_.c_str()};

    return session_.Run(Ort::RunOptions{nullptr}, inputNames.data(),
                        &inputTensor, 1, outputNames.data(), 1);
  }

  std::vector<Ort::Value> runInferenceAll(const std::vector<float> &inputData) {
    std::array<int64_t, 4> tensorShape = {1, 3, inputH_, inputW_};
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo_, const_cast<float *>(inputData.data()), inputData.size(),
        tensorShape.data(), tensorShape.size());

    Ort::AllocatorWithDefaultOptions allocator;
    size_t numOut = session_.GetOutputCount();
    std::vector<std::string> outNameStrs;
    std::vector<const char *> outNamePtrs;
    outNameStrs.reserve(numOut);
    outNamePtrs.reserve(numOut);
    for (size_t i = 0; i < numOut; ++i) {
      auto nameAlloc = session_.GetOutputNameAllocated(i, allocator);
      outNameStrs.push_back(nameAlloc.get());
      outNamePtrs.push_back(outNameStrs.back().c_str());
    }

    std::array<const char *, 1> inputNames = {inputName_.c_str()};
    return session_.Run(Ort::RunOptions{nullptr}, inputNames.data(),
                        &inputTensor, 1, outNamePtrs.data(), numOut);
  }

  // ONNX Runtime 关键对象
  Ort::Env env_;
  Ort::SessionOptions sessionOptions_;
  Ort::Session session_;
  Ort::MemoryInfo memoryInfo_;

  // 输入输出信息
  std::string inputName_;
  std::string outputName_;
  int inputH_ = 640;
  int inputW_ = 640;
  bool useLetterbox_ = true;
};
