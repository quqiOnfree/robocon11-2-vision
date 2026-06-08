#pragma once

#include "geometry_utils.hpp"
#include "yolo_model_classifier.hpp"
#include "yolo_model_detector.hpp"
#include "yolo_model_segmentor.hpp"
#include <opencv2/core/types.hpp>

class BlockFacePipeline {
public:
  // 分类的结果：这张面属于哪个类别
  struct FaceResult {
    cv::Mat warped_face; // 透视矫正后的正方形面图像
    int class_id = -1;
    std::string class_name;
    float confidence = 0.0f;
    cv::Rect source_bbox; // 在原图中检测框的位置（用于索引）
    int face_index = 0;   // 该检测框内的第几个面
  };

  // 构造函数：传入三个模型的路径和对应的类别名
  BlockFacePipeline(const std::filesystem::path &detect_model_path,
                    const std::vector<std::string> &detect_classes,
                    const std::filesystem::path &segment_model_path,
                    const std::vector<std::string>
                        &seg_classes, // 分割模型的类别名（可能有背景类）
                    const std::filesystem::path &classify_model_path,
                    const std::vector<std::string> &face_classes,
                    int warp_size = 512, float detect_conf = 0.5f,
                    float seg_conf = 0.3f, float min_face_area = 500)
      : detector_(detect_model_path, detect_classes),
        segmentor_(segment_model_path, seg_classes), // 实例分割，需要类别名
        classifier_(classify_model_path, face_classes), warp_size_(warp_size),
        detect_conf_(detect_conf), seg_conf_(seg_conf),
        min_face_area_(min_face_area) {}

  /** 阶段1：检测图像中的所有方块 */
  std::vector<YoloOnnxDetector::Detection> detectBlocks(const cv::Mat &image) {
    return detector_.process(image, detect_conf_);
  }

  // 从全图和检测框提取裁剪子图（公开，方便用户获取裁剪图）
  cv::Mat cropBlock(const cv::Mat &image, const cv::Rect &box,
                    int pad = 0) const {
    cv::Rect roi = box;
    roi.x -= pad;
    roi.y -= pad;
    roi.width += 2 * pad;
    roi.height += 2 * pad;
    roi &= cv::Rect(0, 0, image.cols, image.rows);
    if (roi.width <= 0 || roi.height <= 0)
      return cv::Mat();
    return image(roi).clone();
  }

  /** 阶段2：从单个方块区域提取并透视矫正所有面（不分类） */
  std::vector<YoloOnnxSegmentor::SegmentedFace>
  extractInstances(const cv::Mat &image,
                   const YoloOnnxDetector::Detection &block) {
    // 运行分割，直接返回 SegmentedFace 列表
    return segmentor_.process(image, seg_conf_);
  }

  /** 阶段3：对单个矫正后的面进行分类 */
  FaceResult classifyFace(const cv::Mat &cropped_image,
                          const YoloOnnxSegmentor::SegmentedFace &face,
                          const cv::Rect &source_box, int face_index = 0) {
    FaceResult result;
    result.source_bbox = source_box;
    result.face_index = face_index;

    // 1. 从二值掩码中找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(face.binary_mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
      return result;

    // 2. 取面积最大的轮廓，并检查面积阈值
    auto biggest = std::max_element(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });
    if (cv::contourArea(*biggest) < min_face_area_)
      return result;

    // 3. 四边形近似
    auto quad = approxToQuad(*biggest);
    if (quad.size() != 4)
      return result;

    // 4. 透视矫正
    cv::Mat warped = warpPolygonToSquare(cropped_image, quad, warp_size_);

    cv::imshow("Warped Face", warped); // 显示矫正图（调试用）

    // 5. 分类
    auto cls = classifier_.process(warped);
    if (cls.classId < 0)
      return result;

    result.warped_face = warped;
    result.class_id = cls.classId;
    result.class_name = cls.className;
    result.confidence = cls.confidence;
    return result;
  }

  YoloOnnxDetector &getDetector() { return detector_; }
  YoloOnnxSegmentor &getSegmentor() { return segmentor_; }
  YoloOnnxClassifier &getClassifier() { return classifier_; }

  const YoloOnnxDetector &getDetector() const { return detector_; }
  const YoloOnnxSegmentor &getSegmentor() const { return segmentor_; }
  const YoloOnnxClassifier &getClassifier() const { return classifier_; }

private:
  YoloOnnxDetector detector_;
  YoloOnnxSegmentor segmentor_; // 实例分割
  YoloOnnxClassifier classifier_;
  int warp_size_;
  float detect_conf_;
  float seg_conf_;
  float min_face_area_;
};
