#pragma once

#include "yolo_model_detector.hpp"
#include <opencv2/core/types.hpp>

class WeaponPipeline {
public:
  // 匹配结果：中心 weapon → 框内 ITF 的对应关系
  struct WeaponTarget {
    bool found = false;
    cv::Point weapon_center;  // 离画面中心最近的 weapon 中心点
    cv::Point itf_center;     // 匹配到的 ITF 中心点
    float itf_score = 0.0f;   // ITF 置信度
    float distance = 0.0f;    // X 轴距离（右正左负）
    cv::Rect weapon_box;      // weapon 检测框
    cv::Rect itf_box;         // ITF 检测框
  };

  // 构造函数
  // model_path:    ONNX 模型路径
  // class_names:   模型输出的类别名，如 {"weapon", "itf"}
  // weapon_class_id: weapon 在类别列表中的索引
  // itf_class_id: ITF 在类别列表中的索引
  // conf_thres:    置信度阈值
  WeaponPipeline(const std::filesystem::path &model_path,
                 const std::vector<std::string> &class_names,
                 int weapon_class_id = 0, int itf_class_id = 1,
                 float conf_thres = 0.25f)
      : detector_(model_path, class_names), weapon_class_id_(weapon_class_id),
        itf_class_id_(itf_class_id), conf_thres_(conf_thres) {}

  // ===================================================================
  // Stage 1: 检测 — 对一帧图像运行 YOLO 推理
  // ===================================================================
  std::vector<YoloOnnxDetector::Detection>
  detect(const cv::Mat &frame) {
    return detector_.process(frame, conf_thres_);
  }

  // ===================================================================
  // Stage 2: 匹配 — 从检测结果中找 "中心weapon → 框内ITF"
  //
  // 规则:
  //   1. 从所有 detection 中筛选 weapon 类别
  //   2. 取离画面中心最近的 weapon
  //   3. 从所有 detection 中筛选 ITF 类别
  //   4. 找位于 weapon 框内、score 最高的 ITF
  //   5. 计算 ITF → 画面中心的 X 轴距离（右正左负）
  // ===================================================================
  WeaponTarget
  match(const std::vector<YoloOnnxDetector::Detection> &dets,
        const cv::Size &frame_size) const {
    WeaponTarget result;
    result.found = false;

    const cv::Point img_center(frame_size.width / 2,
                               frame_size.height / 2);

    // 1. 筛选 weapon
    const auto weapons =
        YoloOnnxDetector::getByClass(dets, weapon_class_id_);
    if (weapons.empty())
      return result;

    // 2. 找离画面中心最近的 weapon
    const auto *center_weapon =
        YoloOnnxDetector::nearestToCenter(weapons, frame_size);
    if (!center_weapon)
      return result;

    // 3. 筛选 ITF
    const auto itfs = YoloOnnxDetector::getByClass(dets, itf_class_id_);

    // 4. 找在 weapon 框内 score 最高的 ITF
    const YoloOnnxDetector::Detection *best_itf = nullptr;
    for (const auto &itf : itfs) {
      if (center_weapon->box.contains(itf.center())) {
        if (!best_itf || itf.score > best_itf->score)
          best_itf = &itf;
      }
    }

    if (!best_itf)
      return result;

    // 5. 填充结果
    result.found = true;
    result.weapon_center = center_weapon->center();
    result.weapon_box = center_weapon->box;
    result.itf_center = best_itf->center();
    result.itf_box = best_itf->box;
    result.itf_score = best_itf->score;

    // 6. 计算 X 轴距离（右正左负）
    result.distance = computeDistance(result.itf_center, img_center);

    return result;
  }

  // ===================================================================
  // Stage 3: 计算距离 — 点到画面中心的 X 轴距离（右正左负）
  // ===================================================================
  static float computeDistance(const cv::Point &target,
                               const cv::Point &img_center) {
    return static_cast<float>(target.x - img_center.x);
  }

  // ===================================================================
  // 便捷方法: 一步完成 detect + match
  // ===================================================================
  WeaponTarget process(const cv::Mat &frame) {
    auto dets = detect(frame);
    return match(dets, frame.size());
  }

  // ===================================================================
  // 可视化 — 在图像上绘制检测框、标记点和距离连线
  // ===================================================================
  void drawTarget(cv::Mat &display, const WeaponTarget &target) const {
    const cv::Point img_center(display.cols / 2, display.rows / 2);

    // 画面中心十字
    cv::drawMarker(display, img_center, cv::Scalar(0, 255, 0),
                   cv::MARKER_CROSS, 20, 2);

    // 中心 weapon 标记（蓝色）
    cv::drawMarker(display, target.weapon_center, cv::Scalar(255, 0, 0),
                   cv::MARKER_CROSS, 30, 2);

    if (!target.found)
      return;

    // ITF 标记（红色）
    cv::drawMarker(display, target.itf_center, cv::Scalar(0, 0, 255),
                   cv::MARKER_CROSS, 30, 2);

    // 连线（青色）
    cv::line(display, img_center, target.itf_center,
             cv::Scalar(0, 255, 255), 2);

    // 距离标注
    cv::putText(display,
                "dist=" + std::to_string(static_cast<int>(target.distance)),
                cv::Point(img_center.x + 15, img_center.y - 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
  }

  // ===================================================================
  // 访问器
  // ===================================================================
  YoloOnnxDetector &getDetector() { return detector_; }
  const YoloOnnxDetector &getDetector() const { return detector_; }

  float getConfThres() const { return conf_thres_; }
  void setConfThres(float conf) { conf_thres_ = conf; }

private:
  YoloOnnxDetector detector_;
  int weapon_class_id_;
  int itf_class_id_;
  float conf_thres_;
};
