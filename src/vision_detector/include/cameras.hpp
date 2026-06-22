#pragma once

#include <mutex>
#include <opencv2/opencv.hpp>
#include <optional>
#include <unordered_map>

class Cameras {
public:
  ~Cameras() = default;
  Cameras(const Cameras &) = delete;
  Cameras(Cameras &&) = delete;
  Cameras &operator=(const Cameras &) = delete;
  Cameras &operator=(Cameras &&) = delete;

  enum CameraIndex { null = 0, weapon, pole, block };

  static Cameras &getInstance() {
    static Cameras cams;
    return cams;
  }

  void set_index(CameraIndex cidx, int idx) {
    std::lock_guard<std::mutex> lock(mtx_);
    cap_index_[cidx] = idx;
  }

  CameraIndex get_index() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return cur_idx_;
  }

  int get_camera_index(CameraIndex idx) const {
    std::lock_guard<std::mutex> lock(mtx_);
    auto iter = cap_index_.find(idx);
    if (iter == cap_index_.cend()) {
      return -1;
    }
    return iter->second;
  }

  cv::Mat read(CameraIndex idx) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto &cst_cap_idx = std::as_const(cap_index_);
    auto iter = cst_cap_idx.find(idx);
    if (iter == cst_cap_idx.cend()) {
      return {};
    }
    if (cur_idx_ != idx) {
      cap_.release();
      cap_.open(iter->second);
      cur_idx_ = idx;
    }
    if (!cap_.isOpened()) {
      cap_.open(iter->second);
      if (!cap_.isOpened()) {
        return {};
      }
    }
    cv::Mat frame;
    if (!cap_.read(frame)) {
      return {};
    }
    return frame;
  }

  void release() {
    std::lock_guard<std::mutex> lock(mtx_);
    cap_.release();
    cur_idx_ = CameraIndex::null;
  }

protected:
  Cameras() = default;

private:
  cv::VideoCapture cap_;
  CameraIndex cur_idx_{CameraIndex::null};
  std::unordered_map<CameraIndex, int> cap_index_;
  mutable std::mutex mtx_;
};
