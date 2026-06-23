#include "weapon_pipeline.hpp"
#include "yolo_model_detector.hpp"

#include <cameras.hpp>
#include <r2_serial/msg/serial_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

class NineGridBlockNode : public rclcpp::Node {
public:
  enum class Code : std::uint8_t { request = 0, finish };
  enum class Response : std::uint8_t { left = 0, medium = 1, right = 2 };

  NineGridBlockNode() : Node("nine_grid_block_node") {
    this->declare_parameter<std::string>("block_model_path",
                                         "model/block_detect.onnx");
    this->declare_parameter<int>("block_camera_index", 3);
    this->declare_parameter<std::string>("left_pos", "(0.3, 0.5)");
    this->declare_parameter<std::string>("mid_pos", "(0.5, 0.5)");
    this->declare_parameter<std::string>("right_pos", "(0.7, 0.5)");

    std::string block_model_path;
    this->get_parameter("block_model_path", block_model_path);
    if (!std::filesystem::exists(block_model_path)) {
      throw std::invalid_argument("invalid block model path");
    }
    {
      int block_camera_index;
      this->get_parameter("block_camera_index", block_camera_index);
      Cameras::getInstance().set_index(Cameras::CameraIndex::block,
                                       block_camera_index);
    }
    {
      std::string buf;
      float x, y;
      this->get_parameter("left_pos", buf);
      std::sscanf(buf.c_str(), "(%f, %f)", &x, &y);
      left_pos_ = {x, y};
      this->get_parameter("mid_pos", buf);
      std::sscanf(buf.c_str(), "(%f, %f)", &x, &y);
      mid_pos_ = {x, y};
      this->get_parameter("right_pos", buf);
      std::sscanf(buf.c_str(), "(%f, %f)", &x, &y);
      right_pos_ = {x, y};
    }

    detector_ = std::make_unique<YoloOnnxDetector>(
        block_model_path, std::vector<std::string>{"red", "blue", "empty"});

    pos_pub_ = this->create_publisher<std_msgs::msg::UInt8>("pub", 10);
    req_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "sub", 10,
        std::bind(&NineGridBlockNode::request_callback, this,
                  std::placeholders::_1));
  }

  ~NineGridBlockNode() = default;

protected:
  void request_callback(std_msgs::msg::UInt8::SharedPtr code) {
    switch (static_cast<Code>(code->data)) {
    case Code::request: {
      cv::Mat frame = Cameras::getInstance().read(Cameras::CameraIndex::block);
      auto results = detector_->process(frame);
      const float weight = frame.cols, height = frame.rows;
      const cv::Point2f const_left_pos = {left_pos_.x * weight,
                                          left_pos_.y * height};
      const cv::Point2f const_mid_pos = {mid_pos_.x * weight,
                                         mid_pos_.y * height};
      const cv::Point2f const_right_pos = {right_pos_.x * weight,
                                           right_pos_.y * height};

      auto iter = results.begin();
      while (iter != results.end()) {
        if (std::abs(iter->center().y / height - 0.5) > 0.2) {
          iter = results.erase(iter);
        } else {
          ++iter;
        }
      }

      auto left_iter = std::min_element(
          results.cbegin(), results.cend(),
          [pos = const_left_pos](const decltype(results)::value_type &a,
                                 const decltype(results)::value_type &b) {
            cv::Point a_center = a.center(), b_center = b.center();
            float a_dis = std::sqrt(std::pow(pos.x - a_center.x, 2) +
                                    std::pow(pos.y - a_center.y, 2));
            float b_dis = std::sqrt(std::pow(pos.x - b_center.x, 2) +
                                    std::pow(pos.y - b_center.y, 2));
            return a_dis < b_dis;
          });
      if (left_iter != results.cend() &&
          std::sqrt(std::pow(left_iter->center().x / weight - left_pos_.x, 2) +
                    std::pow(left_iter->center().y / height - left_pos_.y, 2)) >
              0.2) {
        left_iter = results.cend();
      }
      auto mid_iter = std::min_element(
          results.cbegin(), results.cend(),
          [pos = const_mid_pos](const decltype(results)::value_type &a,
                                const decltype(results)::value_type &b) {
            cv::Point a_center = a.center(), b_center = b.center();
            float a_dis = std::sqrt(std::pow(pos.x - a_center.x, 2) +
                                    std::pow(pos.y - a_center.y, 2));
            float b_dis = std::sqrt(std::pow(pos.x - b_center.x, 2) +
                                    std::pow(pos.y - b_center.y, 2));
            return a_dis < b_dis;
          });
      if (mid_iter != results.cend() &&
          std::sqrt(std::pow(mid_iter->center().x / weight - mid_pos_.x, 2) +
                    std::pow(mid_iter->center().y / height - mid_pos_.y, 2)) >
              0.2) {
        mid_iter = results.cend();
      }
      auto right_iter = std::min_element(
          results.cbegin(), results.cend(),
          [pos = const_right_pos](const decltype(results)::value_type &a,
                                  const decltype(results)::value_type &b) {
            cv::Point a_center = a.center(), b_center = b.center();
            float a_dis = std::sqrt(std::pow(pos.x - a_center.x, 2) +
                                    std::pow(pos.y - a_center.y, 2));
            float b_dis = std::sqrt(std::pow(pos.x - b_center.x, 2) +
                                    std::pow(pos.y - b_center.y, 2));
            return a_dis < b_dis;
          });
      if (right_iter != results.cend() &&
          std::sqrt(
              std::pow(right_iter->center().x / weight - right_pos_.x, 2) +
              std::pow(right_iter->center().y / height - right_pos_.y, 2)) >
              0.2) {
        right_iter = results.cend();
      }

      std_msgs::msg::UInt8 msg;
      if (mid_iter != results.cend()) {
        msg.data = static_cast<std::uint8_t>(Response::medium);
        pos_pub_->publish(msg);
      } else if (left_iter != results.cend()) {
        msg.data = static_cast<std::uint8_t>(Response::left);
        pos_pub_->publish(msg);
      } else {
        msg.data = static_cast<std::uint8_t>(Response::right);
        pos_pub_->publish(msg);
      }
      RCLCPP_INFO(this->get_logger(),
                  "Received request code, return response: %d",
                  static_cast<int>(msg.data));
      break;
    }
    case Code::finish:
      RCLCPP_INFO(this->get_logger(), "Received finish code");
      rclcpp::shutdown();
      break;
    default:
      RCLCPP_INFO(this->get_logger(), "Received unknown code: %d", code->data);
      break;
    };
  }

private:
  std::unique_ptr<YoloOnnxDetector> detector_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pos_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr req_sub_;
  cv::Point2f left_pos_, mid_pos_, right_pos_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NineGridBlockNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
