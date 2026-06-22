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

  NineGridBlockNode() : Node("nine_grid_block_node") {
    this->declare_parameter("block_model_path", "model/block_detect.onnx");
    this->declare_parameter("block_camera_index", 3);

    std::string block_model_path;
    this->get_parameter("block_model_path", block_model_path);
    if (!std::filesystem::exists(block_model_path)) {
      throw std::invalid_argument("invalid block model path");
    }
    int block_camera_index = -1;
    this->get_parameter("block_camera_index", block_camera_index);
    if (block_camera_index == -1) {
      throw std::invalid_argument("invalid block camera index");
    }
    Cameras::getInstance().set_index(Cameras::CameraIndex::block,
                                     block_camera_index);

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
      auto results = detector_->process(
          Cameras::getInstance().read(Cameras::CameraIndex::block));

      for (auto &res : std::as_const(results)) {
        cv::Point center = res.center();
      }
      RCLCPP_INFO(this->get_logger(), "Received request code");
      break;
    }
    case Code::finish:
      RCLCPP_INFO(this->get_logger(), "Received finish code");
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
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NineGridBlockNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
