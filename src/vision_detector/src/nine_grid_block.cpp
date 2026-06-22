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
  NineGridBlockNode() : Node("nine_grid_block_node") {
    pos_pub_ = this->create_publisher<std_msgs::msg::UInt8>("pub", 10);
    req_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "sub", 10,
        std::bind(&NineGridBlockNode::request_callback, this,
                  std::placeholders::_1));
  }

  ~NineGridBlockNode() = default;

protected:
  void request_callback(std::shared_ptr<std_msgs::msg::Empty>) {}

private:
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::UInt8>> pos_pub_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> req_sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NineGridBlockNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}