#include "weapon_pipeline.hpp"

#include <rclcpp/rclcpp.hpp>
#include <fast_lio/msg/serial_packet.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int16.hpp>

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

using std::placeholders::_1;

class WeaponNode : public rclcpp::Node {
public:
  enum State : uint8_t {
    GRIPPER_IDLE      = 0,
    GRIPPER_TRACKING  = 1,
    GRIPPER_GRAB      = 2
  };

  WeaponNode()
      : Node("weapon_node"), current_state_(GRIPPER_IDLE),
        frame_count_(0) , grab_triggered(false){
    // ---- 声明参数 ----
    this->declare_parameter("model_path", "model/weapon_pickup.onnx");
    this->declare_parameter("camera_index", 1);
    this->declare_parameter("conf_thres", 0.25);
    this->declare_parameter("show_window", true);

    // ---- 订阅状态指令话题 ----
    state_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/vision/weapon_cmd_state", 10,
        std::bind(&WeaponNode::state_callback, this, _1));
    
    state_sub2_ = this->create_subscription<std_msgs::msg::UInt8>("/vision/weapon_cmd_state_2", 10,
        std::bind(&WeaponNode::state_callback, this, _1));

    // ---- 发布距离数据（供其他节点使用） ----
    distance_pub_ = this->create_publisher<std_msgs::msg::Int16>(
        "/vision/weapon_distance", 10);

    RCLCPP_INFO(this->get_logger(),
        "武器检测节点已启动，当前 [GRIPPER_IDLE] 状态，等待 /vision/weapon_cmd_state 指令...");
  }

  bool init() {
    std::string model_path;
    int camera_index;
    bool show_window;

    this->get_parameter("model_path", model_path);
    this->get_parameter("camera_index", camera_index);
    this->get_parameter("conf_thres", conf_thres_);
    this->get_parameter("show_window", show_window);

    if (!std::filesystem::exists(model_path)) {
      RCLCPP_ERROR(this->get_logger(), "模型文件不存在: %s", model_path.c_str());
      return false;
    }

    // ---- downlink 发布者 ----
    packet_pub_ = this->create_publisher<fast_lio::msg::SerialPacket>(
        "/r2/downlink/packet", 10);
    RCLCPP_INFO(this->get_logger(), "downlink 发布者已创建: /r2/downlink/packet");

    // ---- 管线 ----
    RCLCPP_INFO(this->get_logger(), "加载模型: %s", model_path.c_str());
    pipeline_ = std::make_unique<WeaponPipeline>(model_path,
        std::vector<std::string>{"weapon", "itf"}, 0, 1, conf_thres_);
    RCLCPP_INFO(this->get_logger(), "模型输入: %dx%d",
        pipeline_->getDetector().inputW(), pipeline_->getDetector().inputH());

    // ---- 摄像头 ----
    RCLCPP_INFO(this->get_logger(), "打开摄像头 %d...", camera_index);
    cap_.open(camera_index);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开摄像头 %d", camera_index);
      return false;
    }

    show_window_ = show_window;
    t0_ = std::chrono::steady_clock::now();

    // ---- 处理定时器 (~30 fps) ----
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&WeaponNode::process_frame, this));

    return true;
  }

private:
  void state_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    uint8_t prev = current_state_;
    current_state_ = msg->data;

    switch (current_state_) {
      case GRIPPER_IDLE:
        RCLCPP_INFO(this->get_logger(), "收到指令 [GRIPPER_IDLE]：待机，停止发送");
        break;
      case GRIPPER_TRACKING:
        RCLCPP_INFO(this->get_logger(), "收到指令 [GRIPPER_TRACKING]：开始视觉追踪武器");
        break;
      case GRIPPER_GRAB:
        RCLCPP_INFO(this->get_logger(), "收到指令 [GRIPPER_GRAB]：位置到达，执行夹取, 即将关闭");
        grab_triggered = true;
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "收到未知状态码: %d，回退到 GRIPPER_IDLE", msg->data);
        current_state_ = GRIPPER_IDLE;
        break;
    }
  }

  void process_frame() {
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty())
      return;
    frame_count_++;

    // GRIPPER_IDLE 状态下不做检测、不发送
    if (current_state_ == GRIPPER_IDLE) {
      if (show_window_) {
        cv::imshow("Weapon Detection", frame);
        cv::waitKey(1);
      }
      return;
    }

    // ---- 检测 ----
    auto target = pipeline_->process(frame);

    // ---- 发布距离到 ROS2 话题 ----
    if (target.found) {
      auto msg = std_msgs::msg::Int16();
      msg.data = static_cast<int16_t>(std::floor(target.distance));
      distance_pub_->publish(msg);

      // ---- downlink 发送 ----
      publish_packet(target.distance);
    }

    // ---- 终端输出 ----
    if (target.found) {
      std::cout << "ITF: (" << target.itf_center.x << ", "
                << target.itf_center.y << ")"
                << "  dist=" << target.distance
                << "  score=" << target.itf_score
                << "  weapon=(" << target.weapon_center.x << ","
                << target.weapon_center.y << ")\n";
    } else if (target.weapon_center != cv::Point(0, 0)) {
      std::cout << "无 ITF  weapon=(" << target.weapon_center.x
                << "," << target.weapon_center.y << ")\n";
    }

    // ---- 可视化 ----
    if (show_window_) {
      cv::Mat display = frame.clone();
      pipeline_->drawTarget(display, target);

      auto t1 = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(t1 - t0_).count();
      if (elapsed >= 1.0) {
        double fps = frame_count_ / elapsed;
        cv::putText(display, cv::format("FPS: %.1f", fps),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 255), 2);
        // 状态显示
        const char* state_str = current_state_ == GRIPPER_IDLE ? "GRIPPER_IDLE" :
                                current_state_ == GRIPPER_TRACKING ? "GRIPPER_TRACKING" : "GRIPPER_GRAB";
        cv::putText(display, cv::format("State: %s", state_str),
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 255, 0), 2);
        frame_count_ = 0;
        t0_ = t1;
      }

      cv::imshow("Weapon Detection", display);
      if (cv::waitKey(1) == 27) {
        rclcpp::shutdown();
      }
    }
  
    if(grab_triggered == true)
    {
      RCPCPP_INFO(this->get_logger(),"GRIPPER_GRAB 完成，关闭节点");
      rclcpp::shutdown();
    }
  }

  void publish_packet(float distance) {
    const double rounded = std::floor(distance);
    if (!std::isfinite(rounded) ||
        rounded < std::numeric_limits<int16_t>::min() ||
        rounded > std::numeric_limits<int16_t>::max()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "视觉距离超出 int16_t 范围，跳过下发: %.3f", distance);
      return;
    }

    const int16_t number = static_cast<int16_t>(rounded);
    const auto bits = static_cast<uint16_t>(number);

    fast_lio::msg::SerialPacket msg;
    msg.code = weapon_code_;  // weapon: 0x0001
    msg.clear_pending = false;
    msg.payload.push_back(static_cast<uint8_t>(bits & 0xff));
    msg.payload.push_back(static_cast<uint8_t>((bits >> 8) & 0xff));

    packet_pub_->publish(msg);
  }

  // ---- ROS2 ----
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub2_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr distance_pub_;
  rclcpp::Publisher<fast_lio::msg::SerialPacket>::SharedPtr packet_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint8_t current_state_;
  bool grab_triggered;

  static constexpr uint16_t weapon_code_ = 0x0001;

  // ---- 管线 ----
  std::unique_ptr<WeaponPipeline> pipeline_;
  float conf_thres_;

  // ---- 摄像头与显示 ----
  cv::VideoCapture cap_;
  bool show_window_;
  int frame_count_;
  std::chrono::steady_clock::time_point t0_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WeaponNode>();
  if (!node->init()) {
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::spin(node);
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}
