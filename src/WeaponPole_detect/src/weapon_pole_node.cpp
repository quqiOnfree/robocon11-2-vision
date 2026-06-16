#include "yolo_model_detector.hpp"
#include "weapon_pipeline.hpp"

#include <rclcpp/rclcpp.hpp>
#include <r2_serial/msg/serial_packet.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <cameras.hpp>

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

class PoleNode : public rclcpp::Node {
public:
  enum State : uint8_t {
    IDLE      = 0,
    WEAPON_TRACKING  = 1,
    WAITING          = 2,
    POLE_TRACKING  = 3,
    GRAB      = 4
  };

  PoleNode(): 
        Node("weapon_pole_node"), current_state_(IDLE),
        frame_count_(0) , grab_triggered_(false) /*, cam_(Cameras::getInstance())*/{
    // 声明参数
    this->declare_parameter("weapon_model_path", "model/weapon_pickup.onnx");
    this->declare_parameter("pole_model_path", "model/pole_detect.onnx");
    this->declare_parameter("weapon_camera_index", 0);
    this->declare_parameter("pole_camera_index", 2);
    this->declare_parameter("conf_thres", 0.25);
    this->declare_parameter("show_window", true);

    // 订阅状态指令话题
    state_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/vision/weapon_pole_cmd_state", 10,
                  std::bind(&PoleNode::state_callback, this, _1));

    state_sub2_ = this->create_subscription<std_msgs::msg::UInt8>("/vision/weapon_pole_cmd_state_2", 10,
                  std::bind(&PoleNode::state_callback, this, _1));

    // 发布距离数据
    distance_pub_ = this->create_publisher<std_msgs::msg::Int16>(
        "/vision/distance", 10);

    RCLCPP_INFO(this->get_logger(),
        "视觉检测节点已启动，当前 [IDLE] 状态，等待 /vision/weapon_pole_cmd_state 指令...");
  }

  bool init() {
    std::string weapon_model_path;
    std::string pole_model_path;
    int weapon_camera_index;
    int pole_camera_index;
    bool show_window;

    this->get_parameter("weapon_model_path", weapon_model_path);
    this->get_parameter("pole_model_path", pole_model_path);
    this->get_parameter("weapon_camera_index", weapon_camera_index);
    this->get_parameter("pole_camera_index", pole_camera_index);
    this->get_parameter("conf_thres", conf_thres_);
    this->get_parameter("show_window", show_window);

    weapon_camera_index_ = weapon_camera_index;
    pole_camera_index_ = pole_camera_index;

    if (!std::filesystem::exists(weapon_model_path)) {
      RCLCPP_ERROR(this->get_logger(), "武器模型文件不存在: %s", weapon_model_path.c_str());
      return false;
    }
    if (!std::filesystem::exists(pole_model_path)) {
      RCLCPP_ERROR(this->get_logger(), "长杆模型文件不存在: %s", pole_model_path.c_str());
      return false;
    }

    // downlink 发布者
    packet_pub_ = this->create_publisher<r2_serial::msg::SerialPacket>( "/r2/downlink/packet", 10);
    RCLCPP_INFO(this->get_logger(), "downlink 发布者已创建: /r2/downlink/packet");

    // 管线
    RCLCPP_INFO(this->get_logger(), "加载武器模型: %s", weapon_model_path.c_str());
    pipeline_ = std::make_unique<WeaponPipeline>(weapon_model_path, std::vector<std::string>{"weapon", "itf"}, 0, 1, conf_thres_);
    RCLCPP_INFO(this->get_logger(), "模型输入: %dx%d", pipeline_->getDetector().inputW(), pipeline_->getDetector().inputH());

    // 长杆检测器
    RCLCPP_INFO(this->get_logger(), "加载长杆模型: %s", pole_model_path.c_str());
    pole_detector_ = std::make_unique<YoloOnnxDetector>(pole_model_path, std::vector<std::string>{"pole"});
    RCLCPP_INFO(this->get_logger(), "长杆模型输入: %dx%d", pole_detector_->inputW(), pole_detector_->inputH());

    // 摄像头：只打开武器摄像头，长杆摄像等切换时再打开
    RCLCPP_INFO(this->get_logger(), "打开摄像头 %d...", weapon_camera_index);
    cap_.open(weapon_camera_index);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开摄像头 %d", weapon_camera_index);
      return false;
    }
    active_camera_ = weapon_camera_index_;

    show_window_ = show_window;
    t0_ = std::chrono::steady_clock::now();

    // 处理定时器 (~30 fps)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&PoleNode::process_frame, this));

    // 显示定时器 (~100 fps)
    display_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PoleNode::update_display, this));

    return true;
  }

private:
  bool switch_camera(int cam_index) {
    if (active_camera_ == cam_index)
      return true;

    // 先关闭当前摄像头
    if (active_camera_ == weapon_camera_index_)
      cap_.release();
    else if (active_camera_ == pole_camera_index_)
      cap2_.release();

    // 打开目标摄像头
    bool ok = false;
    if (cam_index == weapon_camera_index_) {
      ok = cap_.open(weapon_camera_index_);
    } else if (cam_index == pole_camera_index_) {
      ok = cap2_.open(pole_camera_index_);
    }

    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "无法打开摄像头 %d", cam_index);
      active_camera_ = -1;
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "切换到摄像头 %d", cam_index);
    active_camera_ = cam_index;
    return true;
  }

  void update_display() {
    if (!show_window_ || display_frame_.empty())
      return;
    cv::imshow("Weapon&&Pole Detection", display_frame_);
    if (cv::waitKey(1) == 27) {
      rclcpp::shutdown();
    }
  }

  void state_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    current_state_ = msg->data;

    switch (current_state_) {
      case IDLE:
        RCLCPP_INFO(this->get_logger(), "收到指令 [IDLE]：待机，停止发送");
        break;
      case WEAPON_TRACKING:
        RCLCPP_INFO(this->get_logger(), "收到指令 [WEAPON_TRACKING]：开始追踪武器头");
        break;
      case WAITING:
        RCLCPP_INFO(this->get_logger(), "收到指令 [WAITING]：武器头夹取任务完成，等待接收夹取长杆状态指令");
        break;
      case POLE_TRACKING:
        RCLCPP_INFO(this->get_logger(), "收到指令 [POLE_TRACKING]：开始追踪长杆");
        break;
      case GRAB:
        RCLCPP_INFO(this->get_logger(), "收到指令 [GRAB]：位置到达, 武器&&长杆节点即将关闭");
        grab_triggered_ = true;
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "收到未知状态码: %d，回退到 IDLE", msg->data);
        current_state_ = IDLE;
        break;
    }
  }

  void process_frame()
  {
    cv::Mat frame;

    // IDLE 状态下不做检测、不发送
    if (current_state_ == IDLE)
    {
      if (!switch_camera(weapon_camera_index_)) return;
      cap_ >> frame;
      // frame = cam_.read(Cameras::weapon);
      if (frame.empty())
      {
          RCLCPP_INFO(this->get_logger(),"获取图片失败");
          return;
      }
      frame_count_++;
      if (show_window_)
      display_frame_ = frame.clone();
      return;
    }
    // 武器检测
    else if(current_state_ == WEAPON_TRACKING)
    {
      if (!switch_camera(weapon_camera_index_)) return;
      cap_ >> frame;
      // frame = cam_.read(Cameras::weapon);
      if (frame.empty())
      {
          RCLCPP_INFO(this->get_logger(),"获取图片失败");
          return;
      }
      frame_count_++;
      
      // 检测
      auto target = pipeline_->process(frame);

      // 发布距离到 ROS2 话题
      if (target.found) {
      auto msg = std_msgs::msg::Int16();
      msg.data = static_cast<int16_t>(std::floor(target.distance));
      distance_pub_->publish(msg);

      // downlink 发送
      publish_packet(target.distance);
      }

      // 终端输出
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

      // 可视化
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
          const char* state_str = current_state_ == IDLE ? "IDLE" :
                                  current_state_ == WEAPON_TRACKING ? "WEAPON_TRACKING" : "WAITING";
          cv::putText(display, cv::format("State: %s", state_str),
                      cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                      cv::Scalar(0, 255, 0), 2);
          frame_count_ = 0;
          t0_ = t1;
      }

      display_frame_ = display.clone();
      }
    }
    // 等待状态
    else if(current_state_ == WAITING)
    {
      if (!switch_camera(pole_camera_index_)) return;
      cap2_ >> frame;
      // frame = cam_.read(Cameras::pole);
      if (frame.empty())
      {
        RCLCPP_INFO(this->get_logger(),"获取图片失败");
        return;
      }
      frame_count_++;
      if (show_window_)
      display_frame_ = frame.clone();
      return;
    }
    // 长杆检测状态
    else if(current_state_ == POLE_TRACKING)
    {
      if (!switch_camera(pole_camera_index_)) return;
      cap2_ >> frame;
      // frame = cam_.read(Cameras::pole);
      if (frame.empty())
      {
          RCLCPP_INFO(this->get_logger(),"获取图片失败");
          return;
      }
      frame_count_++;

      // 检测
      auto dets = pole_detector_->process(frame, conf_thres_);
      const auto *best = YoloOnnxDetector::nearestToCenter(dets, frame.size());
  
      // 计算 X 轴距离（右正左负）
      float distance = 0.0f;
      cv::Point pole_center(0, 0);
      if (best) {
      cv::Point img_center(frame.cols / 2, frame.rows / 2);
      pole_center = best->center();
      distance = static_cast<float>(pole_center.x - img_center.x);
      }
  
      // 发布距离到 ROS2 话题
      if (best) {
      auto msg = std_msgs::msg::Int16();
      msg.data = static_cast<int16_t>(std::floor(distance));
      distance_pub_->publish(msg);
  
      // downlink 发送
      publish_packet(distance);
      }
  
      // 终端输出
      if (best) {
      std::cout << "pole=(" << pole_center.x << "," << pole_center.y << ")"
                  << "  dist=" << distance
                  << "  score=" << best->score << "\n";
      }
  
      // 可视化
      if (show_window_) {
      cv::Mat display = frame.clone();
  
      const cv::Point img_center(display.cols / 2, display.rows / 2);
  
      // 画面中心十字
      cv::drawMarker(display, img_center, cv::Scalar(0, 255, 0),
                      cv::MARKER_CROSS, 20, 2);
  
      if (best) {
          // pole 标记
          cv::drawMarker(display, pole_center, cv::Scalar(0, 0, 255),
                      cv::MARKER_CROSS, 30, 2);
  
          // 连线
          cv::line(display, img_center, pole_center,
                  cv::Scalar(0, 255, 255), 2);
  
          // 距离标注
          cv::putText(display,
                      "dist=" + std::to_string(static_cast<int>(distance)),
                      cv::Point(img_center.x + 15, img_center.y - 15),
                      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
      }
  
  
      // FPS
      auto t1 = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(t1 - t0_).count();
      if (elapsed >= 1.0) 
      {
          double fps = frame_count_ / elapsed;
          cv::putText(display, cv::format("FPS: %.1f", fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                      cv::Scalar(0, 255, 255), 2);
          
          const char* state_str = current_state_ == IDLE ? "IDLE" : current_state_ == POLE_TRACKING ? "POLE_TRACKING" : "GRAB";
          
          cv::putText(display, cv::format("State: %s", state_str), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                      cv::Scalar(0, 255, 0), 2);
          frame_count_ = 0;
          t0_ = t1;
      }
  
      display_frame_ = display.clone();
      }
    }

    if (grab_triggered_) 
    {
      RCLCPP_INFO(this->get_logger(), "GRAB 完成，关闭节点");
      rclcpp::shutdown();
    }
  }

  void publish_packet(float distance) 
  {
    const double rounded = std::floor(distance);
    if (!std::isfinite(rounded) ||
        rounded < std::numeric_limits<int16_t>::min() ||
        rounded > std::numeric_limits<int16_t>::max()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "视觉距离超出 int16_t 范围，跳过下发: %.3f", distance);
      return;
    }

    const int16_t number = static_cast<int16_t>(rounded);
    const auto bits = static_cast<uint16_t>(number);

    r2_serial::msg::SerialPacket msg;
    msg.code = weapon_pole_code_;  // 
    msg.clear_pending = false;
    msg.payload.push_back(static_cast<uint8_t>(bits & 0xff));
    msg.payload.push_back(static_cast<uint8_t>((bits >> 8) & 0xff));

    packet_pub_->publish(msg);
  }

  // ROS2
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub2_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr distance_pub_;
  rclcpp::Publisher<r2_serial::msg::SerialPacket>::SharedPtr packet_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr display_timer_;
  uint8_t current_state_;
  bool grab_triggered_;

  // 武器和长杆总结到同一个
  static constexpr uint16_t weapon_pole_code_ = 0x0001;

  // 检测器
  std::unique_ptr<YoloOnnxDetector> pole_detector_;
  // 管线
  std::unique_ptr<WeaponPipeline> pipeline_;
  float conf_thres_;

  // 摄像头与显示
  cv::VideoCapture cap_;
  cv::VideoCapture cap2_;
  int weapon_camera_index_ = 0;
  int pole_camera_index_ = 2;
  int active_camera_ = -1;   // -1: 未打开, 0: cap_, 2: cap2_
  bool show_window_;
  int frame_count_;
  std::chrono::steady_clock::time_point t0_;
  cv::Mat display_frame_;
  // Cameras& cam_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PoleNode>();
  if (!node->init()) {
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::spin(node);
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}
