#pragma once

#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <span>

#include "path_planning_node/path_planning.hpp"

template<typename Node>
inline static void print_command(const Node* node, path_planning::command cmd) {
  switch (cmd) {
  case path_planning::command::move_forward:
    RCLCPP_INFO(node->get_logger(), "Move Forward");
    break;
  case path_planning::command::move_backward:
    RCLCPP_INFO(node->get_logger(), "Move Backward");
    break;
  case path_planning::command::turn_left:
    RCLCPP_INFO(node->get_logger(), "Turn Left");
    break;
  case path_planning::command::turn_right:
    RCLCPP_INFO(node->get_logger(), "Turn Right");
    break;
  case path_planning::command::grab_lower_r2_kfs:
    RCLCPP_INFO(node->get_logger(), "Grab Lower R2 KFS");
    break;
  case path_planning::command::grab_higher_r2_kfs:
    RCLCPP_INFO(node->get_logger(), "Grab Higher R2 KFS");
    break;
  case path_planning::command::grab_highest_r2_kfs:
    RCLCPP_INFO(node->get_logger(), "Grab Highest R2 KFS");
    break;
  case path_planning::command::move_left:
    RCLCPP_INFO(node->get_logger(), "Move Left");
    break;
  case path_planning::command::move_right:
    RCLCPP_INFO(node->get_logger(), "Move Right");
    break;
  case path_planning::command::turn_around:
    RCLCPP_INFO(node->get_logger(), "Turn Around");
    break;
  case path_planning::command::complete_task:
    RCLCPP_INFO(node->get_logger(), "Complete task");
    break;
  default:
    RCLCPP_INFO(node->get_logger(), "Unknown command");
    break;
  }
}

class PathPlanningSenderNReceiver {
public:
  PathPlanningSenderNReceiver(rclcpp::Node* node) : node_(node) {
    path_forward_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/r2_serial/downlink/path/forward", 10);
    path_backward_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/r2_serial/downlink/path/backward", 10);
    path_turn_left_90_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/r2_serial/downlink/path/turn_left_90", 10);
    path_turn_right_90_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/r2_serial/downlink/path/turn_right_90", 10);
    path_shift_left_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/r2_serial/downlink/path/shift_left", 10);
    path_shift_right_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/r2_serial/downlink/path/shift_right", 10);
    path_grab_low_kfs_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/r2_serial/downlink/path/grab_low_kfs", 10);
    path_grab_mid_kfs_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/r2_serial/downlink/path/grab_mid_kfs", 10);
    path_grab_high_kfs_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/r2_serial/downlink/path/grab_high_kfs", 10);
    path_replace_kfs_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/r2_serial/downlink/path/replace_kfs", 10);
    path_no_command_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/r2_serial/downlink/path/no_command", 10);
    path_turn_around_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/r2_serial/downlink/path/turn_around_180", 10);

    path_request_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/r2_serial/uplink/path_request_next", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
          if (command_callback_) {
            command_callback_();
          }
        });
    path_request_new_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
        "/r2_serial/uplink/path_request_next_new", 10,
        [this](const std_msgs::msg::UInt16::SharedPtr msg){
          if (command_with_index_callback_) {
            command_with_index_callback_(msg->data);
          }
        });
  }

  void setCommandCallback(std::function<void()> callback) {
    command_callback_ = std::move(callback);
  }

  void setCommandWithIndexCallback(std::function<void(std::uint16_t)> callback) {
    command_with_index_callback_ = std::move(callback);
  }

  void publish(path_planning::command cmd) {
    std_msgs::msg::Empty msg;
    switch (cmd) {
    case path_planning::command::move_forward:
      path_forward_pub_->publish(msg);
      break;
    case path_planning::command::move_backward:
      path_backward_pub_->publish(msg);
      break;
    case path_planning::command::turn_left:
      path_turn_left_90_pub_->publish(msg);
      break;
    case path_planning::command::turn_right:
      path_turn_right_90_pub_->publish(msg);
      break;
    case path_planning::command::move_left:
      path_shift_left_pub_->publish(msg);
      break;
    case path_planning::command::move_right:
      path_shift_right_pub_->publish(msg);
      break;
    case path_planning::command::grab_lower_r2_kfs:
      path_grab_low_kfs_pub_->publish(msg);
      break;
    case path_planning::command::grab_higher_r2_kfs:
      path_grab_mid_kfs_pub_->publish(msg);
      break;
    case path_planning::command::grab_highest_r2_kfs:
      path_grab_high_kfs_pub_->publish(msg);
      break;
    case path_planning::command::turn_around:
      path_turn_around_pub_->publish(msg);
      break;
    default:
      path_no_command_pub_->publish(msg);
      break;
    }
  }

private:
  rclcpp::Node* node_;
  std::function<void()> command_callback_;
  std::function<void(std::uint16_t)> command_with_index_callback_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_forward_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_backward_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_turn_left_90_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_turn_right_90_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_shift_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_shift_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_grab_low_kfs_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_grab_mid_kfs_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_grab_high_kfs_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_replace_kfs_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_no_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_turn_around_pub_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_request_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr path_request_new_sub_;
};

class PathPlanningNode : public rclcpp::Node {
public:
  PathPlanningNode() : Node("path_planning_node"), sender_receiver_(this) {
    RCLCPP_INFO(this->get_logger(), "Path Planning Node has been started.");

    // set up the command callback to handle commands from the sender_receiver_
    sender_receiver_.setCommandCallback([this]() {
      path_planning::command cmd{path_planning::command::complete_task};
      {
        std::lock_guard<std::mutex> lock(command_array_mutex_);
        if (command_array_.empty()) {
          cmd = path_planning::command::complete_task;
        } else {
          cmd = command_array_[command_array_index_++];
        }
      }
      this->send_command(cmd);
      RCLCPP_INFO(this->get_logger(), "Received a request from command topic");
      print_command(this, cmd);
    });

    sender_receiver_.setCommandWithIndexCallback([this](std::uint16_t index){
      path_planning::command cmd{path_planning::command::complete_task};
      {
        std::lock_guard<std::mutex> lock(command_array_mutex_);
        if (index >= command_array_.size()) {
          cmd = path_planning::command::complete_task;
        } else {
          cmd = command_array_[index];
        }
      }
      this->send_command(cmd);
      RCLCPP_INFO(this->get_logger(), "Received a request with index %d "
        "from command topic",
        static_cast<int>(index));
      print_command(this, cmd);
    });

    path_publisher_ =
        this->create_publisher<std_msgs::msg::String>("path_commands", 10);

    grid_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "grid_data", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "Received grid data: %s",
                      msg->data.c_str());
          try {
            std::shared_ptr<path_planning> planner_ =
                std::make_shared<path_planning>();
            auto grid_json = nlohmann::json::parse(msg->data);
            auto grid = grid_json["grid"].get<std::vector<std::vector<int>>>();
            auto levels =
                grid_json["level"].get<std::vector<std::vector<int>>>();
            std::array<
                std::array<path_planning::kfs_type, path_planning::map_height>,
                path_planning::map_width>
                m_map{};
            for (size_t i = 0; i < grid.size(); ++i) {
              for (size_t j = 0; j < grid[i].size(); ++j) {
                m_map[i][j + 1] = static_cast<path_planning::kfs_type>(
                    grid[grid.size() - 1 - i][j]);
              }
            }
            std::array<
                std::array<path_planning::map_level, path_planning::map_height>,
                path_planning::map_width>
                level_map{};
            for (size_t i = 0; i < levels.size(); ++i) {
              for (size_t j = 0; j < levels[i].size(); ++j) {
                level_map[i][j + 1] = static_cast<path_planning::map_level>(
                    levels[levels.size() - 1 - i][j]);
              }
            }

            auto [commands, path] =
                planner_->generate_commands(m_map, level_map);

            {
              std::lock_guard<std::mutex> lock(command_array_mutex_);
              auto local_commands{commands};
              command_array_.clear();
              command_array_index_ = 0;
              while (!local_commands.empty()) {
                command_array_.push_back(std::move(local_commands.front()));
                local_commands.pop();
              }
            }

            nlohmann::json output_json = nlohmann::json::object();
            output_json["path"] = nlohmann::json::array();
            while (!path.empty()) {
              const auto &node = path.front();
              output_json["path"].push_back(
                  nlohmann::json::array({2 - node.p.x, node.p.y}));
              path.pop();
            }

            std_msgs::msg::String output_msg;
            output_msg.data = output_json.dump();
            RCLCPP_INFO(this->get_logger(), "Publishing path commands: %s",
                        output_msg.data.c_str());
            path_publisher_->publish(output_msg);

            RCLCPP_INFO(this->get_logger(), "Commands generated:");
            while (!commands.empty()) {
              print_command(this, commands.front());
              commands.pop();
            }
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing grid data: %s",
                         e.what());
          }
        });
      }

  void send_command(path_planning::command cmd) {
    sender_receiver_.publish(cmd);
  }

private:
  std::vector<path_planning::command> command_array_;
  std::size_t command_array_index_{0};
  mutable std::mutex command_array_mutex_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> grid_subscriber_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> path_publisher_;
  PathPlanningSenderNReceiver sender_receiver_;
};
