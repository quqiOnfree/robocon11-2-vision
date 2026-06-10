#pragma once

#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <span>

#include "path_planning_node/path_planning.hpp"

class PathPlanningNode : public rclcpp::Node {
public:
  PathPlanningNode() : Node("path_planning_node") {
    RCLCPP_INFO(this->get_logger(), "Path Planning Node has been started.");
    path_publisher_ =
        this->create_publisher<std_msgs::msg::String>("path_commands", 10);
    serial_publisher_ =
        this->create_publisher<std_msgs::msg::String>("serial_data", 10);
    serial_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "serial_data", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          std::uint16_t code = 0; // Extract code from msg->data
          std::vector<std::uint8_t> data; // Extract data from msg->data
          process_serial_data(code, data);
        });
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
              std::lock_guard<std::mutex> lock(command_queue_mutex_);
              command_queue_ = commands;
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

            std::cout << "Commands generated:\n";
            while (!commands.empty()) {
              switch (commands.front()) {
              case path_planning::command::move_forward:
                std::cout << "Move Forward\n";
                break;
              case path_planning::command::move_backward:
                std::cout << "Move Backward\n";
                break;
              case path_planning::command::turn_left:
                std::cout << "Turn Left\n";
                break;
              case path_planning::command::turn_right:
                std::cout << "Turn Right\n";
                break;
              case path_planning::command::grab_lower_r2_kfs:
                std::cout << "Grab Lower R2 KFS\n";
                break;
              case path_planning::command::grab_higher_r2_kfs:
                std::cout << "Grab Higher R2 KFS\n";
                break;
              case path_planning::command::grab_highest_r2_kfs:
                std::cout << "Grab Highest R2 KFS\n";
                break;
              case path_planning::command::move_left:
                std::cout << "Move Left\n";
                break;
              case path_planning::command::move_right:
                std::cout << "Move Right\n";
                break;
              default:
                break;
              }
              commands.pop();
            }
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing grid data: %s",
                         e.what());
          }
        });
      }

  void send_packet(std::uint16_t code, std::span<const std::uint8_t> data) {
    std_msgs::msg::String msg;
    RCLCPP_INFO(this->get_logger(), "Sending packet - Code: %u, Data size: %zu bytes",
                code, data.size());
    serial_publisher_->publish(msg);
  }

  void send_command(path_planning::command cmd) {
    send_packet(static_cast<std::uint16_t>(cmd), {});
  }

protected:
  void process_serial_data(std::uint16_t code, std::span<const std::uint8_t> data) {
    RCLCPP_INFO(this->get_logger(), "Processing serial data - Code: %u, Data size: %zu bytes",
                code, data.size());
    switch (static_cast<path_planning::command>(code)) {
    case path_planning::command::request_command: {
        std::lock_guard<std::mutex> lock(command_queue_mutex_);
        if (command_queue_.empty()) {
          send_command(path_planning::command::complete_task);
        } else {
          send_command(command_queue_.front());
          command_queue_.pop();
        }
      }
      break;
    default:
      break;
    }
  }

private:
  std::queue<path_planning::command> command_queue_;
  mutable std::mutex command_queue_mutex_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> grid_subscriber_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> path_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> serial_publisher_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> serial_subscriber_;
};
