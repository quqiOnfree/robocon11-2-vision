#pragma once

#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "path_planning_node/path_planning.hpp"

class PathPlanningNode : public rclcpp::Node {
public:
  PathPlanningNode() : Node("path_planning_node") {
    RCLCPP_INFO(this->get_logger(), "Path Planning Node has been started.");
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
              }
              commands.pop();
            }
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing grid data: %s",
                         e.what());
          }
        });
  }

private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> grid_subscriber_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> path_publisher_;
};
