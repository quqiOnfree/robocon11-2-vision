#include "path_planning_node/path_planning.hpp"

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <unordered_set>

inline constexpr std::array<
    std::array<path_planning::map_level, path_planning::map_height>,
    path_planning::map_width>
    level_map{
        std::array<path_planning::map_level, path_planning::map_height>{
            path_planning::map_level::ground, path_planning::map_level::medium,
            path_planning::map_level::high, path_planning::map_level::medium,
            path_planning::map_level::low, path_planning::map_level::ground},
        std::array<path_planning::map_level, path_planning::map_height>{
            path_planning::map_level::ground, path_planning::map_level::low,
            path_planning::map_level::medium, path_planning::map_level::high,
            path_planning::map_level::medium, path_planning::map_level::ground},
        std::array<path_planning::map_level, path_planning::map_height>{
            path_planning::map_level::ground, path_planning::map_level::medium,
            path_planning::map_level::low, path_planning::map_level::medium,
            path_planning::map_level::low, path_planning::map_level::ground}};

inline static std::array<std::array<path_planning::kfs_type, path_planning::map_height>, path_planning::map_width> kfs_map{};

int main () {
    kfs_map[1][2] = path_planning::kfs_type::r2kfs;
    kfs_map[1][3] = path_planning::kfs_type::r2kfs;

    path_planning planning;
    auto [commands, path] = planning.generate_commands(kfs_map, level_map);
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
      case path_planning::command::move_left:
        std::cout << "Move Left\n";
        break;
      case path_planning::command::move_right:
        std::cout << "Move Right\n";
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
      case path_planning::command::turn_around:
        std::cout << "Turn Around\n";
        break;
      default:
        std::cout << "Unknown Command\n";
        break;
      }
      commands.pop();
    }
}
