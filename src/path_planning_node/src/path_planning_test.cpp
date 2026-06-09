#include "path_planning_node/path_planning.hpp"

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <unordered_set>

path_planning planner;
std::size_t test_count = 0;
std::ofstream fp("path_planning_test_output.txt");

struct hash {
  std::size_t
  operator()(const std::array<
             std::array<path_planning::kfs_type, path_planning::map_height>,
             path_planning::map_width> &map) const {
    std::size_t result = 0;
    for (std::size_t i = 0; i < path_planning::map_width; ++i) {
      for (std::size_t j = 0; j < path_planning::map_height; ++j) {
        switch (map[i][j]) {
        case path_planning::kfs_type::empty:
          /* code */
          break;

        case path_planning::kfs_type::r1kfs:
          result += 100 * (i * path_planning::map_height + j);
          break;

        case path_planning::kfs_type::r2kfs:
          result += 10000 * (i * path_planning::map_height + j);
          break;

        case path_planning::kfs_type::falsekfs:
          result += 1000000 * (i * path_planning::map_height + j);
          break;

        default:
          break;
        }
      }
    }
    return result;
  }
};

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

int main() {
  auto start_time = std::chrono::steady_clock::now();
  std::streambuf *cout_buf = std::cout.rdbuf();
  std::cout.rdbuf(
      fp ? fp.rdbuf()
         : cout_buf); // Redirect std::cout to file if opened successfully

  // 设置障碍物和特殊节点
  std::function<void(int)> setup_map;
  std::vector<path_planning::kfs_type> types = {
      path_planning::kfs_type::r1kfs, path_planning::kfs_type::r1kfs,
      path_planning::kfs_type::r1kfs, path_planning::kfs_type::r2kfs,
      path_planning::kfs_type::r2kfs, path_planning::kfs_type::r2kfs,
      path_planning::kfs_type::r2kfs};
  std::unordered_set<
      std::array<std::array<path_planning::kfs_type, path_planning::map_height>,
                 path_planning::map_width>,
      hash>
      test_maps;
  std::array<std::array<path_planning::kfs_type, path_planning::map_height>,
             path_planning::map_width>
      empty_map{};
  empty_map[0][0] =
      path_planning::kfs_type::falsekfs; // Set an obstacle at (0, 0)
  empty_map[2][0] =
      path_planning::kfs_type::falsekfs; // Set an obstacle at (2, 0)
  empty_map[1][5] =
      path_planning::kfs_type::falsekfs; // Set an obstacle at (1, 5)
  auto func = [&](int idx) {
    if (idx == -1) {
      for (int i = 0; i < path_planning::map_width; ++i) {
        for (int j = 2; j < path_planning::map_height - 1; ++j) {
          if (empty_map[i][j] != path_planning::kfs_type::empty) {
            continue; // Skip already set nodes
          }
          empty_map[i][j] = path_planning::kfs_type::falsekfs;
          setup_map(idx + 1);
          empty_map[i][j] = path_planning::kfs_type::empty;
        }
      }
    }
    if (idx >= types.size()) {
      if (test_maps.find(empty_map) == test_maps.end()) {
        test_maps.insert(empty_map);
      }
      return;
    }
    for (int i = 0; i < path_planning::map_width; ++i) {
      for (int j = 1; j < path_planning::map_height - 1; ++j) {
        if (empty_map[i][j] != path_planning::kfs_type::empty) {
          continue; // Skip already set nodes
        }
        if (i == 1 && (j >= 2 && j <= 3) &&
            types[idx] == path_planning::kfs_type::r1kfs) {
          continue; // Skip setting r1kfs on the path
        }
        empty_map[i][j] = types[idx];
        setup_map(idx + 1);
        empty_map[i][j] = path_planning::kfs_type::empty;
      }
    }
  };
  setup_map = func;

  std::cout << "Show level map:\n";
  std::cout << "Height: \n";
  std::cout << "Ground: " << static_cast<int>(path_planning::map_level::ground)
            << "\n";
  std::cout << "Low: " << static_cast<int>(path_planning::map_level::low)
            << "\n";
  std::cout << "Medium: " << static_cast<int>(path_planning::map_level::medium)
            << "\n";
  std::cout << "High: " << static_cast<int>(path_planning::map_level::high)
            << "\n";
  for (std::size_t j = 0; j < path_planning::map_height; ++j) {
    for (std::size_t i = 0; i < path_planning::map_width; ++i) {
      std::cout << static_cast<std::size_t>(level_map[i][j]) << " ";
    }
    std::cout << "\n";
  }

  std::cout << "Generating test maps...\n";
  func(-1); // Start generating test maps with -1 to indicate the initial call

  std::cout << "Test maps generated: " << test_maps.size() << "\n";
  std::cout << "Running tests...\n";

  // path_planning::point start{1, 0};
  // path_planning::point end1{0, 5};
  // path_planning::point end2{2, 5};

  for (const auto &map : test_maps) {
    for (int i = 0; i < path_planning::map_width; ++i) {
      for (int j = 0; j < path_planning::map_height; ++j) {
        planner.set_kfs_type({i, j}, map[i][j]);
      }
    }

    std::cout << "Test case " << ++test_count << ":\n";

    auto path = planner.find_path();
    auto commands = planner.generate_commands(path, level_map,
                                              path_planning::direction::up);

    // Remove the obstacle for visualization
    planner.set_kfs_type({1, 5}, path_planning::kfs_type::empty);
    planner.set_kfs_type({0, 0}, path_planning::kfs_type::empty);
    planner.set_kfs_type({2, 0}, path_planning::kfs_type::empty);

    std::cout << "Grid:\n";
    std::cout << "KFS Type:\n";
    std::cout << "Empty: " << static_cast<int>(path_planning::kfs_type::empty)
              << '\n';
    std::cout << "R1KFS: " << static_cast<int>(path_planning::kfs_type::r1kfs)
              << '\n';
    std::cout << "R2KFS: " << static_cast<int>(path_planning::kfs_type::r2kfs)
              << '\n';
    std::cout << "False KFS: "
              << static_cast<int>(path_planning::kfs_type::falsekfs) << '\n';
    for (int j = 0; j < path_planning::map_height; ++j) {
      for (int i = 0; i < path_planning::map_width; ++i) {
        std::cout << static_cast<int>(planner.get_kfs_type({i, j})) << " ";
      }
      std::cout << "\n";
    }

    std::cout << "Path found:\n";
    while (!path.empty()) {
      const auto &node = path.front();
      std::cout << "(" << node.p.x << ", " << node.p.y
                << ") - Type: " << static_cast<int>(node.type) << "\n";
      path.pop();
    }

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
      }
      commands.pop();
    }
  }

  auto end_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time;
  std::cout << "Total tests run: " << test_count << "\n";
  std::cout << "Total time taken: " << elapsed_seconds.count() << " seconds\n";

  std::cout.rdbuf(cout_buf); // Restore original std::cout buffer
  return 0;
}
