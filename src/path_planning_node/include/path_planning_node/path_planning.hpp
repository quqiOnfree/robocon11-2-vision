#pragma once

#include <array>
#include <deque>
#include <memory_resource>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>

class path_planning {
public:
  inline static constexpr std::size_t map_width = 3;
  inline static constexpr std::size_t map_height = 6;
  inline static constexpr std::size_t max_r2kfs_can_be_grabed = 3;

  enum class kfs_type { empty = 0, r1kfs, r2kfs, falsekfs };

  struct point {
    int x;
    int y;
  };

  struct path_node {
    point p;
    kfs_type type;
  };

  enum class direction { up, left, down, right };

  enum class command {
    move_forward,
    turn_left,
    move_backward,
    turn_right,
    move_left,
    move_right,
    grab_lower_r2_kfs,
    grab_higher_r2_kfs,
    grab_highest_r2_kfs
  };

  template <typename T,
            typename = std::enable_if_t<std::is_integral_v<std::decay_t<T>>>>
  friend direction operator+(direction c, T n) {
    return static_cast<direction>(((static_cast<int>(c) + n) % 4 + 4) % 4);
  }

  template <typename T,
            typename = std::enable_if_t<std::is_integral_v<std::decay_t<T>>>>
  friend direction operator+(T n, direction c) {
    return static_cast<direction>(((static_cast<int>(c) + n) % 4 + 4) % 4);
  }

  template <typename T,
            typename = std::enable_if_t<std::is_integral_v<std::decay_t<T>>>>
  friend direction operator-(direction c, T n) {
    return static_cast<direction>(((static_cast<int>(c) - n) % 4 + 4) % 4);
  }

  template <typename T,
            typename = std::enable_if_t<std::is_integral_v<std::decay_t<T>>>>
  friend direction operator-(T n, direction c) {
    return static_cast<direction>(((static_cast<int>(c) - n) % 4 + 4) % 4);
  }

  enum class map_level { ground, low, medium, high };

  path_planning() = default;
  ~path_planning() = default;

  std::pair<std::queue<command>, std::queue<path_node>> generate_commands (
    std::array<std::array<kfs_type, map_height>, map_width> m_map,
    const std::array<std::array<map_level, map_height>, map_width> &map) const {
    std::queue<command> result;
    std::size_t r2kfs_count = 0;
    if (m_map[1][1] == kfs_type::r2kfs) {
      result.push(command::grab_higher_r2_kfs);
      m_map[1][1] = kfs_type::empty;
      ++r2kfs_count;
    }
    if (m_map[0][1] == kfs_type::r2kfs) {
      result.push(command::move_left);
      result.push(command::grab_highest_r2_kfs);
      result.push(command::move_right);
      m_map[0][1] = kfs_type::empty;
      ++r2kfs_count;
    }
    if (m_map[2][1] == kfs_type::r2kfs) {
      result.push(command::move_right);
      result.push(command::grab_highest_r2_kfs);
      result.push(command::move_left);
      m_map[2][1] = kfs_type::empty;
      ++r2kfs_count;
    }

    m_map[0][0] = kfs_type::falsekfs;
    m_map[2][0] = kfs_type::falsekfs;
    m_map[1][5] = kfs_type::falsekfs;

    auto path = find_path(m_map);
    auto commands = generate_commands(m_map, path, map, direction::up, r2kfs_count);

    while (!commands.empty()) {
      result.push(std::move(commands.front()));
      commands.pop();
    }
    return {result, path};
  }

protected:
  struct a_star_node {
    point p;
    kfs_type type;
    std::size_t g_cost; // Cost from start to current node
    std::size_t h_cost; // Heuristic cost from current node to end
    std::array<std::array<bool, map_height>, map_width>
        walked{}; // To track walked nodes
    std::size_t f_cost() const { return g_cost + h_cost; } // Total cost
  };

  using a_star_queue_t = std::queue<
      a_star_node,
      std::deque<a_star_node, std::pmr::polymorphic_allocator<a_star_node>>>;

  static std::size_t get_manhattan_distance(const point &a, const point &b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
  }

  struct compare_a_star_node {
    bool operator()(const a_star_queue_t &a, const a_star_queue_t &b) const {
      return a.back().f_cost() > b.back().f_cost();
    }
  };

  std::queue<path_node> find_path(const std::array<std::array<kfs_type, map_height>, map_width> &map) const {
    point start{1, 0};
    point end1{0, 5};
    point end2{2, 5};

    auto path1 = a_star(map, start, end1);
    auto path2 = a_star(map, start, end2);
    a_star_queue_t a_star_result{&pool_resource};
    std::queue<path_node> result;
    if (path1.empty() && path2.empty()) {
      return result; // No path found
    } else if (path1.empty()) {
      a_star_result = std::move(path2);
    } else if (path2.empty()) {
      a_star_result = std::move(path1);
    } else {
      if (path1.back().f_cost() <= path2.back().f_cost()) {
        a_star_result = std::move(path1);
      } else {
        a_star_result = std::move(path2);
      }
    }

    while (!a_star_result.empty()) {
      a_star_node node = std::move(a_star_result.front());
      result.push({node.p, node.type});
      a_star_result.pop();
    }
    return result;
  }

  std::queue<command> generate_commands(
      const std::array<std::array<kfs_type, map_height>, map_width> &m_map,
      std::queue<path_node> ipath,
      const std::array<std::array<map_level, map_height>, map_width> &map,
      direction initial_direction,
      std::size_t initial_grabbed_r2_kfs) const {
    auto original_commands =
        generate_original_commands(m_map, std::move(ipath), map, initial_direction, initial_grabbed_r2_kfs);
    std::queue<command> optimized_commands;
    int turn_count = 0;
    while (!original_commands.empty()) {
      command cmd = original_commands.front();
      original_commands.pop();
      switch (cmd) {
      case command::turn_left:
        turn_count += 1;
        break;
      case command::turn_right:
        turn_count -= 1;
        break;
      default:
        switch ((turn_count % 4 + 4) % 4) {
        case 0:
          break;
        case 1:
          optimized_commands.push(command::turn_left);
          break;
        case 2:
          optimized_commands.push(command::turn_right);
          optimized_commands.push(command::turn_right);
          break;
        case 3:
          optimized_commands.push(command::turn_right);
          break;
        };
        turn_count = 0;
        optimized_commands.push(cmd);
      };
    }
    return optimized_commands;
  }

  a_star_queue_t a_star(const std::array<std::array<kfs_type, map_height>, map_width> &m_map, const point &start, const point &end) const {
    std::priority_queue<a_star_queue_t, std::pmr::vector<a_star_queue_t>,
                        compare_a_star_node>
        path{compare_a_star_node{}, &pool_resource};

    auto get_kfs_type = [&m_map](const point &p) -> kfs_type {
      if (p.x < 0 || p.x >= map_width || p.y < 0 || p.y >= map_height) {
        throw std::out_of_range("Point is out of map bounds");
      }
      return m_map[p.x][p.y];
    };

    {
      // 初始化路径
      a_star_queue_t initial_path{&pool_resource};
      a_star_node start_node{start, get_kfs_type(start), 0,
                             get_manhattan_distance(start, end)};
      start_node.walked[start.x][start.y] = true;
      initial_path.push(std::move(start_node));
      path.push(std::move(initial_path));
    }

    while (!path.empty()) {
      a_star_queue_t current_path{&pool_resource};
      current_path = path.top();
      path.pop();
      const a_star_node current_node = current_path.back();
      if (current_node.p.x == end.x && current_node.p.y == end.y) {
        return current_path;
      }

      auto get_r2kfs_count = [&](const point &p) {
        std::size_t count = 0;
        for (int i = 0; i < 5; ++i) {
          point adjacent_point{p.x, p.y};
          switch (i) {
          case 0:
            adjacent_point.y -= 1;
            break; // Up
          case 1:
            adjacent_point.y += 1;
            break; // Down
          case 2:
            adjacent_point.x -= 1;
            break; // Left
          case 3:
            adjacent_point.x += 1;
            break; // Right
          case 4:
            break; // Current node itself
          default:
            break;
          }
          if (adjacent_point.x >= 0 && adjacent_point.x < map_width &&
              adjacent_point.y >= 0 && adjacent_point.y < map_height &&
              get_kfs_type(adjacent_point) == kfs_type::r2kfs) {
            ++count;
          }
        }
        return count;
      };

      for (int i = 0; i < 4; ++i) {
        auto generate_next_path = [&](point next_point) {
          if (current_node.walked[next_point.x][next_point.y])
            return; // Already walked
          const kfs_type next_type = get_kfs_type(next_point);
          if (next_type == kfs_type::falsekfs)
            return; // Obstacle
          std::size_t r2kfs_count = get_r2kfs_count(next_point);
          a_star_node next_node{
              next_point, next_type,
              current_node.g_cost + 1 + (r2kfs_count > 0 ? 0 : 1) +
                  (next_type == kfs_type::r1kfs ? 1 : 0) + [&]() -> int {
                std::size_t r2kfs_count = 0;
                for (int i = 0; i < map_width; ++i) {
                  for (int j = 0; j < map_height; ++j) {
                    if (current_node.walked[i][j] &&
                        get_kfs_type({i, j}) == kfs_type::r2kfs) {
                      ++r2kfs_count;
                      if (r2kfs_count > 3) {
                        return 1; // If there's at least one r2kfs in the path,
                                  // no extra cost for r1kfs
                      }
                    }
                  }
                }
                return 0;
              }(),
              get_manhattan_distance(next_point, end), current_node.walked};
          next_node.walked[next_point.x][next_point.y] = true;
          a_star_queue_t new_path{&pool_resource};
          new_path = current_path;
          new_path.push(next_node);
          path.push(std::move(new_path));
        };

        switch (i) {
        case 0: // Up
        {
          point next_point{current_node.p.x, current_node.p.y - 1};
          if (next_point.y < 0)
            continue; // Out of bounds
          generate_next_path(next_point);
        } break;
        case 1: // Down
        {
          point next_point{current_node.p.x, current_node.p.y + 1};
          if (next_point.y >= map_height)
            continue; // Out of bounds
          generate_next_path(next_point);
        } break;
        case 2: // Left
        {
          point next_point{current_node.p.x - 1, current_node.p.y};
          if (next_point.x < 0)
            continue; // Out of bounds
          generate_next_path(next_point);
        } break;
        case 3: // Right
        {
          point next_point{current_node.p.x + 1, current_node.p.y};
          if (next_point.x >= map_width)
            continue; // Out of bounds
          generate_next_path(next_point);
        } break;
        }
      }
    }
    return {};
  }

  // template<typename T, typename =
  // std::enable_if_t<std::is_same_v<std::decay_t<T>, std::queue<path_node>>>>
  std::queue<command> generate_original_commands(
      const std::array<std::array<kfs_type, map_height>, map_width>& m_map,
      std::queue<path_node> ipath,
      std::array<std::array<map_level, map_height>, map_width> map,
      direction initial_direction,
      std::size_t initial_grabbed_r2_kfs) const {
    // initilize map
    map[0][0] = map_level::ground;
    map[1][0] = map_level::ground;
    map[2][0] = map_level::ground;
    map[0][map_height - 1] = map_level::ground;
    map[1][map_height - 1] = map_level::ground;
    map[2][map_height - 1] = map_level::ground;

    std::queue<command> commands;
    std::queue<path_node> path{std::forward<std::queue<path_node>>(ipath)};
    if (path.empty()) {
      return commands; // No path, no commands
    }
    const path_node initial_node = std::move(path.front());
    path.pop();
    path_node current_node = initial_node;

    struct point_with_direction {
      point p;
      direction d;
    };

    struct point_compare {
      bool operator()(const point &a, const point &b) const {
        return std::tie(a.x, a.y) < std::tie(b.x, b.y);
      }
    };

    std::queue<point_with_direction,
               std::deque<point_with_direction, std::pmr::polymorphic_allocator<
                                                    point_with_direction>>>
        directions{&pool_resource};
    std::set<point, point_compare, std::pmr::polymorphic_allocator<point>>
        must_be_walked_points{&pool_resource};
    std::size_t r2kfs_must_be_grabed = 0;
    auto local_map = m_map;

    auto get_kfs_type = [&m_map](const point &p) -> kfs_type {
      if (p.x < 0 || p.x >= map_width || p.y < 0 || p.y >= map_height) {
        throw std::out_of_range("Point is out of map bounds");
      }
      return m_map[p.x][p.y];
    };

    while (!path.empty()) {
      auto node = std::move(path.front());
      path.pop();
      point next_point = node.p;
      must_be_walked_points.insert(next_point);

      if (get_kfs_type(next_point) == kfs_type::r2kfs) {
        ++r2kfs_must_be_grabed;
      }
      if (next_point.x == current_node.p.x) {
        if (next_point.y == current_node.p.y + 1) {
          directions.push({next_point, direction::up});
        } else if (next_point.y == current_node.p.y - 1) {
          directions.push({next_point, direction::down});
        }
      } else if (next_point.y == current_node.p.y) {
        if (next_point.x == current_node.p.x + 1) {
          directions.push({next_point, direction::right});
        } else if (next_point.x == current_node.p.x - 1) {
          directions.push({next_point, direction::left});
        }
      }
      current_node = std::move(node);
    }

    point_with_direction current = {initial_node.p, initial_direction};
    local_map = m_map;
    std::size_t ext_r2kfs_count = initial_grabbed_r2_kfs;

    while (!directions.empty()) {
      point_with_direction next = directions.front();
      point current_point = current.p;
      point next_point = next.p;
      direction current_direction = current.d;
      direction next_direction = next.d;
      directions.pop();
      // move forward to level up, move backward to level down
      const map_level current_level = map[current_point.x][current_point.y];
      const map_level next_level = map[next_point.x][next_point.y];

      auto get_dir_pos = [&](direction dire) -> std::optional<point> {
        point adjacent_point = current_point;
        switch (current_direction) {
        case direction::up:
          switch (dire) {
          case direction::up:
            adjacent_point.y += 1;
            break;
          case direction::down:
            adjacent_point.y -= 1;
            break;
          case direction::left:
            adjacent_point.x -= 1;
            break;
          case direction::right:
            adjacent_point.x += 1;
            break;
          };
          break;
        case direction::down:
          switch (dire) {
          case direction::up:
            adjacent_point.y -= 1;
            break;
          case direction::down:
            adjacent_point.y += 1;
            break;
          case direction::left:
            adjacent_point.x += 1;
            break;
          case direction::right:
            adjacent_point.x -= 1;
            break;
          };
          break;
        case direction::left:
          switch (dire) {
          case direction::up:
            adjacent_point.x -= 1;
            break;
          case direction::down:
            adjacent_point.x += 1;
            break;
          case direction::left:
            adjacent_point.y -= 1;
            break;
          case direction::right:
            adjacent_point.y += 1;
            break;
          };
          break;
        case direction::right:
          switch (dire) {
          case direction::up:
            adjacent_point.x += 1;
            break;
          case direction::down:
            adjacent_point.x -= 1;
            break;
          case direction::left:
            adjacent_point.y += 1;
            break;
          case direction::right:
            adjacent_point.y -= 1;
            break;
          };
          break;
        }
        if (adjacent_point.x < 0 || adjacent_point.x >= map_width ||
            adjacent_point.y < 0 || adjacent_point.y >= map_height) {
          return std::nullopt; // Out of bounds
        }
        return adjacent_point;
      };

      auto get_kfs = [&](point p) {
        point adjacent_point = p;
        if (adjacent_point.x < 0 || adjacent_point.x >= map_width ||
            adjacent_point.y < 0 || adjacent_point.y >= map_height) {
          return kfs_type::empty;
        }
        return local_map[p.x][p.y];
      };

      std::optional<point> up = get_dir_pos(direction::up);
      std::optional<point> left = get_dir_pos(direction::left);
      std::optional<point> down = get_dir_pos(direction::down);
      std::optional<point> right = get_dir_pos(direction::right);

      auto gen_grab_command = [&]() {
        if (current_level == map_level::ground) {
          commands.push(command::grab_higher_r2_kfs);
        } else if (current_level == map_level::low) {
          if (next_level == map_level::medium ||
              next_level == map_level::high) {
            commands.push(command::grab_higher_r2_kfs);
          } else {
            commands.push(command::grab_lower_r2_kfs);
          }
        } else if (current_level == map_level::medium) {
          if (next_level == map_level::high) {
            commands.push(command::grab_higher_r2_kfs);
          } else {
            commands.push(command::grab_lower_r2_kfs);
          }
        } else if (current_level == map_level::high) {
          commands.push(command::grab_lower_r2_kfs);
        }
      };

      if (up.has_value() &&
          (must_be_walked_points.find(up.value()) !=
               must_be_walked_points.end() ||
           (ext_r2kfs_count + r2kfs_must_be_grabed <
            max_r2kfs_can_be_grabed)) &&
          get_kfs(up.value()) == kfs_type::r2kfs) {
        gen_grab_command();
        local_map[up.value().x][up.value().y] = kfs_type::empty;
        if (must_be_walked_points.find(up.value()) ==
            must_be_walked_points.end()) {
          ++ext_r2kfs_count;
        }
      }
      if (left.has_value() &&
          (must_be_walked_points.find(left.value()) !=
               must_be_walked_points.end() ||
           (ext_r2kfs_count + r2kfs_must_be_grabed <
            max_r2kfs_can_be_grabed)) &&
          get_kfs(left.value()) == kfs_type::r2kfs) {
        commands.push(command::turn_left);
        gen_grab_command();
        commands.push(command::turn_right);
        local_map[left.value().x][left.value().y] = kfs_type::empty;
        if (must_be_walked_points.find(left.value()) ==
            must_be_walked_points.end()) {
          ++ext_r2kfs_count;
        }
      }
      if (right.has_value() &&
          (must_be_walked_points.find(right.value()) !=
               must_be_walked_points.end() ||
           (ext_r2kfs_count + r2kfs_must_be_grabed <
            max_r2kfs_can_be_grabed)) &&
          get_kfs(right.value()) == kfs_type::r2kfs) {
        commands.push(command::turn_right);
        gen_grab_command();
        commands.push(command::turn_left);
        local_map[right.value().x][right.value().y] = kfs_type::empty;
        if (must_be_walked_points.find(right.value()) ==
            must_be_walked_points.end()) {
          ++ext_r2kfs_count;
        }
      }
      if (down.has_value() &&
          (must_be_walked_points.find(down.value()) !=
               must_be_walked_points.end() ||
           (ext_r2kfs_count + r2kfs_must_be_grabed <
            max_r2kfs_can_be_grabed)) &&
          get_kfs(down.value()) == kfs_type::r2kfs) {
        commands.push(command::turn_right);
        commands.push(command::turn_right);
        gen_grab_command();
        commands.push(command::turn_right);
        commands.push(command::turn_right);
        local_map[down.value().x][down.value().y] = kfs_type::empty;
        if (must_be_walked_points.find(down.value()) ==
            must_be_walked_points.end()) {
          ++ext_r2kfs_count;
        }
      }

      if (current_direction == next_direction) {
        if (current_level == map_level::ground) {
          commands.push(command::move_forward);
          current = {next_point, current_direction};
        } else if (current_level == map_level::low) {
          if (next_level == map_level::medium ||
              next_level == map_level::high) {
            commands.push(command::move_forward);
            current = {next_point, current_direction};
          } else {
            commands.push(command::turn_right);
            commands.push(command::turn_right);
            commands.push(command::move_backward);
            current = {next_point, (current_direction - 2)};
          }
        } else if (current_level == map_level::medium) {
          if (next_level == map_level::high) {
            commands.push(command::move_forward);
            current = {next_point, current_direction};
          } else {
            commands.push(command::turn_right);
            commands.push(command::turn_right);
            commands.push(command::move_backward);
            current = {next_point, (current_direction - 2)};
          }
        } else if (current_level == map_level::high) {
          commands.push(command::turn_right);
          commands.push(command::turn_right);
          commands.push(command::move_backward);
          current = {next_point, (current_direction - 2)};
        }
      } else if (current_direction - 1 == next_direction) {
        if (current_level == map_level::ground) {
          commands.push(command::turn_right);
          commands.push(command::move_forward);
          current = {next_point, (current_direction - 1)};
        } else if (current_level == map_level::low) {
          if (next_level == map_level::medium ||
              next_level == map_level::high) {
            commands.push(command::turn_right);
            commands.push(command::move_forward);
            current = {next_point, (current_direction - 1)};
          } else {
            commands.push(command::turn_left);
            commands.push(command::move_backward);
            current = {next_point, (current_direction + 1)};
          }
        } else if (current_level == map_level::medium) {
          if (next_level == map_level::high) {
            commands.push(command::turn_right);
            commands.push(command::move_forward);
            current = {next_point, (current_direction - 1)};
          } else {
            commands.push(command::turn_left);
            commands.push(command::move_backward);
            current = {next_point, (current_direction + 1)};
          }
        } else if (current_level == map_level::high) {
          commands.push(command::turn_left);
          commands.push(command::move_backward);
          current = {next_point, (current_direction + 1)};
        }
      } else if (current_direction + 1 == next_direction) {
        if (current_level == map_level::ground) {
          commands.push(command::turn_left);
          commands.push(command::move_forward);
          current = {next_point, (current_direction + 1)};
        } else if (current_level == map_level::low) {
          if (next_level == map_level::medium ||
              next_level == map_level::high) {
            commands.push(command::turn_left);
            commands.push(command::move_forward);
            current = {next_point, (current_direction + 1)};
          } else {
            commands.push(command::turn_right);
            commands.push(command::move_backward);
            current = {next_point, (current_direction - 1)};
          }
        } else if (current_level == map_level::medium) {
          if (next_level == map_level::high) {
            commands.push(command::turn_left);
            commands.push(command::move_forward);
            current = {next_point, (current_direction + 1)};
          } else {
            commands.push(command::turn_right);
            commands.push(command::move_backward);
            current = {next_point, (current_direction - 1)};
          }
        } else if (current_level == map_level::high) {
          commands.push(command::turn_right);
          commands.push(command::move_backward);
          current = {next_point, (current_direction - 1)};
        }
      } else {
        // 180 degree turn
        if (current_level == map_level::ground) {
          commands.push(command::turn_right);
          commands.push(command::turn_right);
          commands.push(command::move_forward);
          current = {next_point, (current_direction - 2)};
        } else if (current_level == map_level::low) {
          if (next_level == map_level::medium ||
              next_level == map_level::high) {
            commands.push(command::turn_right);
            commands.push(command::turn_right);
            commands.push(command::move_forward);
            current = {next_point, (current_direction - 2)};
          } else {
            commands.push(command::move_backward);
            current = {next_point, current_direction};
          }
        } else if (current_level == map_level::medium) {
          if (next_level == map_level::high) {
            commands.push(command::turn_right);
            commands.push(command::turn_right);
            commands.push(command::move_forward);
            current = {next_point, (current_direction - 2)};
          } else {
            commands.push(command::move_backward);
            current = {next_point, current_direction};
          }
        } else if (current_level == map_level::high) {
          commands.push(command::move_backward);
          current = {next_point, current_direction};
        }
      }
    }

    return commands;
  }

private:
  // std::array<std::array<kfs_type, map_height>, map_width> m_map{};
  inline static std::pmr::synchronized_pool_resource pool_resource{};
};
