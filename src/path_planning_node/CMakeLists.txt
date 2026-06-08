cmake_minimum_required(VERSION 3.8)
project(path_planning_node)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_subdirectory(json)

add_executable(path_planning_node
  src/path_planning_node.cpp)
target_include_directories(path_planning_node PRIVATE
  ${std_msgs_INCLUDE_DIRS}
  include)
target_link_libraries(path_planning_node PRIVATE
  rclcpp::rclcpp
  ${std_msgs_LIBRARIES}
  nlohmann_json)

# add_executable(path_planning_test
#   src/path_planning_test.cpp)
# target_include_directories(path_planning_test PRIVATE
#   ${std_msgs_INCLUDE_DIRS}
#   include)
# target_link_libraries(path_planning_test PRIVATE
#   rclcpp::rclcpp
#   ${std_msgs_LIBRARIES}
#   nlohmann_json)


install(TARGETS path_planning_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
