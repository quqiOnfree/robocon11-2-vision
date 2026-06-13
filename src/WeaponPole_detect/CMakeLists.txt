cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(robot_vision)

# ---- 查找基础依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(onnxruntime REQUIRED)
find_package(fast_lio REQUIRED)
find_package(OpenCV REQUIRED)

# ---- 公共头文件 ----
file(GLOB INCLUDE_FILES "include/*.hpp")

add_library(asio INTERFACE)
target_include_directories(asio INTERFACE
  asio/include)

# ====================================================================
# ---- weapon 检测节点 ----
# ====================================================================
add_executable(weapon_node
  ${INCLUDE_FILES}
  src/weapon_node.cpp)

ament_target_dependencies(weapon_node PUBLIC rclcpp std_msgs fast_lio)

target_include_directories(weapon_node PUBLIC
  include/
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(weapon_node PUBLIC
  ${OpenCV_LIBS}
  onnxruntime::onnxruntime
  $<$<BOOL:${WIN32}>:ws2_32>
)

target_compile_definitions(weapon_node PUBLIC
  $<$<BOOL:${WIN32}>:_WIN32_WINNT=0x0A00>
)

# ====================================================================
# ---- pole 检测节点 ----
# ====================================================================
add_executable(pole_node
  ${INCLUDE_FILES}
  src/pole_node.cpp)

ament_target_dependencies(pole_node PUBLIC rclcpp std_msgs fast_lio)

target_include_directories(pole_node PUBLIC
  include/
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(pole_node PUBLIC
  ${OpenCV_LIBS}
  asio
  onnxruntime::onnxruntime
  $<$<BOOL:${WIN32}>:ws2_32>
)

target_compile_definitions(pole_node PUBLIC
  $<$<BOOL:${WIN32}>:_WIN32_WINNT=0x0A00>
)

# ---- 安装 ----
install(TARGETS weapon_node pole_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY
  model/
  DESTINATION share/${PROJECT_NAME}/model)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
