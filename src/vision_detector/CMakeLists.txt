cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(vision_detector)

# ---- 查找基础依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(r2_serial REQUIRED)
find_package(OpenCV REQUIRED)
find_package(onnxruntime REQUIRED)

# ---- 公共头文件 ----
file(GLOB INCLUDE_FILES "include/*.hpp")

# ====================================================================
# ---- weapon & pole 检测节点 ----
# ====================================================================
add_executable(weapon_pole
  ${INCLUDE_FILES}
  src/weapon_pole.cpp)

ament_target_dependencies(weapon_pole PUBLIC rclcpp std_msgs r2_serial)

target_include_directories(weapon_pole PUBLIC
  include/
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(weapon_pole PUBLIC
  ${OpenCV_LIBS}
  onnxruntime::onnxruntime
)

# ====================================================================
# ---- nine-grid-block 检测节点 ----
# ====================================================================
add_executable(nine_grid_block
  ${INCLUDE_FILES}
  src/nine_grid_block.cpp)

ament_target_dependencies(nine_grid_block PUBLIC rclcpp std_msgs r2_serial)

target_include_directories(nine_grid_block PUBLIC
  include/
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(nine_grid_block PUBLIC
  ${OpenCV_LIBS}
  onnxruntime::onnxruntime
)

# ---- 安装 ----
install(TARGETS weapon_pole
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS nine_grid_block
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
