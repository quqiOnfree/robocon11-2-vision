cmake_minimum_required(VERSION 3.8)
project(r2_serial)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/SerialPacket.msg
)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

add_executable(r2_data_downlink src/r2_data_downlink.cpp)
target_include_directories(r2_data_downlink PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(r2_data_downlink Threads::Threads)
ament_target_dependencies(r2_data_downlink rclcpp std_msgs nav_msgs tf2)
rosidl_get_typesupport_target(r2_serial_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(r2_data_downlink ${r2_serial_typesupport_target})

install(TARGETS r2_data_downlink
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
