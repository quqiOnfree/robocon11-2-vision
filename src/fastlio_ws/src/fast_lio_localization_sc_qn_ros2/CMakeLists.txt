cmake_minimum_required(VERSION 3.10)
project(fast_lio_localization_sc_qn_ros2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration features kdtree search)
find_package(OpenMP REQUIRED)
find_package(TBB REQUIRED)
find_package(teaserpp REQUIRED)
find_package(scancontext_tro REQUIRED)
find_package(nano_gicp REQUIRED)
find_package(quatro REQUIRED)

get_filename_component(FASTLIO_WS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(THIRD_PARTY_LIBRARY_HINTS "")
foreach(prefix IN LISTS CMAKE_PREFIX_PATH)
  list(APPEND THIRD_PARTY_LIBRARY_HINTS "${prefix}/lib")
endforeach()
find_library(SCANCONTEXT_TRO_LIBRARY NAMES scancontext_tro HINTS ${THIRD_PARTY_LIBRARY_HINTS} REQUIRED)
find_library(NANOFLANN_LIBRARY NAMES nanoflann HINTS ${THIRD_PARTY_LIBRARY_HINTS} REQUIRED)
find_library(NANO_GICP_LIBRARY NAMES nano_gicp HINTS ${THIRD_PARTY_LIBRARY_HINTS} REQUIRED)
find_library(QUATRO_LIBRARY NAMES quatro HINTS ${THIRD_PARTY_LIBRARY_HINTS} REQUIRED)
find_library(LZ4_LIBRARY NAMES lz4 REQUIRED)

add_library(${PROJECT_NAME}_core
  src/map_matcher.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${FASTLIO_WS_SRC_DIR}/third_party/scancontext_tro
    ${FASTLIO_WS_SRC_DIR}/third_party/nano_gicp/include
    ${FASTLIO_WS_SRC_DIR}/third_party/Quatro/include
)
ament_target_dependencies(${PROJECT_NAME}_core
  PUBLIC
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    pcl_conversions
)
target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${SCANCONTEXT_TRO_LIBRARY}
    ${NANOFLANN_LIBRARY}
    ${NANO_GICP_LIBRARY}
    ${QUATRO_LIBRARY}
    OpenMP::OpenMP_CXX
    TBB::tbb
    teaserpp::teaser_registration
    teaserpp::teaser_io
    ${LZ4_LIBRARY}
)
target_compile_options(${PROJECT_NAME}_core PRIVATE -O3 -Wall -Wextra)

add_executable(fast_lio_localization_sc_qn_node
  src/localization_node.cpp
)
target_compile_features(fast_lio_localization_sc_qn_node PUBLIC cxx_std_17)
target_include_directories(fast_lio_localization_sc_qn_node
  PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
    ${FASTLIO_WS_SRC_DIR}/third_party/scancontext_tro
    ${FASTLIO_WS_SRC_DIR}/third_party/nano_gicp/include
    ${FASTLIO_WS_SRC_DIR}/third_party/Quatro/include
)
ament_target_dependencies(fast_lio_localization_sc_qn_node
  PUBLIC
    rclcpp
    message_filters
    nav_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
    visualization_msgs
    pcl_conversions
)
target_link_libraries(fast_lio_localization_sc_qn_node
  PRIVATE
    ${PROJECT_NAME}_core
    Eigen3::Eigen
    ${SCANCONTEXT_TRO_LIBRARY}
    ${NANOFLANN_LIBRARY}
    ${NANO_GICP_LIBRARY}
    ${QUATRO_LIBRARY}
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    TBB::tbb
    teaserpp::teaser_registration
    teaserpp::teaser_io
    ${LZ4_LIBRARY}
)
target_compile_options(fast_lio_localization_sc_qn_node PRIVATE -O3 -Wall -Wextra)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}_core fast_lio_localization_sc_qn_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  Eigen3
  PCL
  scancontext_tro
  nano_gicp
  quatro
  teaserpp
)
ament_package()
