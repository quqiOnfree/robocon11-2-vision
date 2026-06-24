cmake_minimum_required(VERSION 3.10)
project(quatro)

# ROS 2 ament wrapper for Quatro robust registration.
# Quatro depends on TEASER++ and is used by the SAM backend for loop-closure validation.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(QUATRO_TBB "Enable TBB acceleration in Quatro" ON)
option(QUATRO_DEBUG "Enable Quatro debug output" OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(teaserpp REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenMP)

add_library(quatro
  src/fpfh.cc
  src/matcher.cc
  src/quatro_module.cc
)
target_compile_features(quatro PUBLIC cxx_std_17)
target_include_directories(quatro
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(quatro
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    teaserpp::teaser_registration
    teaserpp::teaser_io
    TBB::tbb
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(quatro PUBLIC OpenMP::OpenMP_CXX)
endif()

if(QUATRO_TBB)
  target_compile_definitions(quatro PUBLIC TBB_EN)
endif()
if(QUATRO_DEBUG)
  target_compile_definitions(quatro PUBLIC QUATRO_DEBUG)
endif()

target_compile_options(quatro PRIVATE -O3 -Wall)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS quatro
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(quatro)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)
ament_package()
