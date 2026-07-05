cmake_minimum_required(VERSION 3.10)
project(scancontext_tro)

# ROS 2 ament wrapper for Scan Context T-RO implementation.
# Headers are installed directly into include/ so legacy includes like
# `#include <Scancontext.h>` continue to work during the backend port.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

add_library(scancontext_tro Scancontext.cpp)
target_compile_features(scancontext_tro PUBLIC cxx_std_17)
target_include_directories(scancontext_tro
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(scancontext_tro
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)
ament_target_dependencies(scancontext_tro PUBLIC pcl_conversions)
target_compile_options(scancontext_tro PRIVATE -O3 -Wall)

install(
  FILES
    Scancontext.h
    KDTreeVectorOfVectorsAdaptor.h
    nanoflann.hpp
  DESTINATION include
)
install(
  TARGETS scancontext_tro
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(scancontext_tro)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL pcl_conversions)
ament_package()
