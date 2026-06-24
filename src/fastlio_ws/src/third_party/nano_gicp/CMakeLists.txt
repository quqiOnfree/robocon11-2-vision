cmake_minimum_required(VERSION 3.10)
project(nano_gicp)

# ROS 2 ament wrapper for the original Nano-GICP library used by FAST-LIO-SAM-SC-QN.
# Keep target names `nanoflann` and `nano_gicp` so downstream code can link them directly.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

add_library(nanoflann src/nanoflann.cc)
target_compile_features(nanoflann PUBLIC cxx_std_17)
target_include_directories(nanoflann
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(nanoflann PUBLIC ${PCL_LIBRARIES})
target_compile_options(nanoflann PRIVATE -O3 -Wall -Wextra)

add_library(nano_gicp src/nano_gicp.cc)
target_compile_features(nano_gicp PUBLIC cxx_std_17)
target_include_directories(nano_gicp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(nano_gicp
  PUBLIC
    nanoflann
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(nano_gicp PUBLIC OpenMP::OpenMP_CXX)
endif()
target_compile_options(nano_gicp PRIVATE -O3 -Wall -Wextra)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS nanoflann nano_gicp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(nanoflann nano_gicp)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)
ament_package()
