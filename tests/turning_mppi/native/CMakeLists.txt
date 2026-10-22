cmake_minimum_required(VERSION 3.16)
project(turning_mppi_native LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(yaml-cpp REQUIRED)
add_library(turning_mppi_native SHARED native_mppi.cpp)
target_compile_features(turning_mppi_native PUBLIC cxx_std_17)
target_compile_options(turning_mppi_native PRIVATE -O3 -Wall -Wextra)
target_compile_definitions(turning_mppi_native PRIVATE
  OFFLINE_FASTDDS_PROFILE="${CMAKE_CURRENT_SOURCE_DIR}/offline_fastdds.xml")
ament_target_dependencies(turning_mppi_native nav2_mppi_controller)
target_link_libraries(turning_mppi_native yaml-cpp)
set_target_properties(turning_mppi_native PROPERTIES BUILD_RPATH /opt/ros/humble/lib)

# Small diagnostic of the installed header's path-distance boundary handling.
# Not part of the baseline controller and never loaded by benchmark.py.
add_executable(path_bounds_repro EXCLUDE_FROM_ALL path_bounds_repro.cpp)
target_compile_features(path_bounds_repro PRIVATE cxx_std_17)
target_compile_options(path_bounds_repro PRIVATE -O1 -g -fsanitize=address -fno-omit-frame-pointer)
target_link_options(path_bounds_repro PRIVATE -fsanitize=address)
ament_target_dependencies(path_bounds_repro nav2_mppi_controller)
