cmake_minimum_required(VERSION 3.8)
project(inspection_geometry_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(map_alignment_node src/map_alignment_node.cpp)
ament_target_dependencies(map_alignment_node rclcpp nav_msgs visualization_msgs std_msgs)

add_executable(feature_snap_node src/feature_snap_node.cpp)
ament_target_dependencies(feature_snap_node rclcpp geometry_msgs)

add_executable(polygon_editor_node src/polygon_editor_node.cpp)
ament_target_dependencies(polygon_editor_node rclcpp geometry_msgs visualization_msgs std_srvs)

add_executable(polygon_offset_node src/polygon_offset_node.cpp)
ament_target_dependencies(polygon_offset_node rclcpp nav_msgs geometry_msgs visualization_msgs)

add_executable(inspection_path_executor_node src/inspection_path_executor_node.cpp)
target_include_directories(inspection_path_executor_node PRIVATE include)
ament_target_dependencies(inspection_path_executor_node rclcpp rclcpp_action nav_msgs nav2_msgs geometry_msgs visualization_msgs std_msgs std_srvs lifecycle_msgs tf2_ros)

add_library(inspection_ordered_route_goal_checker SHARED src/ordered_route_goal_checker.cpp)
target_include_directories(inspection_ordered_route_goal_checker PRIVATE include)
ament_target_dependencies(inspection_ordered_route_goal_checker rclcpp nav_msgs nav2_core nav2_controller tf2_geometry_msgs pluginlib)
pluginlib_export_plugin_description_file(nav2_core ordered_route_goal_checker_plugin.xml)
install(TARGETS inspection_ordered_route_goal_checker LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ordered_path_progress test/test_ordered_path_progress.cpp)
  target_include_directories(test_ordered_path_progress PRIVATE include)
endif()

add_library(inspection_route_panel SHARED src/inspection_route_panel.cpp)
set_target_properties(inspection_route_panel PROPERTIES AUTOMOC ON)
target_link_libraries(inspection_route_panel Qt5::Widgets)
ament_target_dependencies(inspection_route_panel rclcpp std_msgs std_srvs rviz_common pluginlib)
pluginlib_export_plugin_description_file(rviz_common inspection_route_panel_plugin.xml)

install(TARGETS
  map_alignment_node
  feature_snap_node
  polygon_editor_node
  polygon_offset_node
  inspection_path_executor_node
  inspection_route_panel
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md inspection_route_panel_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()
