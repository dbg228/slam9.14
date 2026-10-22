cmake_minimum_required(VERSION 3.8)
project(static_map_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NdtMatchDiagnostic.msg"
  "msg/NdtMatchDiagnosticDetail.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs)

add_executable(static_map_localizer_node src/static_map_localizer_node.cpp)
target_include_directories(static_map_localizer_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(static_map_localizer_node ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(static_map_localizer_node
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2_msgs tf2_ros pcl_conversions)
rosidl_target_interfaces(static_map_localizer_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS static_map_localizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/map_to_odom_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
