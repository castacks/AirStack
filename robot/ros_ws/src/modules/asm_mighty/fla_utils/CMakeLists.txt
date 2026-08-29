cmake_minimum_required(VERSION 3.14.4)
project(fla_utils)

# asm_mighty port: trimmed to a headers-only package (param_utils.h,
# process_status.h — all global_mapper_ros consumes). The upstream
# laserscan_to_pointcloud/quat_to_rpy executables and their
# laser_geometry/cv_bridge/OpenCV/joy dependency tail are dropped.

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fla_interfaces REQUIRED)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp fla_interfaces)
ament_package()
