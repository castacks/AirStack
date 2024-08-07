cmake_minimum_required(VERSION 3.5)
project(core_trajectory_msgs)


if(${CMAKE_VERSION} VERSION_GREATER 3.27)
  cmake_policy(SET CMP0148 OLD)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/WaypointXYZVYaw.msg
  msg/TrajectoryXYZVYaw.msg
  msg/FixedTrajectory.msg
  DEPENDENCIES std_msgs geometry_msgs diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_package()
