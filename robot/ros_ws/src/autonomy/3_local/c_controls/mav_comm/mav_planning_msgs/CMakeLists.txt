cmake_minimum_required(VERSION 3.14.4...3.27)
project(mav_planning_msgs)

if(${CMAKE_VERSION} VERSION_GREATER 3.27)
  cmake_policy(SET CMP0148 OLD)
endif()

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Point2D.msg
  msg/PointCloudWithPose.msg
  msg/Polygon2D.msg
  msg/PolygonWithHoles.msg
  msg/PolygonWithHolesStamped.msg
  msg/PolynomialSegment.msg
  msg/PolynomialSegment4D.msg
  msg/PolynomialTrajectory.msg
  msg/PolynomialTrajectory4D.msg
  srv/PlannerService.srv
  srv/PolygonService.srv
  srv/ChangeNameService.srv
  DEPENDENCIES builtin_interfaces geometry_msgs mav_msgs sensor_msgs std_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_package()
