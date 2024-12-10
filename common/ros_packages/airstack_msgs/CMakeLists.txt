cmake_minimum_required(VERSION 3.8)
project(airstack_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Odometry.msg"
  "msg/FixedTrajectory.msg"
  "msg/TrajectoryXYZVYaw.msg"
  "msg/WaypointXYZVYaw.msg"
  "msg/KeepOutZone.msg"
  "msg/PlanRequest.msg"
  "msg/SearchMissionRequest.msg"
  "msg/BeliefMapData.msg"
  "msg/SearchPrior.msg"
  "msg/TaskAssignment.msg"
  "msg/query/TextQueryResponse.msg"
  "srv/RobotCommand.srv"
  "srv/TrajectoryMode.srv"
  "srv/TakeoffLandingCommand.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs diagnostic_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
