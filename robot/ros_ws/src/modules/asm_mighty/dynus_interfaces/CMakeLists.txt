cmake_minimum_required(VERSION 3.8)
project(dynus_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QuinticPWPTraj.msg"
  "msg/PWPTraj.msg"
  "msg/QuinticCoeffPoly3.msg"
  "msg/CoeffPoly3.msg"
  "msg/DynTraj.msg"
  "msg/DynTrajArray.msg"
  "msg/State.msg"
  "msg/Goal.msg"
  "msg/Trajectory.msg"
  "msg/Mode.msg"
  "msg/YawOutput.msg"
  "msg/PNAdaptation.msg"
  "msg/ComputationTimes.msg"
  "msg/SpeedyPath.msg"
  "msg/Frontier.msg"
  "msg/FrontierList.msg"
  "srv/FlightCommand.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()