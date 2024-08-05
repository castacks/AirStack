cmake_minimum_required(VERSION 2.8.3)
project(core_trajectory_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  diagnostic_msgs
)

add_message_files(
  FILES
  WaypointXYZVYaw.msg
  TrajectoryXYZVYaw.msg
  FixedTrajectory.msg
  )

generate_messages(DEPENDENCIES std_msgs geometry_msgs diagnostic_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs diagnostic_msgs)
