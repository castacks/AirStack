cmake_minimum_required(VERSION 3.14.4...3.27)
project(mav_state_machine_msgs)

if(${CMAKE_VERSION} VERSION_GREATER 3.27)
  cmake_policy(SET CMP0148 OLD)
endif()

find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/StartStopTask.msg
  srv/RunTaskService.srv
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
