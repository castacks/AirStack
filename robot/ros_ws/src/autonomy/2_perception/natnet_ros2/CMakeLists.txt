cmake_minimum_required(VERSION 3.5)

project(natnet_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_auto_find_build_dependencies()

set(dependencies
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
  std_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MarkerPoses.srv"
  DEPENDENCIES std_msgs
)

execute_process(
  COMMAND chmod "+x" "${CMAKE_CURRENT_SOURCE_DIR}/install_sdk.sh"
  COMMAND chmod "+x" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/helper_node_r2.py"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

execute_process(
  COMMAND bash "-c" "${CMAKE_CURRENT_SOURCE_DIR}/install_sdk.sh"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

set(NATNET_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/NatNetSDK/) 
set(NATNET_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/NatNetSDK/lib/libNatNet.so)

if(NOT EXISTS ${NATNET_SDK_DIR} OR NOT EXISTS ${NATNET_LIBRARY_DIR})
  message(FATAL_ERROR "NatNet SDK not found. Please run the install_sdk.sh script.")
endif()

set(my_include_directories
  include
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${NATNET_SDK_DIR}/include
)

include_directories(
  include
  ${my_include_directories}
)

add_executable(natnet_ros2_node
  src/natnet_ros2.cpp src/nn_filter.cpp
)

ament_target_dependencies(natnet_ros2_node ${dependencies} rosidl_default_runtime)
target_link_libraries(natnet_ros2_node
  ${NATNET_LIBRARY_DIR}
)

#ament_export_dependencies(rosidl_default_runtime)
add_executable(marker_poses_server src/marker_poses_server.cpp)
ament_target_dependencies(marker_poses_server ${dependencies})
rosidl_target_interfaces(
  marker_poses_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(marker_poses_server ${NATNET_LIBRARY_DIR})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(DIRECTORY
  launch
  config 
  ui
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
natnet_ros2_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
natnet_ros2_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  marker_poses_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  marker_poses_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_python_install_package(${PROJECT_NAME}_py)
install(PROGRAMS
  scripts/helper_node_r2.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)

  file(GLOB_RECURSE EXCLUDE_FILES NatNetSDK/*)

  ament_copyright(EXCLUDE ${EXCLUDE_FILES})
  ament_cppcheck(
    EXCLUDE ${EXCLUDE_FILES}
    LANGUAGE c++
  )
  ament_cpplint(EXCLUDE ${EXCLUDE_FILES})
  ament_lint_cmake()
  ament_uncrustify(
    EXCLUDE ${EXCLUDE_FILES}
    LANGUAGE c++
  )

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_auto_package()
#ament_package()
