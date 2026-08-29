cmake_minimum_required(VERSION 3.5)
project(disparity_expansion)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
# cv_bridge
# image_geometry
# image_transport
# roscpp
# std_msgs
# tf
# )
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(stereo_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# CONFIGURE OPENCV
find_package(OpenCV REQUIRED)

# CONFIGURE PCL
find_package(PCL 1.12 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # System dependencies are found with CMake's conventions
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependencies might have been
# #     pulled in transitively but can be declared for certainty nonetheless:
# #     * add a build_depend tag for "message_generation"
# #     * add a run_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# # Generate messages in the 'msg' folder
# add_message_files(
# FILES
# Message1.msg
# Message2.msg
# )

# # Generate services in the 'srv' folder
# add_service_files(
# FILES
# Service1.srv
# Service2.srv
# )

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# # Generate added messages and services with any dependencies listed here
# generate_messages(
# DEPENDENCIES
# std_msgs
# )

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if you package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
# INCLUDE_DIRS
# include
# ${catkin_INCLUDE_DIRS}
# ${OpenCV_INCLUDE_DIRS}
# ${PCL_INCLUDE_DIRS}
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_geometry_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# LIBRARIES disparity_expansion
# CATKIN_DEPENDS cv_bridge image_geometry image_transport roscpp std_msgs tf
# DEPENDS system_lib
# )

# ##########
# # Build ##
# ##########
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
# include_directories(include)
include_directories(

  # ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# # Declare a cpp library
# add_library(disparity_expansion
# src/${PROJECT_NAME}/disparity_expansion.cpp
# )

# # Declare a cpp executable
add_executable(disparity_expansion src/disparity_expansion.cpp)
target_include_directories(disparity_expansion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(disparity_expansion ${OpenCV_LIBS} ${catkin_LIBRARIES}
)

ament_target_dependencies(disparity_expansion
  rclcpp
  rcl_interfaces
  cv_bridge
  image_geometry
  image_transport
  sensor_msgs
  std_msgs
  tf2
  pcl_conversions
  message_filters
  visualization_msgs
  stereo_msgs
  tf2_geometry_msgs
)

add_executable(disparity_pcd src/disparity_pcd.cpp)
target_link_libraries(disparity_pcd ${OpenCV_LIBS} ${catkin_LIBRARIES}
)

ament_target_dependencies(disparity_pcd
  rclcpp
  rcl_interfaces
  cv_bridge
  image_geometry
  image_transport
  sensor_msgs
  std_msgs
  tf2
  pcl_conversions
  message_filters
  tf2_geometry_msgs
)

# install(TARGETS disparity_expansion
# RUNTIME DESTINATION bin)
install(TARGETS disparity_expansion
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS disparity_pcd
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# # Add cmake target dependencies of the executable/library
# # as an example, message headers may need to be generated before nodes
# add_dependencies(disparity_expansion_node disparity_expansion_generate_messages_cpp)

# # Specify libraries to link a library or executable target against
# target_link_libraries(disparity_expansion_node
# ${catkin_LIBRARIES}
# )
ament_package()

# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
# install(PROGRAMS
# scripts/my_python_script
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark executables and/or libraries for installation
# install(TARGETS disparity_expansion disparity_expansion_node
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h"
# PATTERN ".svn" EXCLUDE
# )

# # Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
# # myfile1
# # myfile2
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# ############
# # Testing ##
# ############

# # Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_disparity_expansion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
