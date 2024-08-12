cmake_minimum_required(VERSION 3.5)
project(tflib)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(tflib src/tflib.cpp)

target_link_libraries(tflib
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

ament_export_targets(tflibTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp sensor_msgs std_msgs nav_msgs
tf2
tf2_ros
geometry_msgs
)
# install(TARGETS disparity_graph
 #  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 #  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 #  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 #)
 install(
  TARGETS tflib
  EXPORT tflibTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)
#install(DIRECTORY include
##  DESTINATION include/${PROJECT_NAME}
#S)
install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h"
)

ament_package()