cmake_minimum_required(VERSION 3.8)
project(disparity_graph)

#find_package(catkin REQUIRED COMPONENTS
#  cmake_modules
#  ca_common
#  cv_bridge -
#  geometry_msgs
#  image_geometry -
#  image_transport -
#  nav_msgs -
#  roscpp -
#  rospy 
#  sensor_msgs -
#  std_msgs -
#  tf -
#  tf_conversions -
#  tictoc_profiler -
#  pcl_conversions -
#  pcl_ros
#)

find_package(ament_cmake REQUIRED)
find_package(disparity_graph REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# CONFIGURE OPENCV
find_package(OpenCV REQUIRED)

# CONFIGURE PCL
find_package(PCL REQUIRED)
find_package(tf2_eigen REQUIRED)

#CONFIGURER ARMADILLO
find_package(Armadillo REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#find_package(PkgConfig REQUIRED)
#find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES disparity_graph
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf image_transport image_geometry
#  DEPENDS eigen Boost PCL
#)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#DEBUGGGGGGGGGGGEEEEEEEEEEERRRRRRRRRRR
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -std=gnu++0x")

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}

  ${image_transport_INCLUDE_DIRS}
  ${image_geometry_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS} 
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${disparity_graph_INCLUDE_DIRS}
)

add_library(disparity_graph
            src/disparity_graph.cpp)

target_link_libraries(disparity_graph
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS})

ament_target_dependencies(disparity_graph
                      rclcpp
                      sensor_msgs
                      std_msgs
                      nav_msgs
                      tf2
                      tf2_ros
                      tf2_eigen
                      tf2_geometry_msgs
                      image_transport
                      image_geometry
                      visualization_msgs
                      cv_bridge
                      image_geometry
                      pcl_conversions
                      disparity_graph
                    )
#Install files                      
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(TARGETS disparity_graph
 #  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 #  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 #  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 #)
install(TARGETS disparity_graph
 DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)
#install(DIRECTORY include
##  DESTINATION include/${PROJECT_NAME}
#S)
install(DIRECTORY include/
  DESTINATION include/disparity_graph
  FILES_MATCHING PATTERN "*.h"
)


ament_package()

