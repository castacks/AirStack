cmake_minimum_required(VERSION 2.8.3)
project(disparity_graph)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ca_common
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  tictoc_profiler
  pcl_conversions
)

# CONFIGURE OPENCV
find_package(OpenCV REQUIRED)

# CONFIGURE PCL
find_package(PCL REQUIRED)

#CONFIGURER ARMADILLO
find_package(Armadillo REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES disparity_graph
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf image_transport image_geometry
  DEPENDS eigen Boost PCL
)

#DEBUGGGGGGGGGGGEEEEEEEEEEERRRRRRRRRRR
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -std=gnu++0x")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

add_library(disparity_graph
            src/disparity_graph.cpp)

target_link_libraries(disparity_graph
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS})

#Install files                      
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

 install(TARGETS disparity_graph
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


