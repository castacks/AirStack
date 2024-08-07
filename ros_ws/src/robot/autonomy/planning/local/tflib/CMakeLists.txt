cmake_minimum_required(VERSION 2.8.3)
project(tflib)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp tf
  LIBRARIES tflib
)

include_directories(${catkin_INCLUDE_DIRS} include ${base_INCLUDE_DIRS})


add_library(tflib
  src/tflib.cpp
  )

target_link_libraries(tflib ${catkin_LIBRARIES} ${base_LIBRARIES})


install(TARGETS tflib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

