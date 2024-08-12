cmake_minimum_required(VERSION 2.8.3)
project(base)

add_compile_options(-Wno-reorder -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nodelet
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES base_node base_nodelet
  CATKIN_DEPENDS roscpp std_msgs nodelet
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

#============================================================================================
# ---------------------------- Base Nodelet Library -----------------------------------------
#============================================================================================

add_library(base_nodelet
  src/Base.cpp
  src/BaseNodelet.cpp
  src/HealthMonitor.cpp
)

add_dependencies(base_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(base_nodelet
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


#target_link_libraries(base_nodelet ${catkin_LIBRARIES})
#if(catkin_EXPORTED_LIBRARIES)
#  add_dependencies(base_nodelet ${catkin_EXPORTED_LIBRARIES})
#endif()

#install(FILES base_nodelet.xml
#DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#============================================================================================
# ---------------------------- Base Node Library --------------------------------------------
#============================================================================================

add_library(base_node
  src/Base.cpp
  src/BaseNode.cpp
  src/HealthMonitor.cpp
  src/main.cpp
)

add_dependencies(base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(base_node
  ${catkin_LIBRARIES}
)

install(TARGETS base_nodelet base_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/base/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

