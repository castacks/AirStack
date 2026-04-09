cmake_minimum_required(VERSION 3.5)
project(waypoint_rviz2_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(pluginlib REQUIRED)

# Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

# Qt signals and slots to avoid defining "emit", "slots"
add_definitions(-DQT_NO_KEYWORDS)

# Header files that need Qt Moc pre-processing
qt5_wrap_cpp(MOC_FILES
  include/waypoint_rviz2_plugin/waypoint_widget.hpp
  include/waypoint_rviz2_plugin/waypoint_tool.hpp
)

# Convert the Qt UI files
qt5_wrap_ui(UIC_FILES
  ui/waypoint_plugin.ui
)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

# Plugin source files
set(SOURCES
  src/waypoint_widget.cpp
  src/waypoint_tool.cpp
  ${MOC_FILES}
)

# Declare the library
add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
  ${UIC_FILES}
)

# Dependencies
set(DEPENDENCIES
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  rviz_ogre_vendor
  geometry_msgs
  visualization_msgs
  interactive_markers
  nav_msgs
  tf2
  tf2_geometry_msgs
  rosbag2_cpp
  pluginlib
)

ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Export plugin description file for pluginlib
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
