cmake_minimum_required(VERSION 3.8)
project(rviz_tasks_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
find_package(task_msgs REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rviz_polygon_selection_tool REQUIRED)
find_package(waypoint_rviz2_plugin REQUIRED)

qt5_wrap_cpp(MOC_FILES
  include/rviz_tasks_panel/tasks_panel.hpp
)

add_library(tasks_panel SHARED
  src/tasks_panel.cpp
  ${MOC_FILES}
)

target_include_directories(tasks_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tasks_panel
  pluginlib
  rviz_common
  rclcpp
  rclcpp_action
  task_msgs
  airstack_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  action_msgs
  rviz_polygon_selection_tool
  waypoint_rviz2_plugin
)

target_link_libraries(tasks_panel
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
)

install(TARGETS tasks_panel
  EXPORT export_rviz_tasks_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_rviz_tasks_panel)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()
