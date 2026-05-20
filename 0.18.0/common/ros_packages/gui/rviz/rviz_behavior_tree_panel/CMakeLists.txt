cmake_minimum_required(VERSION 3.8)
project(rviz_behavior_tree_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(behavior_tree_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build xdot_cpp library first
add_subdirectory(xdot_cpp)

qt5_wrap_cpp(MOC_FILES
  include/rviz_behavior_tree_panel/behavior_tree_panel.hpp
)

add_library(behavior_tree_panel 
  src/behavior_tree_panel.cpp 
  ${MOC_FILES}
)

target_include_directories(behavior_tree_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/xdot_cpp/include>
)

ament_target_dependencies(behavior_tree_panel
  pluginlib
  rviz_common
  std_msgs
  behavior_tree_msgs
  rclcpp
)

target_link_libraries(behavior_tree_panel
  xdot_qt
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
)

install(TARGETS behavior_tree_panel xdot_qt xdot_core
        EXPORT export_rviz_behavior_tree_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY xdot_cpp/include/
        DESTINATION include
)

install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

install(FILES icons/classes/BehaviorTreePanel.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)

ament_export_include_directories(include)
ament_export_targets(export_rviz_behavior_tree_panel)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()
