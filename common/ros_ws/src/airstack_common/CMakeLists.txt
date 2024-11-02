cmake_minimum_required(VERSION 3.8)
project(airstack_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)


# ros2_helper
add_library(ros2_helper src/ros2_helper.cpp)
ament_target_dependencies(
  ros2_helper
  rclcpp
  )
install(
  TARGETS ros2_helper
  EXPORT ros2_helper
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )
ament_export_libraries(ros2_helper)

# tflib
add_library(tflib SHARED src/tflib.cpp)
target_include_directories(tflib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tflib PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  tflib
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
  airstack_msgs
  tf2_geometry_msgs
)
install(TARGETS tflib
  DESTINATION lib/${PROJECT_NAME}
  )
install(
  TARGETS tflib
  EXPORT tflib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )
ament_export_libraries(tflib)

# vislib
add_library(vislib src/vislib.cpp)
target_include_directories(vislib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(vislib PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  vislib
  visualization_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
  airstack_msgs
  tf2_geometry_msgs
)
install(TARGETS vislib
  DESTINATION lib/${PROJECT_NAME}
  )
install(
  TARGETS vislib
  EXPORT vislib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )
ament_export_libraries(vislib)


# install
ament_export_include_directories(include)
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  )
ament_export_dependencies(rclcpp nav_msgs geometry_msgs tf2 tf2_ros airstack_msgs tf2_geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
