cmake_minimum_required(VERSION 3.8)
project(trajectory_library)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(airstack_common REQUIRED)

add_library(trajectory_library SHARED src/trajectory_library.cpp)
target_include_directories(trajectory_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(trajectory_library
  yaml-cpp
)
target_compile_features(trajectory_library PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
  trajectory_library
  # Required dependencies
  nav_msgs
  visualization_msgs
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  airstack_msgs
  airstack_common
)
ament_export_targets(trajectory_library HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs visualization_msgs tf2 tf2_ros airstack_common)

install(TARGETS trajectory_library
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS trajectory_library
  EXPORT trajectory_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})


install(PROGRAMS scripts/fixed_trajectory_generator.py DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(
  include
  ${mav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
