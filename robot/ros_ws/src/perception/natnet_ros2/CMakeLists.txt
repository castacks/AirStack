cmake_minimum_required(VERSION 3.8)
project(natnet_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament / ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ---------------------------------------------------------------------------
# NatNet SDK — pre-built shared library bundled in this package.
# Headers: include/natnet/   Library: lib/libNatNet.so
# ---------------------------------------------------------------------------
add_library(NatNet SHARED IMPORTED)
set_target_properties(NatNet PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libNatNet.so"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include/natnet"
)

# ---------------------------------------------------------------------------
# C++ NatNet ROS 2 node
# ---------------------------------------------------------------------------
add_executable(natnet_ros2_node src/natnet_ros2_node.cpp)
ament_target_dependencies(natnet_ros2_node rclcpp geometry_msgs nav_msgs)
target_link_libraries(natnet_ros2_node NatNet)

# $ORIGIN RPATH so the node finds libNatNet.so in the same install directory
# without needing LD_LIBRARY_PATH at runtime.
set_target_properties(natnet_ros2_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS natnet_ros2_node
  DESTINATION lib/${PROJECT_NAME})

# Install the bundled NatNet .so alongside the executable
install(FILES lib/libNatNet.so
  DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Python nodes (vision_pose_converter remains Python)
# ---------------------------------------------------------------------------
install(PROGRAMS
  src/vision_pose_converter_node.py
  DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Launch and config files
# ---------------------------------------------------------------------------
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
