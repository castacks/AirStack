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
# NatNet SDK — pre-built shared library downloaded via `airstack setup`.
# Headers: include/natnet/   Library: lib/libNatNet.so
#
# The SDK is proprietary and is NOT committed to the repository.
# Run `airstack setup` (or `airstack setup --natnet`) to download and place
# the SDK files before building this package.
#
# If the SDK is absent the C++ node is skipped with a warning; the Python
# vision_pose_converter_node.py and all launch/config files are still installed
# so the rest of the autonomy stack builds cleanly.
# ---------------------------------------------------------------------------
set(_NATNET_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/libNatNet.so")
set(_NATNET_INC "${CMAKE_CURRENT_SOURCE_DIR}/include/natnet")

if(EXISTS "${_NATNET_LIB}" AND EXISTS "${_NATNET_INC}")
  add_library(NatNet SHARED IMPORTED)
  set_target_properties(NatNet PROPERTIES
    IMPORTED_LOCATION "${_NATNET_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${_NATNET_INC}"
  )

  # C++ NatNet ROS 2 node
  add_executable(natnet_ros2_node
    src/natnet_ros2_node.cpp
    src/natnet_client_adapter.cpp)
  target_include_directories(natnet_ros2_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(natnet_ros2_node rclcpp geometry_msgs nav_msgs)
  target_link_libraries(natnet_ros2_node NatNet)

  install(TARGETS natnet_ros2_node
    DESTINATION lib/${PROJECT_NAME})

  # Install libNatNet.so alongside the node and register an environment hook so
  # that sourcing the workspace adds lib/natnet_ros2/ to LD_LIBRARY_PATH.
  # Use PROGRAMS (not FILES) to preserve the execute bit — shared libraries
  # must be executable for the dynamic linker to map them.
  install(PROGRAMS "${_NATNET_LIB}"
    DESTINATION lib/${PROJECT_NAME})

  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/natnet_library_path.dsv.in"
  )
else()
  message(WARNING
    "[natnet_ros2] NatNet SDK not found — skipping natnet_ros2_node build.\n"
    "  Expected: ${_NATNET_LIB}\n"
    "            ${_NATNET_INC}/\n"
    "  Run 'airstack setup' to download the OptiTrack NatNet SDK.")
endif()

# ---------------------------------------------------------------------------
# Python nodes (vision_pose_converter remains Python)
# ---------------------------------------------------------------------------
install(PROGRAMS
  src/vision_pose_converter_node.py
  src/mavros_gp_origin_node.py
  DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Launch and config files
# ---------------------------------------------------------------------------
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY include/
  DESTINATION include)

# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # --- gtest: pure-logic unit tests (no NatNet SDK, no rclcpp) ---
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_natnet_logic test/test_natnet_logic.cpp)
  target_include_directories(test_natnet_logic PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>)
endif()

ament_package()
