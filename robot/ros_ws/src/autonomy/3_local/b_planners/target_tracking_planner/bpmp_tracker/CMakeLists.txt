cmake_minimum_required(VERSION 3.8)
project(bpmp_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)  # (Yunwoo) For Detection3DArray
find_package(airstack_msgs REQUIRED)
find_package(airstack_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)

# (Yunwoo) DecompUtil - header-only library from parent directory
set(DECOMP_UTIL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../DecompUtil/include)

# --- node executable ---
add_executable(bpmp_tracker_node
  src/bpmp_tracker/bpmp_tracker_node.cpp
  src/bpmp_tracker/BPMPTracker.cpp
  src/bpmp_visualizer/BPMPVisualizer.cpp
  src/bpmp_visualizer/convhull_3d_impl.cpp  # (Yunwoo) convhull_3d implementation
  src/bpmp_utils/bpmp_utils.cpp  # (Yunwoo) Added bpmp_utils
)

ament_target_dependencies(bpmp_tracker_node
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  airstack_msgs
  airstack_common
  decomp_util
)

target_include_directories(bpmp_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${DECOMP_UTIL_INCLUDE_DIR}  # (Yunwoo) DecompUtil headers
)

target_compile_features(bpmp_tracker_node PUBLIC cxx_std_17)

# --- bpmp_predictor_node executable ---
add_executable(bpmp_predictor_node
  src/bpmp_predictor/bpmp_predictor_node.cpp
  src/bpmp_predictor/BPMPPredictor.cpp
)

ament_target_dependencies(bpmp_predictor_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

target_include_directories(bpmp_predictor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(bpmp_predictor_node PUBLIC cxx_std_17)

# --- install ---
install(TARGETS
  bpmp_tracker_node
  bpmp_predictor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
