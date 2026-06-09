cmake_minimum_required(VERSION 3.8)
project(modalai_livox_tf_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(modalai_livox_tf_bridge_node
  src/modalai_tfpub_lidar.cpp
)

target_compile_features(modalai_livox_tf_bridge_node PUBLIC cxx_std_17)
target_link_libraries(modalai_livox_tf_bridge_node Eigen3::Eigen)

ament_target_dependencies(modalai_livox_tf_bridge_node
  geometry_msgs
  nav_msgs
  px4_msgs
  rclcpp
  tf2
  tf2_eigen
  tf2_ros
)

install(TARGETS modalai_livox_tf_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
