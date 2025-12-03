cmake_minimum_required(VERSION 3.8)
project(droan_gl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(airstack_common REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(trajectory_library REQUIRED)

#add_executable(droan src/droan.cpp src/glad/src/glad.c)
#target_include_directories(droan PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(droan PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
#ament_target_dependencies(droan rclcpp sensor_msgs stereo_msgs nav_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge ament_index_cpp pcl_conversions airstack_common airstack_msgs)
#target_link_libraries(droan glfw dl assimp EGL GL)


add_executable(expand src/expand.cpp src/gl_interface.cpp src/global_plan.cpp src/rewind_monitor.cpp src/glad/src/glad.c)
target_include_directories(expand PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(expand PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(expand rclcpp sensor_msgs stereo_msgs nav_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge ament_index_cpp pcl_conversions airstack_common airstack_msgs trajectory_library)
target_link_libraries(expand glfw dl assimp EGL GL)

#install(TARGETS droan DESTINATION lib/${PROJECT_NAME})
install(TARGETS expand DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY shaders DESTINATION share/${PROJECT_NAME})

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
