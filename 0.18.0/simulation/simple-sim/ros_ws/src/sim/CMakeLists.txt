cmake_minimum_required(VERSION 3.8)
project(sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_executable(sim src/sim_node.cpp src/drone.cpp src/mesh_collada.cpp src/mesh_fbx.cpp src/shader_utils.cpp src/glad/src/glad.c)
target_include_directories(sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sim PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(sim rclcpp mavros_msgs nav_msgs tf2 tf2_geometry_msgs rosgraph_msgs)
target_link_libraries(sim glfw dl assimp)

install(TARGETS sim DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
