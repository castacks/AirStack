cmake_minimum_required(VERSION 3.8)
project(mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CGAL_DATA_DIR ".")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(CGAL REQUIRED)

add_executable(mission_manager_node
  src/mission_manager_node.cpp
  src/MissionManager.cpp
  src/BeliefMap.cpp)

add_executable(example_search_request_node
  src/example_search_request.cpp)

ament_target_dependencies(mission_manager_node rclcpp std_msgs airstack_msgs grid_map_ros)
ament_target_dependencies(example_search_request_node rclcpp airstack_msgs)

target_include_directories(mission_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CGAL_INCLUDE_DIRS})

target_include_directories(example_search_request_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(mission_manager_node
  ${CGAL_LIBRARIES}  # Link CGAL libraries
)

target_compile_features(mission_manager_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(example_search_request_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mission_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS example_search_request_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
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
