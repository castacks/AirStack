cmake_minimum_required(VERSION 3.5)
project(behavior_tree)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behavior_tree_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)

include_directories(include ${rclcpp_INCLUDE_DIRS} ${behavior_tree_msgs_INCLUDE_DIRS})

add_executable(behavior_tree_implementation src/behavior_tree_implementation.cpp)
target_compile_features(behavior_tree_implementation PUBLIC c_std_99 cxx_std_17)
target_link_libraries(behavior_tree_implementation
  ${Boost_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${behavior_tree_msgs_LIBRARIES})
#ament_target_dependencies(behavior_tree_implementation rclcpp std_msgs behavior_tree_msgs)

add_library(behavior_tree src/behavior_tree.cpp)
ament_target_dependencies(behavior_tree rclcpp std_msgs behavior_tree_msgs)
ament_export_targets(behavior_tree HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/behavior_tree
  DESTINATION include
  )

install(
  TARGETS behavior_tree
  EXPORT behavior_tree
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  behavior_tree_implementation
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install files.
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_package()
