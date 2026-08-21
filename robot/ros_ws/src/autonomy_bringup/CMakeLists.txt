cmake_minimum_required(VERSION 3.8)
project(autonomy_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# config/ carries the shared DDS-router/domain-bridge YAMLs (moved up from
# the removed AUTONOMY_ROLE role folders onboard_all/ and
# onboard_local_offboard_global/ — stacks select them directly).
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
