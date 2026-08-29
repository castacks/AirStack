cmake_minimum_required(VERSION 3.14.4)
project(global_mapper_ros LANGUAGES C CXX)  # asm_mighty: C enabled so MPI::MPI_C resolves for vtk-config

# ————————————————————————————————————————————————————————
# Use ccache if available
# ————————————————————————————————————————————————————————
find_program(CCACHE ccache)
if(CCACHE)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK   ccache)
endif()

# ————————————————————————————————————————————————————————
# C++17 + warnings, default Release
# ————————————————————————————————————————————————————————
set(CMAKE_CXX_STANDARD 17 CACHE STRING "" FORCE)
set(CMAKE_BUILD_TYPE    Release CACHE STRING "" FORCE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ————————————————————————————————————————————————————————
# Find ROS2 / ament dependencies
# ————————————————————————————————————————————————————————
find_package(ament_cmake             REQUIRED)
find_package(MPI COMPONENTS C QUIET)  # asm_mighty: define MPI::MPI_C before PCL pulls vtk-config (VTK::mpi links it)
find_package(rclcpp                   REQUIRED)
find_package(global_mapper            REQUIRED)
find_package(fla_utils                REQUIRED)
find_package(pcl_conversions          REQUIRED)
find_package(PCL 1.12 COMPONENTS common segmentation search kdtree REQUIRED)
find_package(tf2_ros                  REQUIRED)
find_package(tf2_sensor_msgs          REQUIRED)
find_package(tf2_eigen                REQUIRED)
find_package(std_msgs                 REQUIRED)
find_package(sensor_msgs              REQUIRED)
find_package(nav_msgs                 REQUIRED)
find_package(geometry_msgs            REQUIRED)
find_package(visualization_msgs       REQUIRED)
find_package(image_transport          REQUIRED)
find_package(cv_bridge                REQUIRED)
find_package(fla_interfaces           REQUIRED)
find_package(dynus_interfaces           REQUIRED)
find_package(builtin_interfaces       REQUIRED)
find_package(Eigen3                   REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

message(STATUS ">>> global_mapper_INCLUDE_DIRS = ${global_mapper_INCLUDE_DIRS}")
message(STATUS ">>> global_mapper_LIBRARY_DIRS = ${global_mapper_LIBRARY_DIRS}")
message(STATUS ">>> global_mapper_LIBRARIES    = ${global_mapper_LIBRARIES}")

# make sure we know where its headers *and* libs are
include_directories(${global_mapper_INCLUDE_DIRS})
link_directories(   ${global_mapper_LIBRARY_DIRS})

# ————————————————————————————————————————————————————————
# Your node executable
# ————————————————————————————————————————————————————————
add_executable(global_mapper_node
  src/global_mapper_ros.cc
  src/obstacle_tracker.cc
)

# ————————————————————————————————————————————————————————
# Include paths for your headers + external libs
# ————————————————————————————————————————————————————————
target_include_directories(global_mapper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../fla_utils/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ————————————————————————————————————————————————————————
# Pull in all the ament-provided libraries
# ————————————————————————————————————————————————————————
ament_target_dependencies(global_mapper_node
  rclcpp
  global_mapper
  fla_utils
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  tf2_eigen
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  image_transport
  cv_bridge
  fla_interfaces
  dynus_interfaces
  builtin_interfaces
  Eigen3
)

# bring in global_mapper’s dependencies:
# pick up *all* of the global_mapper libs (and its sub‐libs) plus PCL
target_link_libraries(global_mapper_node
  ${PCL_LIBRARIES}
  ${global_mapper_LIBRARIES}   # this expands to the four .so’s
)

# ————————————————————————————————————————————————————————
# Install headers & the node executable
# ————————————————————————————————————————————————————————
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS global_mapper_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install for launch and config files
install(DIRECTORY
cfg
launch
DESTINATION share/${PROJECT_NAME}
)

# ————————————————————————————————————————————————————————
# Lint & tests
# ————————————————————————————————————————————————————————
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_cpplint_FOUND   TRUE)
#   set(ament_cmake_copyright_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
