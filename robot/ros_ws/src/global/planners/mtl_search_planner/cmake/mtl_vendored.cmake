# -----------------------------------------------------------------------------
#  mtl_vendored.cmake — build the vendored cpp_planner (mtl::planner) as static,
#  position-independent libraries inside this package.
#
#  Why not add_subdirectory(third_party/mtl_planner): its install()/export rules
#  would install a second copy of the headers and a CMake package into the ROS
#  install space. Compiling the same source list here keeps the vendored tree
#  byte-identical to upstream (see third_party/VENDORED.md) and the install
#  space clean. The only dependency is Eigen3 (never fetched here: a ROS image
#  always has it via eigen3_cmake_module / libeigen3-dev).
#
#  Defines:
#    mtl_planner_vendored   the planning chain            (upstream mtl::planner)
#    mtl_eval_vendored      mapgen + detection/audit/report (upstream mtl::mapgen + mtl::eval),
#                           only needed by the upstream self-tests
# -----------------------------------------------------------------------------
set(MTL_VENDOR_DIR "${CMAKE_CURRENT_LIST_DIR}/../third_party/mtl_planner")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(MTL_VENDOR_PLANNER_SOURCES
  src/params.cpp
  src/planner.cpp
  src/core/numeric.cpp
  src/core/dubins.cpp
  src/core/kmeans.cpp
  src/mapping/cells.cpp
  src/routing/tsp.cpp
  src/planning/info_score.cpp
  src/planning/orienteering.cpp
  src/planning/macro_route.cpp
  src/planning/cell_anchors.cpp
  src/planning/agent_sortie.cpp
  src/planning/team_allocation.cpp
  src/trajectory/trajectory_gen.cpp
  src/trajectory/lateral_coverage.cpp
  src/sensing/abeam.cpp
  src/sensing/airframe.cpp
  src/sensing/gimbal_scheduler.cpp)
list(TRANSFORM MTL_VENDOR_PLANNER_SOURCES PREPEND "${MTL_VENDOR_DIR}/")

add_library(mtl_planner_vendored STATIC ${MTL_VENDOR_PLANNER_SOURCES})
target_include_directories(mtl_planner_vendored PUBLIC "${MTL_VENDOR_DIR}/include")
target_link_libraries(mtl_planner_vendored PUBLIC Eigen3::Eigen)
set_target_properties(mtl_planner_vendored PROPERTIES
  POSITION_INDEPENDENT_CODE ON CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Debug")
  # The planner is O(ms) at -O2 but seconds at -O0; keep it optimised even in
  # a Debug workspace build so bws --cmake-args -DCMAKE_BUILD_TYPE=Debug stays usable.
  target_compile_options(mtl_planner_vendored PRIVATE -O2)
endif()

set(MTL_VENDOR_EVAL_SOURCES
  src/mapgen/scenario.cpp
  src/eval/detection.cpp
  src/eval/geometry_audit.cpp
  src/eval/report.cpp)
list(TRANSFORM MTL_VENDOR_EVAL_SOURCES PREPEND "${MTL_VENDOR_DIR}/")
add_library(mtl_eval_vendored STATIC EXCLUDE_FROM_ALL ${MTL_VENDOR_EVAL_SOURCES})
target_link_libraries(mtl_eval_vendored PUBLIC mtl_planner_vendored)
set_target_properties(mtl_eval_vendored PROPERTIES
  POSITION_INDEPENDENT_CODE ON CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

# Upstream self-tests (plain executables, no framework): registered with CTest
# so `colcon test --packages-select mtl_search_planner` runs them.
function(mtl_vendored_add_selftests)
  foreach(t test_dubins test_kmeans test_orienteering test_geometry test_pipeline)
    add_executable(mtl_vendored_${t} "${MTL_VENDOR_DIR}/tests/${t}.cpp")
    target_link_libraries(mtl_vendored_${t} PRIVATE mtl_eval_vendored)
    target_include_directories(mtl_vendored_${t} PRIVATE "${MTL_VENDOR_DIR}/tests")
    set_target_properties(mtl_vendored_${t} PROPERTIES CXX_STANDARD 17)
    add_test(NAME mtl_vendored_${t} COMMAND mtl_vendored_${t})
  endforeach()
endfunction()
