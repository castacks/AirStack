# -----------------------------------------------------------------------------
#  mtl_vendored.cmake — build the vendored cpp_planner (mtl::planner) as static,
#  position-independent libraries inside this package.
#
#  The vendored tree carries one AirStack-side addition (see
#  third_party/VENDORED.md): the information-aware abstraction search
#  (mapping/peak_clusters, planning/coverage_score, planning/info_aware and
#  tests/test_info_aware.cpp), off unless the scenario enables it.
#
#  Why not add_subdirectory(third_party/mtl_planner): its install()/export rules
#  would install a second copy of the headers and a CMake package into the ROS
#  install space. Compiling the same source list here keeps the vendored tree
#  byte-identical to upstream (see third_party/VENDORED.md) and the install
#  space clean. The only dependency is Eigen3 (never fetched here: a ROS image
#  always has it via eigen3_cmake_module / libeigen3-dev).
#
#  Defines:
#    mtl_planner_vendored        the planning chain            (upstream mtl::planner)
#    mtl_eval_vendored           mapgen + detection/audit/report (upstream mtl::mapgen + mtl::eval),
#                                only needed by the upstream self-tests
#    mtl_curve_planner_vendored  the parameterized-curve planner (upstream mtl_curve::planner,
#                                namespace mtl::curve, third_party/mtl_curve_planner, unmodified)
#    mtl_curve_eval_vendored     its mapgen + detection/report (mtl_curve::mapgen + mtl_curve::eval),
#                                only needed by its self-tests
#
#  The two planners live in different namespaces (mtl, mtl::curve) and header
#  trees (mtl/, mtl_curve/), so both link into one binary (the adapter does).
# -----------------------------------------------------------------------------
set(MTL_VENDOR_DIR "${CMAKE_CURRENT_LIST_DIR}/../third_party/mtl_planner")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)   # the info-aware search plans candidates on worker threads

set(MTL_VENDOR_PLANNER_SOURCES
  src/params.cpp
  src/planner.cpp
  src/core/numeric.cpp
  src/core/dubins.cpp
  src/core/kmeans.cpp
  src/mapping/cells.cpp
  src/mapping/peak_clusters.cpp
  src/routing/tsp.cpp
  src/planning/info_score.cpp
  src/planning/coverage_score.cpp
  src/planning/info_aware.cpp
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
target_link_libraries(mtl_planner_vendored PUBLIC Eigen3::Eigen Threads::Threads)
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

# Upstream self-tests (plain executables, no framework; test_info_aware is the AirStack addition): registered with CTest
# so `colcon test --packages-select mtl_search_planner` runs them.
function(mtl_vendored_add_selftests)
  foreach(t test_dubins test_kmeans test_orienteering test_geometry test_pipeline test_info_aware)
    add_executable(mtl_vendored_${t} "${MTL_VENDOR_DIR}/tests/${t}.cpp")
    target_link_libraries(mtl_vendored_${t} PRIVATE mtl_eval_vendored)
    target_include_directories(mtl_vendored_${t} PRIVATE "${MTL_VENDOR_DIR}/tests")
    set_target_properties(mtl_vendored_${t} PROPERTIES CXX_STANDARD 17)
    add_test(NAME mtl_vendored_${t} COMMAND mtl_vendored_${t})
  endforeach()
endfunction()

# =============================================================================
#  The parameterized-curve planner (upstream cpp_curve_planner, vendored
#  byte-identical into third_party/mtl_curve_planner; see VENDORED.md).  Same
#  pattern as above: an explicit source list (upstream CMakeLists.txt), no
#  add_subdirectory, so nothing of it is installed.
# =============================================================================
set(MTL_CURVE_VENDOR_DIR "${CMAKE_CURRENT_LIST_DIR}/../third_party/mtl_curve_planner")

set(MTL_CURVE_VENDOR_PLANNER_SOURCES
  src/params.cpp
  src/planner.cpp
  src/core/numeric.cpp
  src/core/kmeans.cpp
  src/core/bspline.cpp
  src/mapping/cells.cpp
  src/planning/orienteering.cpp
  src/curve_planning/endpoint_constraints.cpp
  src/curve_planning/parametric_curve.cpp
  src/curve_planning/arc_length.cpp
  src/curve_planning/init_spline.cpp
  src/curve_planning/swath_polygon.cpp
  src/curve_planning/team_curves.cpp
  src/optimization/fast_grid.cpp
  src/optimization/swath_kernel.cpp
  src/optimization/objective.cpp
  src/optimization/optimizer.cpp
  src/sensing/sweep.cpp
  src/sensing/airframe.cpp
  src/trajectory/curve_trajectory.cpp)
list(TRANSFORM MTL_CURVE_VENDOR_PLANNER_SOURCES PREPEND "${MTL_CURVE_VENDOR_DIR}/")

add_library(mtl_curve_planner_vendored STATIC ${MTL_CURVE_VENDOR_PLANNER_SOURCES})
target_include_directories(mtl_curve_planner_vendored PUBLIC "${MTL_CURVE_VENDOR_DIR}/include")
target_link_libraries(mtl_curve_planner_vendored PUBLIC Eigen3::Eigen)
set_target_properties(mtl_curve_planner_vendored PROPERTIES
  POSITION_INDEPENDENT_CODE ON CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Debug")
  # Tens of seconds per plan at -O2; minutes at -O0. Keep it optimised in a Debug workspace.
  target_compile_options(mtl_curve_planner_vendored PRIVATE -O2)
endif()
# MTLC_HAVE_OPENMP stays undefined: the multi-start seeds run serially, so a plan is
# deterministic and identical to the stock mtlc_plan build (MTLC_ENABLE_OPENMP=OFF).

set(MTL_CURVE_VENDOR_EVAL_SOURCES
  src/mapgen/scenario.cpp
  src/eval/detection.cpp
  src/eval/report.cpp)
list(TRANSFORM MTL_CURVE_VENDOR_EVAL_SOURCES PREPEND "${MTL_CURVE_VENDOR_DIR}/")
add_library(mtl_curve_eval_vendored STATIC EXCLUDE_FROM_ALL ${MTL_CURVE_VENDOR_EVAL_SOURCES})
target_link_libraries(mtl_curve_eval_vendored PUBLIC mtl_curve_planner_vendored)
set_target_properties(mtl_curve_eval_vendored PROPERTIES
  POSITION_INDEPENDENT_CODE ON CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(mtl_curve_eval_vendored PRIVATE -O2)
endif()

# The six upstream curve self-tests (plain executables, no framework), under a
# distinct prefix so they never collide with the mtl_planner ones.
function(mtl_curve_vendored_add_selftests)
  foreach(t test_kmeans test_orienteering test_curve_geometry test_swath_kernel test_optimizer test_pipeline)
    add_executable(mtl_curve_vendored_${t} "${MTL_CURVE_VENDOR_DIR}/tests/${t}.cpp")
    target_link_libraries(mtl_curve_vendored_${t} PRIVATE mtl_curve_eval_vendored)
    target_include_directories(mtl_curve_vendored_${t} PRIVATE "${MTL_CURVE_VENDOR_DIR}/tests")
    set_target_properties(mtl_curve_vendored_${t} PROPERTIES CXX_STANDARD 17)
    if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Debug")
      target_compile_options(mtl_curve_vendored_${t} PRIVATE -O2)
    endif()
    add_test(NAME mtl_curve_vendored_${t} COMMAND mtl_curve_vendored_${t})
  endforeach()
endfunction()
