cmake_minimum_required(VERSION 3.16)

project(mtl_curve_planner
        VERSION 1.0.0
        DESCRIPTION "Multi-agent parameterized-curve, information-aware search planner"
        LANGUAGES C CXX)

# -----------------------------------------------------------------------------
#  Options  (the same four as cpp_planner, MTLC_ prefix, plus the optional
#  head-to-head against it)
# -----------------------------------------------------------------------------
option(MTLC_BUILD_APPS      "Build the demo and the JSON adapter"          ON)
option(MTLC_BUILD_TESTS     "Build the self-tests"                         ON)
option(MTLC_ENABLE_WARNINGS "Enable a strict warning set"                  ON)
option(MTLC_ENABLE_OPENMP   "Parallelise multi-start seeds and the residual pass with OpenMP" OFF)
option(MTLC_BUILD_COMPARE   "Build mtlc_compare against the sibling ../cpp_planner (mtl::planner)" OFF)
option(MTLC_ENABLE_GCS      "Graph-of-convex-sets seed (InitStrategy::Gcs); needs the SCS conic solver" ON)
set(MTLC_SCS_SOURCE_DIR "" CACHE PATH "Local SCS source tree (empty = fetch SCS ${MTLC_SCS_TAG})")
set(MTLC_SCS_TAG "3.3.1" CACHE STRING "SCS release fetched when MTLC_SCS_SOURCE_DIR is empty")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# -----------------------------------------------------------------------------
#  Eigen3 - found if installed, otherwise fetched.  Header-only either way.
# -----------------------------------------------------------------------------
find_package(Eigen3 3.3 QUIET NO_MODULE)

if(NOT TARGET Eigen3::Eigen)
  message(STATUS "Eigen3 not found locally - fetching it")
  include(FetchContent)
  FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE)
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(eigen)
endif()

if(MTLC_ENABLE_OPENMP)
  find_package(OpenMP)
endif()

# -----------------------------------------------------------------------------
#  SCS (MIT, pure C) - the conic solver behind the GCS seed.  Built here as a
#  small static library from its sources (direct LDL solver, no BLAS/LAPACK:
#  the GCS problems are SOCPs, LAPACK is only needed for SDP cones).
# -----------------------------------------------------------------------------
if(MTLC_ENABLE_GCS)
  if(MTLC_SCS_SOURCE_DIR)
    set(_scs_dir "${MTLC_SCS_SOURCE_DIR}")
  else()
    include(FetchContent)
    FetchContent_Declare(
      mtlc_scs
      GIT_REPOSITORY https://github.com/cvxgrp/scs.git
      GIT_TAG        ${MTLC_SCS_TAG}
      GIT_SHALLOW    TRUE)
    FetchContent_GetProperties(mtlc_scs)
    if(NOT mtlc_scs_POPULATED)
      FetchContent_Populate(mtlc_scs)
    endif()
    set(_scs_dir "${mtlc_scs_SOURCE_DIR}")
  endif()
  file(GLOB _scs_amd "${_scs_dir}/linsys/external/amd/*.c")
  add_library(mtl_curve_scs STATIC
    ${_scs_dir}/src/linalg.c ${_scs_dir}/src/cones.c ${_scs_dir}/src/exp_cone.c
    ${_scs_dir}/src/aa.c ${_scs_dir}/src/util.c ${_scs_dir}/src/scs.c
    ${_scs_dir}/src/ctrlc.c ${_scs_dir}/src/normalize.c ${_scs_dir}/src/scs_version.c
    ${_scs_dir}/src/rw.c
    ${_scs_dir}/linsys/scs_matrix.c ${_scs_dir}/linsys/csparse.c
    ${_scs_dir}/linsys/cpu/direct/private.c
    ${_scs_dir}/linsys/external/qdldl/qdldl.c
    ${_scs_amd})
  target_include_directories(mtl_curve_scs
    PUBLIC  $<BUILD_INTERFACE:${_scs_dir}/include>
    PRIVATE ${_scs_dir}/linsys ${_scs_dir}/linsys/cpu/direct
            ${_scs_dir}/linsys/external/amd ${_scs_dir}/linsys/external/qdldl)
  target_compile_definitions(mtl_curve_scs PRIVATE CTRLC=0 VERBOSITY=0 _POSIX_C_SOURCE=200809L)
  set_target_properties(mtl_curve_scs PROPERTIES POSITION_INDEPENDENT_CODE ON C_STANDARD 99)
  find_package(Threads REQUIRED)
  target_link_libraries(mtl_curve_scs PRIVATE Threads::Threads $<$<NOT:$<PLATFORM_ID:Windows>>:m>)
endif()

# -----------------------------------------------------------------------------
#  Warning set, applied through one interface target.
# -----------------------------------------------------------------------------
add_library(mtl_curve_warnings INTERFACE)
if(MTLC_ENABLE_WARNINGS)
  if(MSVC)
    target_compile_options(mtl_curve_warnings INTERFACE /W4 /permissive-)
  else()
    target_compile_options(mtl_curve_warnings INTERFACE
      -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wold-style-cast
      -Wcast-align -Wunused -Woverloaded-virtual -Wdouble-promotion)
  endif()
endif()

# =============================================================================
#  mtl_curve_planner - the package you integrate.
#
#  Copied from cpp_planner (namespace mtl -> mtl::curve): core/numeric,
#  core/kmeans, mapping/cells, planning/orienteering, sensing/airframe.
#  New: core/bspline, curve_planning/*, optimization/*, sensing/sweep,
#  trajectory/curve_trajectory.  Depends on Eigen and nothing else.
# =============================================================================
add_library(mtl_curve_planner
  src/params.cpp
  src/planner.cpp
  src/core/numeric.cpp
  src/core/kmeans.cpp
  src/core/bspline.cpp
  src/mapping/cells.cpp
  src/planning/orienteering.cpp
  src/planning/sector_orienteering.cpp
  src/curve_planning/endpoint_constraints.cpp
  src/curve_planning/parametric_curve.cpp
  src/curve_planning/arc_length.cpp
  src/curve_planning/init_spline.cpp
  src/curve_planning/path_insert.cpp
  src/curve_planning/swath_polygon.cpp
  src/curve_planning/team_curves.cpp
  src/curve_planning/info_aware.cpp
  src/mapping/peak_clusters.cpp
  src/optimization/fast_grid.cpp
  src/optimization/swath_kernel.cpp
  src/optimization/objective.cpp
  src/optimization/optimizer.cpp
  src/sensing/sweep.cpp
  src/sensing/airframe.cpp
  src/trajectory/curve_trajectory.cpp
  src/quadrotor/flatness.cpp
  src/quadrotor/quad_trajectory.cpp
  src/gcs/kernel_clusters.cpp
  src/gcs/convex_sets.cpp
  src/gcs/conic.cpp
  src/gcs/tour.cpp
  src/gcs/plan_agent.cpp)

add_library(mtl_curve::planner ALIAS mtl_curve_planner)

target_include_directories(mtl_curve_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(mtl_curve_planner
  PUBLIC  Eigen3::Eigen
  PRIVATE mtl_curve_warnings)

if(MTLC_ENABLE_GCS)
  target_link_libraries(mtl_curve_planner PRIVATE mtl_curve_scs)
  target_compile_definitions(mtl_curve_planner PRIVATE MTLC_HAVE_SCS=1)
endif()

if(MTLC_ENABLE_OPENMP AND OpenMP_CXX_FOUND)
  target_link_libraries(mtl_curve_planner PUBLIC OpenMP::OpenMP_CXX)
  target_compile_definitions(mtl_curve_planner PRIVATE MTLC_HAVE_OPENMP=1)
endif()

set_target_properties(mtl_curve_planner PROPERTIES
  VERSION   ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  EXPORT_NAME planner)

# =============================================================================
#  mtl_curve_mapgen - scenario generation (copied from cpp_planner, so the same
#  seed gives the same scenario in both packages).  SEPARATE on purpose: a host
#  brings its own prior and ground truth.
# =============================================================================
add_library(mtl_curve_mapgen src/mapgen/scenario.cpp)
add_library(mtl_curve::mapgen ALIAS mtl_curve_mapgen)
target_link_libraries(mtl_curve_mapgen PUBLIC mtl_curve_planner PRIVATE mtl_curve_warnings)
set_target_properties(mtl_curve_mapgen PROPERTIES
  VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR} EXPORT_NAME mapgen)

# =============================================================================
#  mtl_curve_eval - scoring and reports.  detection.cpp (targets + residual
#  belief) is copied from cpp_planner, so both planners are scored by the same
#  code.  SEPARATE: the detection physics belongs to whoever judges the run.
# =============================================================================
add_library(mtl_curve_eval
  src/eval/detection.cpp
  src/eval/report.cpp)
add_library(mtl_curve::eval ALIAS mtl_curve_eval)
target_link_libraries(mtl_curve_eval PUBLIC mtl_curve_planner PRIVATE mtl_curve_warnings)
if(MTLC_ENABLE_OPENMP AND OpenMP_CXX_FOUND)
  target_compile_definitions(mtl_curve_eval PRIVATE MTL_HAVE_OPENMP=1)
endif()
set_target_properties(mtl_curve_eval PROPERTIES
  VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR} EXPORT_NAME eval)

# =============================================================================
#  Applications
# =============================================================================
if(MTLC_BUILD_APPS)
  # mtlc_demo - the C++ equivalent of main_curve_planner.m.
  add_executable(mtlc_demo apps/demo_curve_pipeline.cpp)
  target_link_libraries(mtlc_demo PRIVATE mtl_curve_planner mtl_curve_mapgen mtl_curve_eval mtl_curve_warnings)

  # mtlc_plan - the JSON adapter (mtl.scenario/1 -> mtl.plan/1), a drop-in for
  # cpp_planner's mtl_plan.  Links mtl_curve_planner ALONE.
  add_executable(mtlc_plan apps/mtl_curve_plan_json.cpp)
  target_link_libraries(mtlc_plan PRIVATE mtl_curve_planner mtl_curve_warnings)
  target_include_directories(mtlc_plan PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/apps)

  # mtlc_bench_gcs - static-stage benchmark: GCS seed vs orienteering / greedy.
  add_executable(mtlc_bench_gcs apps/bench_gcs.cpp)
  target_link_libraries(mtlc_bench_gcs PRIVATE mtl_curve_planner mtl_curve_mapgen mtl_curve_eval mtl_curve_warnings)
  add_executable(mtlc_bench_study apps/bench_study.cpp)
  target_link_libraries(mtlc_bench_study PRIVATE mtl_curve_planner mtl_curve_mapgen mtl_curve_eval mtl_curve_warnings)
endif()

# mtlc_compare - both planners on the same scenario, scored by the same
# residual.  Builds the sibling cpp_planner in-tree; the two packages live in
# different namespaces (mtl, mtl::curve), so they link side by side.
if(MTLC_BUILD_COMPARE)
  set(MTLC_CPP_PLANNER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../cpp_planner" CACHE PATH "cpp_planner source dir")
  if(NOT TARGET mtl::planner)
    set(MTL_BUILD_APPS OFF CACHE BOOL "" FORCE)
    set(MTL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(${MTLC_CPP_PLANNER_DIR} ${CMAKE_CURRENT_BINARY_DIR}/cpp_planner EXCLUDE_FROM_ALL)
  endif()
  add_executable(mtlc_compare apps/compare_planners.cpp)
  target_link_libraries(mtlc_compare PRIVATE mtl_curve_planner mtl_curve_mapgen mtl_curve_eval
                        mtl::planner mtl::mapgen mtl::eval)
endif()

# =============================================================================
#  Tests - no external framework, so `ctest` works out of the box.
# =============================================================================
if(MTLC_BUILD_TESTS)
  enable_testing()

  foreach(t test_kmeans test_orienteering test_curve_geometry test_swath_kernel test_optimizer test_path_insert test_pipeline test_gcs test_info_aware_clusters test_quadrotor test_sector_orienteering)
    add_executable(${t} tests/${t}.cpp)
    target_link_libraries(${t} PRIVATE mtl_curve_planner mtl_curve_mapgen mtl_curve_eval mtl_curve_warnings)
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()

# =============================================================================
#  Install / export, so a consumer can do
#     find_package(mtl_curve_planner REQUIRED)
#     target_link_libraries(my_sim PRIVATE mtl_curve::planner)
# =============================================================================
set(_mtlc_install_targets mtl_curve_planner mtl_curve_mapgen mtl_curve_eval mtl_curve_warnings)
if(MTLC_ENABLE_GCS)
  list(APPEND _mtlc_install_targets mtl_curve_scs)
endif()
install(TARGETS ${_mtlc_install_targets}
        EXPORT mtl_curve_plannerTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT mtl_curve_plannerTargets
        FILE mtl_curve_plannerTargets.cmake
        NAMESPACE mtl_curve::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mtl_curve_planner)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mtl_curve_plannerConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mtl_curve_plannerConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/mtl_curve_plannerConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mtl_curve_planner)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/mtl_curve_plannerConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/mtl_curve_plannerConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mtl_curve_planner)
