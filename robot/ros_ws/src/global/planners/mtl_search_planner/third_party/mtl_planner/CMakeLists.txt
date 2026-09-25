cmake_minimum_required(VERSION 3.16)

project(mtl_planner
        VERSION 1.0.0
        DESCRIPTION "Multi-agent, endurance-budgeted target localization planner"
        LANGUAGES CXX)

# -----------------------------------------------------------------------------
#  Options
# -----------------------------------------------------------------------------
option(MTL_BUILD_APPS      "Build the demo application"          ON)
option(MTL_BUILD_TESTS     "Build the self-tests"                ON)
option(MTL_ENABLE_WARNINGS "Enable a strict warning set"         ON)
option(MTL_ENABLE_OPENMP   "Parallelise per-agent planning with OpenMP" OFF)

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

if(MTL_ENABLE_OPENMP)
  find_package(OpenMP)
endif()

# -----------------------------------------------------------------------------
#  Warning set, applied through one interface target so every library shares it
#  without leaking the flags to consumers.
# -----------------------------------------------------------------------------
add_library(mtl_warnings INTERFACE)
if(MTL_ENABLE_WARNINGS)
  if(MSVC)
    target_compile_options(mtl_warnings INTERFACE /W4 /permissive-)
  else()
    target_compile_options(mtl_warnings INTERFACE
      -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wold-style-cast
      -Wcast-align -Wunused -Woverloaded-virtual -Wdouble-promotion)
  endif()
endif()

# =============================================================================
#  mtl_planner - the package you integrate.
#
#  Core geometry, the mapping abstraction, the budgeted route, the sensor sweep,
#  the trajectory and the gimbal schedule.  Depends on Eigen and nothing else.
# =============================================================================
add_library(mtl_planner
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

add_library(mtl::planner ALIAS mtl_planner)

target_include_directories(mtl_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(mtl_planner
  PUBLIC  Eigen3::Eigen
  PRIVATE mtl_warnings)

if(MTL_ENABLE_OPENMP AND OpenMP_CXX_FOUND)
  target_link_libraries(mtl_planner PUBLIC OpenMP::OpenMP_CXX)
  target_compile_definitions(mtl_planner PRIVATE MTL_HAVE_OPENMP=1)
endif()

set_target_properties(mtl_planner PROPERTIES
  VERSION   ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  EXPORT_NAME planner)

# =============================================================================
#  mtl_mapgen - scenario generation.  SEPARATE on purpose: a host simulation
#  brings its own prior and its own ground truth, and should not have to link
#  the reference one.
# =============================================================================
add_library(mtl_mapgen src/mapgen/scenario.cpp)
add_library(mtl::mapgen ALIAS mtl_mapgen)
target_link_libraries(mtl_mapgen PUBLIC mtl_planner PRIVATE mtl_warnings)
set_target_properties(mtl_mapgen PROPERTIES
  VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR} EXPORT_NAME mapgen)

# =============================================================================
#  mtl_eval - scoring, the geometry audit and the reports.  Also SEPARATE: the
#  planner's objective is "was the sensor pointed at it"; the detection physics
#  belongs to whoever is judging the run.
# =============================================================================
add_library(mtl_eval
  src/eval/detection.cpp
  src/eval/geometry_audit.cpp
  src/eval/report.cpp)
add_library(mtl::eval ALIAS mtl_eval)
target_link_libraries(mtl_eval PUBLIC mtl_planner PRIVATE mtl_warnings)
set_target_properties(mtl_eval PROPERTIES
  VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR} EXPORT_NAME eval)

# =============================================================================
#  Demo application - the C++ equivalent of main_new.m.
# =============================================================================
if(MTL_BUILD_APPS)
  add_executable(mtl_demo apps/demo_pipeline.cpp)
  target_link_libraries(mtl_demo PRIVATE mtl_planner mtl_mapgen mtl_eval mtl_warnings)

  # mtl_plan - the JSON adapter a simulator drives the planner through.
  # Links mtl_planner ALONE: the host brings its own scenario and its own
  # scoring, which is the whole reason mapgen and eval are separate libraries.
  add_executable(mtl_plan apps/mtl_plan_json.cpp)
  target_link_libraries(mtl_plan PRIVATE mtl_planner mtl_warnings)
  target_include_directories(mtl_plan PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/apps)
endif()

# =============================================================================
#  Tests - no external framework, so `ctest` works out of the box.
# =============================================================================
if(MTL_BUILD_TESTS)
  enable_testing()

  foreach(t test_dubins test_kmeans test_orienteering test_geometry test_pipeline)
    add_executable(${t} tests/${t}.cpp)
    target_link_libraries(${t} PRIVATE mtl_planner mtl_mapgen mtl_eval mtl_warnings)
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()

# =============================================================================
#  Install / export, so a consumer can do
#     find_package(mtl_planner REQUIRED)
#     target_link_libraries(my_sim PRIVATE mtl::planner)
# =============================================================================
install(TARGETS mtl_planner mtl_mapgen mtl_eval mtl_warnings
        EXPORT mtl_plannerTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT mtl_plannerTargets
        FILE mtl_plannerTargets.cmake
        NAMESPACE mtl::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mtl_planner)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mtl_plannerConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mtl_plannerConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/mtl_plannerConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mtl_planner)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/mtl_plannerConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/mtl_plannerConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mtl_planner)
