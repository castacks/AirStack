cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules" ${CMAKE_MODULE_PATH})

option(BUILDING_TESTS "Build unit tests." ON)

project(vdb_mapping CXX C)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
set(OpenVDB_INCLUDE_DIR "/usr/local/include/openvdb" CACHE STRING "Path to OpenVDB installation")
find_package(OpenVDB REQUIRED)
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
find_package(ZSTD REQUIRED)

###########
## Build ##
###########


add_library(${PROJECT_NAME} INTERFACE)


### Declare a C++ library
target_include_directories(${PROJECT_NAME}
  INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${ZSTD_INCLUDE_DIRS}
    )

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    OpenVDB::openvdb
    ${EIGEN3_LIBRARIES}
    ${PCL_LIBRARIES}
    ${ZSTD_LIBRARIES}
    -lImath
    )

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

##
## Build testing if enabled by option
##
if (BUILDING_TESTS)
  enable_testing()
  add_subdirectory(tests)
else()
  message(STATUS "Building tests disabled.")
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  EXPORT ${PROJECT_NAME}_targets
  DESTINATION lib/cmake/${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  )

## Mark cpp header files for installation
 install(
   DIRECTORY include/${PROJECT_NAME}/
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING
   PATTERN "*.h"
   PATTERN "*.hpp"
 )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${PROJECT_NAME}ConfigVersion.cmake VERSION 1.0.0
  COMPATIBILITY SameMajorVersion)
install(
  FILES
    ${PROJECT_NAME}Config.cmake ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    CMakeModules/FindOpenVDB.cmake
    CMakeModules/OpenVDBUtils.cmake
    CMakeModules/FindBlosc.cmake
  DESTINATION
    lib/cmake/${PROJECT_NAME}
  )
