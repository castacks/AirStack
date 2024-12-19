cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../CMakeModules" ${CMAKE_MODULE_PATH})

project(vdb_mapping_tests CXX)

# This will allow cmake 3.0 and above to fail on missing link targets (when specified with an double colon syntax)
# Note that not all packages are support the colon syntax, but if they do it is recomended to use this syntax, since it is less error prone.
if(POLICY CMP0028)
  cmake_policy(SET CMP0028 NEW)
endif()

##
## Find dependencies for testing
##
include(FindGTestPackage)
find_gtest_package(GTEST_LIBRARIES)
if (NOT TARGET vdb_mapping::vdb_mapping)
  find_package(vdb_mapping REQUIRED)
endif()

###########
## Build ##
###########

add_executable(mapping_tests mapping.cpp)
target_compile_features(mapping_tests PUBLIC cxx_std_14)
target_include_directories(mapping_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(mapping_tests PRIVATE vdb_mapping::vdb_mapping ${GTEST_LIBRARIES})
add_test(NAME mapping.tests COMMAND mapping_tests)

