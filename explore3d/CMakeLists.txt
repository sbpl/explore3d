cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(ROS_COMPILE_FLAGS "-std=c++0x -Wall -pthread -O3") #-flto -Ofast -pedantic")
set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PkgConfig REQUIRED)
find_package(sbpl REQUIRED)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

rosbuild_add_library(
    explore3d
    src/coverage_map.cpp
    src/exploration_planner.cpp
    src/exploration_thread.cpp
    src/exploration_structs.cpp
    src/raycast.cpp)

target_link_libraries(${PROJECT_NAME} ${SBPL_LIBRARIES})
