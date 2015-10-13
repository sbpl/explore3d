cmake_minimum_required(VERSION 2.8.3)
project(explore3d)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    COMPONENTS 
        nav_msgs 
        pcl_ros 
        roscpp 
        rospy 
        sensor_msgs 
        std_msgs 
        std_srvs 
        utacc_msgs)

find_package(sbpl REQUIRED)
set(sbpl_INCLUDE_DIRS ${SBPL_INCLUDE_DIRS})
set(sbpl_LIBRARIES ${SBPL_LIBRARIES})

find_package(PCL REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        explore3d
    CATKIN_DEPENDS
        nav_msgs
        pcl_ros
        roscpp
        sensor_msgs
        tf
    DEPENDS
        PCL
        sbpl)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(
    explore3d
    src/coverage_map.cpp
    src/exploration_planner.cpp
    src/exploration_thread.cpp
    src/exploration_structs.cpp)

target_link_libraries(explore3d ${SBPL_LIBRARIES} ${PCL_LIBRARIES})
