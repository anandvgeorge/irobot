cmake_minimum_required(VERSION 2.8.3)
project(irobot_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED Boost REQUIRED COMPONENTS system)
# find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES irobot_plugin
 CATKIN_DEPENDS 
 gazebo_ros
 roscpp
 geometry_msgs
 nav_msgs
 sensor_msgs
 
 DEPENDS 
 roscpp 
 gazebo_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(pid_diff_drive src/pid_diff_drive.cpp)
target_link_libraries(pid_diff_drive ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(test_plugin src/test_plugin.cpp)
target_link_libraries(test_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS
  pid_diff_drive
  test_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)