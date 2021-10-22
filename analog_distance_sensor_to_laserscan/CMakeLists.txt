cmake_minimum_required(VERSION 3.0.2)
project(analog_distance_sensor_to_laserscan)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++14 -O3 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nodelet
  pluginlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########
add_library(analog_distance_sensor_to_laserscan
  src/analog_distance_sensor_to_laserscan_node.cpp
)

target_link_libraries(analog_distance_sensor_to_laserscan
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(FILES analog_distance_sensor_to_laserscan.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)