cmake_minimum_required(VERSION 3.0.2)
project(tag_listener)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set( sources
  src/tag_listener.cpp
)

add_executable(tag_listener ${sources})
target_link_libraries(tag_listener ${catkin_LIBRARIES})