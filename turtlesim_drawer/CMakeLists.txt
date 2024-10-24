cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_drawer)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  turtlesim
)

## Declare a catkin package
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES turtlesim_drawer
  CATKIN_DEPENDS rospy std_msgs geometry_msgs turtlesim
  #  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/draw_shapes.py
  scripts/teleop_keyboard.py  # Include this line only if you have teleop_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install configuration files (if any)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
