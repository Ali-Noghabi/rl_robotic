cmake_minimum_required(VERSION 3.5.1)
project(anymal_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  controller_manager
  anymal_b_simple_description
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
