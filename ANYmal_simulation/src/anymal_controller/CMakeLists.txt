cmake_minimum_required(VERSION 3.5.1)
project(anymal_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  joint_state_controller
  effort_controllers
  anymal_b_simple_description
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
