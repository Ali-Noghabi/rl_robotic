cmake_minimum_required(VERSION 3.5.1)
project(anymal_joystick)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  joy
  anymal_controller
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
