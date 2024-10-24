cmake_minimum_required(VERSION 3.0.2)
project(quadruped_analysis)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

# Declare a catkin package
catkin_package()

# Specify the locations of Python scripts to be installed
catkin_install_python(PROGRAMS
  scripts/quadruped_simulation.py
  scripts/gait_analysis_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Add launch file installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
