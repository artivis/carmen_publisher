cmake_minimum_required(VERSION 3.0.2)
project(carmen_publisher)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
    scripts/carmen2rosbag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
