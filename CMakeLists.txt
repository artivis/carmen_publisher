cmake_minimum_required(VERSION 2.8.3)
project(carmen_publisher)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(PROGRAMS
scripts/carmen2rosbag.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
