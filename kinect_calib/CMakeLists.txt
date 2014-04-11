cmake_minimum_required(VERSION 2.8.3)
project(kinect_calib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinect_calib
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
