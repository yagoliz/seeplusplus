cmake_minimum_required(VERSION 2.8.3)
project(emotion_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package( )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/emotion_detect.py
  scripts/emotion_detect_average.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
