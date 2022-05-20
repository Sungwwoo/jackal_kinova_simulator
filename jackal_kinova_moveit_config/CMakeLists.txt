cmake_minimum_required(VERSION 3.0.2)
project(jackal_kinova_moveit_config)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

# This is added to remove policy CMP0054 warning (see https://stackoverflow.com/questions/45900159/how-to-use-variables-and-avoid-cmp0054-policy-violations)
cmake_policy(SET CMP0054 NEW)

find_package(catkin REQUIRED)

find_package(gazebo)

catkin_package()

# Install
install(PROGRAMS
   scripts/home_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
FILES_MATCHING PATTERN "*.launch")

include_directories(BEFORE ./include/)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")