cmake_minimum_required(VERSION 3.0.2)
project(event_callback)

## Compile as C++17 (required by core.hpp: std::any, std::type_index, if constexpr)
add_compile_options(-std=c++17)

###################################
## Find catkin macros and libraries
###################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  event_callback_msg
)

###################################
## catkin specific configuration ##
###################################
catkin_python_setup()

catkin_package(
  INCLUDE_DIRS src src/core src/utils
  CATKIN_DEPENDS roscpp rospy std_msgs event_callback_msg
)

###########
## Build ##
###########
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Install C++ header-only library
install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h"
)
