# General info
cmake_minimum_required(VERSION 3.0.2)
project(herohernan-ros-templates-cpp)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

###########
## Nodes ##
###########

# 1. arduino_volt2percent
add_executable(arduino_volt2percent src/Arduino/arduino_volt2percent.cpp)
target_link_libraries(arduino_volt2percent ${catkin_LIBRARIES})

# 2. arduino_volt2temp
add_executable(arduino_volt2temp src/Arduino/arduino_volt2temp.cpp)
target_link_libraries(arduino_volt2temp ${catkin_LIBRARIES})
