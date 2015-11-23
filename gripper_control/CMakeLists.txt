cmake_minimum_required(VERSION 2.8.3)
project(gripper_control)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
cs_add_library(gripper_control src/gripper_control.cpp)

# Executables

# target_link_library(example my_lib)

cs_install()
cs_export()
    
