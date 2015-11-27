cmake_minimum_required(VERSION 2.8.3)
project(merry_gripper)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
cs_add_library(merry_gripper src/merry_gripper.cpp)

# Executables
cs_add_executable(merry_gripper_main src/merry_gripper_main.cpp)

# target_link_library(example my_lib)
target_link_libraries(merry_gripper_main merry_gripper ${catkin_LIBRARIES})

cs_install()
cs_export()
    
