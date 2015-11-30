cmake_minimum_required(VERSION 2.8.3)
project(merry_motionplanner)

find_package(catkin_simple REQUIRED)

catkin_simple()
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# TODO do I need this? ^^


# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
cs_add_library(merry_motionplanner src/merry_motionplanner.cpp)   

# Executables
cs_add_executable(merry_motionplanner_main src/merry_motionplanner_main.cpp)

target_link_libraries(merry_motionplanner_main merry_motionplanner ${catkin_LIBRARIES})
# TODO make sure you link with all necessary libraries!

cs_install()
cs_export()