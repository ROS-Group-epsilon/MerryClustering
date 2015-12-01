cmake_minimum_required(VERSION 2.8.3)
project(block_sorter)

find_package(catkin_simple REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#looks like Eigen3 works, but Eigen does not!
find_package(Eigen3 REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
cs_add_library(block_sorter src/block_sorter.cpp)
   

# Executables
cs_add_executable(block_sorter_main src/block_sorter_main.cpp)


#target_link_libraries(process_pcl ${PCL_LIBRARIES})
target_link_libraries(block_sorter_main block_sorter ${catkin_LIBRARIES})
cs_install()
cs_export()
    
