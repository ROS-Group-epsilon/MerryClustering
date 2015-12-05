cmake_minimum_required(VERSION 2.8.3)
project(merry_pcl_utils)

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
cs_add_library(merry_pcl_utils src/merry_pcl_utils.cpp)
   

# Executables
cs_add_executable(merry_pcl_utils_main src/merry_pcl_utils_main.cpp)
cs_add_executable(merry_pcl_utils_test src/merry_pcl_utils_test.cpp)
cs_add_executable(test_frameid src/test_frameid.cpp)

#target_link_libraries(process_pcl ${PCL_LIBRARIES})
target_link_libraries(merry_pcl_utils_main merry_pcl_utils ${catkin_LIBRARIES})
target_link_libraries(merry_pcl_utils_test merry_pcl_utils ${catkin_LIBRARIES})

cs_install()
cs_export()
    
