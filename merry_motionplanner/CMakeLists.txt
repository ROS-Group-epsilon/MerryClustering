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

# Libraries
cs_add_library(merry_motionplanner src/merry_motionplanner.cpp)   

# Executables
cs_add_executable(merry_motionplanner_test1 src/merry_motionplanner_test1.cpp)
cs_add_executable(merry_motionplanner_test2 src/merry_motionplanner_test2.cpp)

# target_link_library(example my_lib)
target_link_libraries(merry_motionplanner_test1 merry_motionplanner ${catkin_LIBRARIES})
target_link_libraries(merry_motionplanner_test2 merry_motionplanner ${catkin_LIBRARIES})

cs_install()
cs_export()


