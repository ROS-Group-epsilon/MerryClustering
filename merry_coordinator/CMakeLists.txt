cmake_minimum_required(VERSION 2.8.3)
project(merry_coordinator)

find_package(catkin_simple REQUIRED)

catkin_simple()

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread) 

# Executables
cs_add_executable(merry_coordinator src/merry_coordinator.cpp)

# target_link_libraries
target_link_libraries(merry_coordinator merry_motionplanner ${catkin_LIBRARIES})
target_link_libraries(merry_coordinator merry_gripper ${catkin_LIBRARIES})
target_link_libraries(merry_coordinator merry_pcl_utils ${catkin_LIBRARIES})
# target_link_libraries(merry_coordinator merry_hmi ${catkin_LIBRARIES})


cs_install()
cs_export()
