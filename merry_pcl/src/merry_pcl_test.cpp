// merry_pcl_main.cpp

// run this w/:
// roslaunch cwru_baxter_sim baxter_world.launch
// roslaunch cwru_baxter_sim kinect_xform.launch
// rosrun merry_pcl merry_pcl_main

#include <merry_pcl/merry_pcl.h>

using namespace std;

int main(int argc, char** argv) {

    ros::init(argc, argv, "merry_pcl_main"); //node name
    ros::NodeHandle nh;
    MerryPcl merry_pcl(&nh);

    merry_pcl.grasp_kinect_cloud();
    //merry_pcl.read_points_color();
    //merry_pcl.select_patch();
    merry_pcl.select_colored_patch();

    ROS_INFO("my work here is done!");
}

