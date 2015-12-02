#include <merry_pcl/merry_pcl.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "merry_pcl_utils_test2"); //node name
    ros::NodeHandle nh;

	ROS_INFO("CONSTRUCTING START");
    // initialize merry_pcl object
    MerryPcl merry_pcl_utils(&nh);

 /*    ROS_INFO("CONSTRUCTING DONE");

    // get kinect_cloud
    while (!merry_pcl_utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_cloud = merry_pcl_utils.getKinectCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_color_cloud = merry_pcl_utils.getKinectColorCloud();

    // check color of each points
    merry_pcl_utils.read_points_color(kinect_color_cloud);

    ros::Duration(0.5).sleep(); // sleep for half a second
    ros::spinOnce();*/
}

