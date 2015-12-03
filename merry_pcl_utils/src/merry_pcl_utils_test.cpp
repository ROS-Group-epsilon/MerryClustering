#include <merry_pcl_utils/merry_pcl_utils.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "merry_pcl_utils_test"); //node name
    ros::NodeHandle nh;

	ROS_INFO("CONSTRUCTING START");
    // initialize merry_pcl object
    MerryPclutils merry_pcl_utils(&nh);

    ROS_INFO("CONSTRUCTING DONE");

    // get kinect_cloud
    while (!merry_pcl_utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Merry got a pointcloud!");
    ROS_INFO("Merry is saving pointcloud ...");
    merry_pcl_utils.save_kinect_snapshot();
    merry_pcl_utils.save_kinect_clr_snapshot();  // save color version of pointcloud as well

    // read and import the cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_cloud = merry_pcl_utils.getKinectCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_color_cloud = merry_pcl_utils.getKinectColorCloud();

    while (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("kinect_clr_snapshot.pcd", *kinect_color_cloud) == -1) { //* load the file
        PCL_ERROR ("Couldn't read file kinect_clr_snapshot.pcd \n");
    }

    // check color of each points
    Eigen::Vector3d pt_color;
    int color;
    for (size_t i = 0; i < kinect_color_cloud->points.size (); ++i) {
        // std::cout << " " << (int) kinect_color_cloud->points[i].r
        //           << " " << (int) kinect_color_cloud->points[i].g
        //           << " " << (int) kinect_color_cloud->points[i].b << std::endl;

        pt_color[0] = (int) kinect_color_cloud->points[i].r;
        pt_color[1] = (int) kinect_color_cloud->points[i].g;
        pt_color[2] = (int) kinect_color_cloud->points[i].b;
        color = merry_pcl_utils.get_point_color(pt_color);
        //cout << color << endl;
    }

    ros::Duration(0.5).sleep(); // sleep for half a second
    ros::spinOnce();

}
