#ifndef MERRY_PCL_H_
#define MERRY_PCL_H_

#include <cwru_pcl_utils/cwru_pcl_utils.h>

class MerryPcl
{
public:
    MerryPcl(ros::NodeHandle* nodehandle); //constructor
    void grasp_kinect_cloud();
    void read_points_color();
    pcl::PointCloud<pcl::PointXYZ>::Ptr import_point_cloud();
    Eigen::Vector3f compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void select_patch();
    void select_colored_patch();

private:
    ros::NodeHandle nh_;
    CwruPclUtils cwru_pcl_utils;

    void initializeSubscribers();
    void initializePublishers();
};

#endif
