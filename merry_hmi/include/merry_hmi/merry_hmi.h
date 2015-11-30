#ifndef MERRY_HMI_H
#define MERRY_HMI_H

#include <ros/ros.h> 
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <sensor_msgs/PointCloud2.h> //useful ROS message types
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <cwru_msgs/PatchParams.h>

#include <tf/transform_listener.h>  // transform listener headers
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>  //point-cloud library headers; likely don't need all these
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>

#include <cwru_pcl_utils/cwru_pcl_utils.h>

class merry_hmi{
public:
	merry_hmi(ros::NodeHandle* nodehandle);
	bool isObstructed();
private:
	void updateKinectCB(const sensor_msgs::PointCloud2 & message_holder);
	ros::NodeHandle nh_;
	ros::Subscriber cloudSub_;
	bool obstructed_;

};

#endif