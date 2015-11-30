#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/PCLPointCloud2.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>

#ifndef PCL_BLOCK_SORTER_H_
#define PCL_BLOCK_SORTER_H_

class block_sorter{
public:
	block_sorter(ros::NodeHandle* nodehandle);
	void top_height();
	void computes_centroid();
	void find_avg_color();
	void color_detection();
private:
	ros::NodeHandle nh_;
	ros::Publisher  PCLPub_;
};

#endif