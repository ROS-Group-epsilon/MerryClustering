#ifndef BLOCK_SORTER_H_
#define BLOCK_SORTER_H_

#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
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

class BlockSorter {
public:
	BlockSorter(ros::NodeHandle* nodehandle);
	float top_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud);
	Eigen::Vector3f computes_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
	Eigen::Vector3f m_axis(Eigen::MatrixXf points_mat);
	Eigen::Vector3d find_avg_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr_);
	int color_detection(Eigen::Vector3d pt_color);

private:
	ros::NodeHandle nh_;
	Eigen::Vector3d pt_color;
	Eigen::Vector3f major_axis_,centroid;
};

#endif
