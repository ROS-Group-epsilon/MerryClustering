#ifndef MERRY_PCL_UTILS_H_
#define MERRY_PCL_UTILS_H_

#include <ros/ros.h> //generic C++ stuff
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
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;

enum COLORS { RED, GREEN, BLUE, BLACK, WHITE, WOODCOLOR, NONE };

class MerryPclutils {
public:
	MerryPclutils(ros::NodeHandle* nodehandle);

	pcl::PointCloud<pcl::PointXYZ>::Ptr getKinectCloud() { return pclKinect_ptr_; };
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getKinectColorCloud() { return pclKinect_clr_ptr_; };

	void save_kinect_snapshot() { pcl::io::savePCDFileASCII ("kinect_snapshot.pcd", *pclKinect_ptr_); };
	void save_kinect_clr_snapshot() { pcl::io::savePCDFileASCII ("kinect_clr_snapshot.pcd", *pclKinect_clr_ptr_); };

 	Eigen::Vector3f get_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
	Eigen::Vector3f get_plane_normal(Eigen::MatrixXf points_mat);
	Eigen::Vector3f get_major_axis(Eigen::MatrixXf points_mat);

	int get_point_color(Eigen::Vector3d pt_color);
	void get_transformed_extracted_points(pcl::PointCloud<pcl::PointXYZ> & outputCloud);
	void get_general_purpose_cloud(pcl::PointCloud<pcl::PointXYZ> & outputCloud);

	bool got_kinect_cloud() { return got_kinect_cloud_; };
	bool got_extracted_points() {return got_extracted_points_;};


	float top_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud);
	Eigen::Vector3d find_avg_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr_);

private:
	// member variables
	ros::NodeHandle nh_;
    ros::Subscriber pointcloud_subscriber_; //use this to subscribe to a pointcloud topic
    //ros::Subscriber selected_points_subscriber_; // this to subscribe to "selectedPoints" topic from Rviz
    //ros::Publisher  pointcloud_publisher_;
    //ros::Publisher patch_publisher_;  

	bool got_kinect_cloud_;
    bool got_extracted_points_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect_ptr_; //(new PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformed_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclExtractedPoints_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformedExtractedPoints_ptr_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr_; //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclExtractedPtsClr_ptr_; //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclGenPurposeCloud_ptr_; 

	Eigen::Vector3d pt_color;
	Eigen::Vector3f plane_normal_, major_axis_, centroid_;

	// member functions
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();

    void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);

	Eigen::Vector3f compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
	void compute_plane_normal_and_major_axis(Eigen::MatrixXf points_mat);
	void compute_plane_normal_and_major_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr);

	bool isRed(int r, int g, int b, int tolerance);
	bool isGreen(int r, int g, int b, int tolerance);
	bool isBlue(int r, int g, int b, int tolerance);
	bool isBlack(int r, int g, int b, int tolerance);
	bool isWhite(int r, int g, int b, int tolerance);
	bool isWoodcolor(int r, int g, int b, int tolerance);

	void transform_cloud(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr);
	double distance_between(Eigen::Vector3f pt1, Eigen::Vector3f pt2);
	void extract_coplanar_pcl_operation();
};

#endif
