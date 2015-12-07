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
//#include <pcl/filters/crop_box.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;

enum COLORS { RED, GREEN, BLUE, BLACK, WHITE, WOODCOLOR, CANCOLOR, NONE };

class MerryPclutils {
public:
	MerryPclutils(ros::NodeHandle* nodehandle);

	void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();

	pcl::PointCloud<pcl::PointXYZ>::Ptr getKinectCloud() { return pclKinect_ptr_; };
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getKinectColorCloud() { return pclKinect_clr_ptr_; };
	pcl::PointCloud<pcl::PointXYZ>::Ptr getTransformedKinectCloud() { return pclTransformed_ptr_; };
	pcl::PointCloud<pcl::PointXYZ>::Ptr getGenPurposeCloud() { return pclGenPurposeCloud_ptr_; };

	void save_kinect_snapshot() { pcl::io::savePCDFileASCII ("kinect_snapshot.pcd", *pclKinect_ptr_); };
	void save_kinect_clr_snapshot() { pcl::io::savePCDFileASCII ("kinect_clr_snapshot.pcd", *pclKinect_clr_ptr_); };

	// used when output a plane point cloud
 	Eigen::Vector3f get_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
	Eigen::Vector3f get_plane_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
	Eigen::Vector3f get_major_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

	// output pcl cloud
	void get_transformed_extracted_points(pcl::PointCloud<pcl::PointXYZ> & outputCloud);
	void get_general_purpose_cloud(pcl::PointCloud<pcl::PointXYZ> & outputCloud);

	bool got_kinect_cloud() { return got_kinect_cloud_; };
	bool got_extracted_points() {return got_extracted_points_;};
	
	// transformation needed
	Eigen::Affine3f transformTFToEigen(const tf::Transform &t);
	void transform_kinect_cloud(Eigen::Affine3f A);
	void transform_selected_points_cloud(Eigen::Affine3f A);

	// most relevant to project MerryClustering
	Eigen::Vector3f get_top_point(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
	bool isBlockExist();
	void extract_coplanar_pcl_operation(Eigen::Vector3f centroid);
	int detect_color(Eigen::Vector3d pt_color);
	void find_indices_color_match(vector<int> &input_indices,
                    Eigen::Vector3d normalized_avg_color,
                    double color_match_thresh, vector<int> &output_indices);
	void find_coplanar_pts_z_height(double plane_height,double z_eps,vector<int> &indices);
	void filter_cloud_z(double z_nom, double z_eps, 
                double radius, Eigen::Vector3f centroid, vector<int> &indices);
	void filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, 
                double radius, Eigen::Vector3f centroid, vector<int> &indices);
	void filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, vector<int> &indices);
	Eigen::Vector3d find_avg_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr_);
	Eigen::Vector3d find_avg_color_selected_pts(vector<int> &indices);

private:
	// member variables
	ros::NodeHandle nh_;
    ros::Subscriber pointcloud_subscriber_; //use this to subscribe to a pointcloud topic
    ros::Subscriber extracted_points_subscriber_; // this to subscribe to "selectedPoints" topic from Rviz
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
    void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
	void extractCB(const sensor_msgs::PointCloud2ConstPtr& cloud);

	Eigen::Vector3f compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
	void compute_plane_normal_and_major_axis(Eigen::MatrixXf points_mat);
	void compute_plane_normal_and_major_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr);

	bool isRed(int r, int g, int b, int tolerance);
	bool isGreen(int r, int g, int b, int tolerance);
	bool isBlue(int r, int g, int b, int tolerance);
	bool isBlack(int r, int g, int b, int tolerance);
	bool isWhite(int r, int g, int b, int tolerance);
	bool isWoodcolor(int r, int g, int b, int tolerance);
	bool isCancolor(int r, int g, int b, int tolerance);

	void transform_cloud(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr);
	double distance_between(Eigen::Vector3f pt1, Eigen::Vector3f pt2);
	bool isWithinRadius(Eigen::Vector3f pt, Eigen::Vector3f centroid, double radius);
};

#endif
