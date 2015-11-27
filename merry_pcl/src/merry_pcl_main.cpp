// merry_pcl_main.cpp

// run this w/:
// roslaunch cwru_baxter_sim baxter_world.launch
// roslaunch cwru_baxter_sim kinect_xform.launch
// rosrun merry_pcl merry_pcl_main

#include <cwru_pcl_utils/cwru_pcl_utils.h>

using namespace std;

int main(int argc, char** argv) {

    ros::init(argc, argv, "merry_pcl_main"); //node name
    ros::NodeHandle nh;
    CwruPclUtils cwru_pcl_utils(&nh);



    
    /* grasp kinect point cloud */
    while (!cwru_pcl_utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Merry got a pointcloud!");
    ROS_INFO("Merry is saving pointcloud ...");
    cwru_pcl_utils.save_kinect_snapshot();
    cwru_pcl_utils.save_kinect_clr_snapshot();  // save color version of pointcloud as well




    /* read point color */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	while (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("kinect_clr_snapshot.pcd", *cloud) == -1) { //* load the file
		PCL_ERROR ("Couldn't read file kinect_clr_snapshot.pcd \n");
	}
	std::cout << "Loaded "
			  << cloud->width * cloud->height
		   	  << " data points from test_pcd.pcd with the following fields: "
			  << std::endl;

	for (size_t i = 0; i < cloud->points.size ()/10; ++i) {
		std::cout << " " << (int) cloud->points[i].r
				  << " " << (int) cloud->points[i].g
				  << " " << (int) cloud->points[i].b << std::endl;
	}






    /* 2-dimensional points processing */ 
    //set up a publisher to display clouds in rviz:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/merry_pcl_cloud_display", 1);
    pcl::PointCloud<pcl::PointXYZ> display_cloud; // instantiate a pointcloud object, which will be used for display in rviz
    sensor_msgs::PointCloud2 pcl2_display_cloud; //(new sensor_msgs::PointCloud2); //corresponding data type for ROS message

    /* transform listener */
    tf::StampedTransform tf_sensor_frame_to_torso_frame; //use this to transform sensor frame to torso frame
    tf::TransformListener tf_listener; //start a transform listener

    //let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso ...");
    while (tferr) {
        tferr = false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from sensor to world; ready to roll

    /* transform to the torso frame */
    //convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_sensor_wrt_torso;
    A_sensor_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    //transform the kinect data to the torso frame;
    // we don't need to have it returned; cwru_pcl_utils can own it as a member var
    cwru_pcl_utils.transform_kinect_cloud(A_sensor_wrt_torso);
    //save this transformed data to disk:
    cwru_pcl_utils.save_transformed_kinect_snapshot();
    
    Eigen::Vector3f plane_normal;
    double plane_dist;
    while (ros::ok()) {

    	/* select points */
        if (cwru_pcl_utils.got_selected_points()) {
            ROS_INFO("transforming selected points");
            cwru_pcl_utils.transform_selected_points_cloud(A_sensor_wrt_torso);

            //fit a plane to these selected points:
            cwru_pcl_utils.fit_xformed_selected_pts_to_plane(plane_normal, plane_dist);
            ROS_INFO_STREAM(" normal: " << plane_normal.transpose() << "; dist = " << plane_dist);
           
            //here is a function to get a copy of the transformed, selected points;
            //cwru_pcl_utils.get_transformed_selected_points(display_cloud);
            //alternative: compute and get offset points from selected, transformed points
            cwru_pcl_utils.example_pcl_operation(); // offset the transformed, selected points and put result in gen-purpose object
            cwru_pcl_utils.get_gen_purpose_cloud(display_cloud);
            cwru_pcl_utils.reset_got_selected_points();   // reset the selected-points trigger
        }

        pcl::toROSMsg(display_cloud, pcl2_display_cloud); //convert datatype to compatible ROS message type for publication
        pcl2_display_cloud.header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain        
        pubCloud.publish(pcl2_display_cloud); //publish a point cloud that can be viewed in rviz (under topic pcl_cloud_display)

        ros::Duration(0.5).sleep(); // sleep for half a second
        ros::spinOnce();
    }
    ROS_INFO("my work here is done!");

}

