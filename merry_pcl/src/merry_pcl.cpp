#include <merry_pcl/merry_pcl.h>

MerryPcl::MerryPcl(ros::NodeHandle* nodehandle) : 
        nh_(*nodehandle),
        cwru_pcl_utils(nodehandle) {
    // to do or not to do
}

void CwruPclUtils::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");

}

//member helper function to set up publishers;

void CwruPclUtils::initializePublishers() {
    ROS_INFO("Initializing Publishers");

}

/* grasp kinect point cloud */
void MerryPcl::grasp_kinect_cloud() {
    while (!cwru_pcl_utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Merry got a pointcloud!");
    ROS_INFO("Merry is saving pointcloud ...");
    cwru_pcl_utils.save_kinect_snapshot();
    cwru_pcl_utils.save_kinect_clr_snapshot();  // save color version of pointcloud as well    
}

void MerryPcl::read_points_color() {
    /* read point color */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    while (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("kinect_clr_snapshot.pcd", *cloud) == -1) { //* load the file
        PCL_ERROR ("Couldn't read file kinect_clr_snapshot.pcd \n");
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    for (size_t i = 0; i < cloud->points.size (); ++i) {
        std::cout << " " << (int) cloud->points[i].r
                  << " " << (int) cloud->points[i].g
                  << " " << (int) cloud->points[i].b << std::endl;
    }
}

void MerryPcl::select_patch() {
    /* 2-dimensional points processing */ 
    //set up a publisher to display clouds in rviz:
    ros::Publisher pubCloud = nh_.advertise<sensor_msgs::PointCloud2> ("/merry_pcl_cloud_display", 1);
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
    ROS_INFO("selecing patch is done!");
}

void MerryPcl::select_colored_patch() {
    //set up a publisher to display clouds in rviz:
    ros::Publisher pubCloud = nh_.advertise<sensor_msgs::PointCloud2> ("/merry_colored_cloud_display", 1);
    //pcl::PointCloud<pcl::PointXYZ> & outputCloud
    pcl::PointCloud<pcl::PointXYZ> display_cloud; // instantiate a pointcloud object, which will be used for display in rviz
    pcl::PointCloud<pcl::PointXYZRGB> display_color_cloud; // instantiate a pointcloud object, which will be used for display in rviz
    
    sensor_msgs::PointCloud2 pcl2_display_cloud; //(new sensor_msgs::PointCloud2); //corresponding data type for ROS message

    tf::StampedTransform tf_sensor_frame_to_torso_frame; //use this to transform sensor frame to torso frame
    tf::TransformListener tf_listener; //start a transform listener

    //let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
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
    //convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_sensor_wrt_torso;
    A_sensor_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    //transform the kinect data to the torso frame;
    // we don't need to have it returned; cwru_pcl_utils can own it as a member var
    cwru_pcl_utils.transform_kinect_cloud(A_sensor_wrt_torso);
    //save this transformed data to disk:
    //cwru_pcl_utils.save_transformed_kinect_snapshot();
    
    Eigen::Vector3f plane_normal;
    double plane_dist;
    double z_eps = 0.005; //+/- 5mm tolerance
    double radius = 0.05; // try a 5cm radial search
    double radius_large = 0.5; //0.25; //radius for inclusion of points on a plane for color matching
    Eigen::Vector3f centroid;
    vector<int> selected_indices;
    vector<int> selected_indices_colormatch;
    Eigen::Vector3d avg_color,normalized_avg_color; 
    double color_match_thresh = 0.1;
    while (ros::ok()) {
        if (cwru_pcl_utils.got_selected_points()) {
            ROS_INFO("transforming selected points");
            cwru_pcl_utils.transform_selected_points_cloud(A_sensor_wrt_torso);

            //fit a plane to these selected points:
            ROS_INFO("fitting a plane to the transformed selected points:");
            cwru_pcl_utils.fit_xformed_selected_pts_to_plane(plane_normal, plane_dist);
            ROS_INFO_STREAM(" normal: " << plane_normal.transpose() << "; dist = " << plane_dist);
            centroid = cwru_pcl_utils.get_centroid();
            
            //operate on transformed kinect cloud:
            //extract indices of pts within +/- z_eps of height "plane_dist" from transformed 
            // kinect cloud AND within radius "radius" of "centroid";
            // get the indices of qualifying points
            ROS_INFO("getting indices of coplanar points within radius %f of patch centroid",radius);
            cwru_pcl_utils.filter_cloud_z(plane_dist, z_eps, radius,centroid,selected_indices);
            

            /* extract colored points */
            //copy indexed points from Kinect color pointer to display cloud, also in color
            // This provides a circle of points on plane of selected points
            // the circle of points, specified by "selected_indices", extracts colored points
            // from the original pointcloud (in Kinect sensor frame)
            cwru_pcl_utils.copy_indexed_pts_to_output_cloud(selected_indices,display_color_cloud);            
            // display (publish) the color cloud extracted from color Kinect cloud
            pcl::toROSMsg(display_color_cloud, pcl2_display_cloud); //convert datatype to compatible ROS message type for publication
            pcl2_display_cloud.header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain        
            pubCloud.publish(pcl2_display_cloud); //publish a point cloud that can be viewed in rviz (under topic pcl_cloud_display)

            // refer to the original colored Kinect pointcloud to get average color of points of interest
            ROS_INFO("computing average color of representative points...");
            avg_color = cwru_pcl_utils.find_avg_color_selected_pts(selected_indices);
            //normalize the color, to make it less sensitive to lighting
            // better would be to use HSV
            normalized_avg_color = avg_color/avg_color.norm();
            
            //expand the candidate list of points to a larger radius
            ROS_INFO("expanding search for coplanar points using radius= %f",radius_large);
            cwru_pcl_utils.filter_cloud_z(plane_dist, z_eps, radius_large,centroid,selected_indices);
            
            ROS_INFO("extracting points colored similar to patch average: ");
            cout<<"enter color threshold (0 to 1): ";
            cin>>color_match_thresh;

            cwru_pcl_utils.find_indices_color_match(selected_indices,
                    normalized_avg_color,color_match_thresh,selected_indices_colormatch);
            
            
            //copy indexed points from Kinect color pointer to display cloud, also in color
            // This provides a circle of points on plane of selected points
            // the circle of points, specified by "selected_indices", extracts colored points
            // from the original pointcloud (in Kinect sensor frame)
            cwru_pcl_utils.copy_indexed_pts_to_output_cloud(selected_indices_colormatch,display_color_cloud);
            
            cwru_pcl_utils.reset_got_selected_points();   // reset the selected-points trigger
        }

        // display (publish) the color cloud extracted from color Kinect cloud
        pcl::toROSMsg(display_color_cloud, pcl2_display_cloud); //convert datatype to compatible ROS message type for publication
        pcl2_display_cloud.header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain        
        pubCloud.publish(pcl2_display_cloud); //publish a point cloud that can be viewed in rviz (under topic pcl_cloud_display)

        ros::Duration(0.5).sleep(); // sleep for half a second
        ros::spinOnce();
    }
    ROS_INFO("selecting colored patch is done!");
}