#include <merry_pcl_utils/merry_pcl_utils.h>



void check_color_of_each_point(MerryPclutils merry_pcl_utils, pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud) {
    // check color of each points
    Eigen::Vector3d pt_color;
    int color;
    for (size_t i = 0; i < color_cloud->points.size (); ++i) {
        // std::cout << " " << (int) color_cloud->points[i].r
        //           << " " << (int) color_cloud->points[i].g
        //           << " " << (int) kinect_color_cloud->points[i].b << std::endl;
        pt_color[0] = (int) color_cloud->points[i].r;
        pt_color[1] = (int) color_cloud->points[i].g;
        pt_color[2] = (int) color_cloud->points[i].b;
        color = merry_pcl_utils.detect_color(pt_color);
        //cout << color << endl;
    }    
}


void check_height_of_each_point(MerryPclutils merry_pcl_utils, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // check color of each points
    Eigen::Vector3f pt;
    cout << "============================ check_height_of_each_point ================================" << endl;
    cout << "points num = " << cloud->points.size() << endl;
    for (size_t i = 0; i < cloud->points.size (); ++i) {
        cout << i << endl;
        std::cout << " " << (double) cloud->points[i].x
                  << " " << (double) cloud->points[i].y
                  << " " << (double) cloud->points[i].z << std::endl;
    }    
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "merry_pcl_utils_test"); //node name
    ros::NodeHandle nh;

	
    // initialize merry_pcl object
    ROS_INFO("CONSTRUCTING START");
    MerryPclutils merry_pcl_utils(&nh);
    ROS_INFO("CONSTRUCTING DONE");


    // initialize variables
    // these are waht we want for motion planning using
    Eigen::Vector3f plane_normal, major_axis, centroid;

    // initialize cloud to display in rviz
    pcl::PointCloud<pcl::PointXYZ> DisplayCloud;
    sensor_msgs::PointCloud2 pcl2_DisplayCloud; //(new sensor_msgs::PointCloud2); //corresponding data type for ROS message
    // import kinect cloud, both uncolored and colored
    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_cloud = merry_pcl_utils.getKinectCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_color_cloud = merry_pcl_utils.getKinectColorCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_kinect_cloud, extracted_plane;
    
    // set up a publisher to display clouds in rviz:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/extracted_points", 1);

    // for color detect and match
    vector<int> selected_indices;
    Eigen::Vector3d avg_color;
    int Avg_Color;

    // get kinect_cloud
    while (!merry_pcl_utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Merry got a pointcloud!");
    ROS_INFO("Merry is saving pointcloud ...");
    // merry_pcl_utils.save_kinect_snapshot();
    // merry_pcl_utils.save_kinect_clr_snapshot();  // save color version of pointcloud as well
    // while (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("kinect_clr_snapshot.pcd", *kinect_color_cloud) == -1) { //* load the file
    //     PCL_ERROR ("Couldn't read file kinect_clr_snapshot.pcd \n");
    // }


    // transformation
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
    A_sensor_wrt_torso = merry_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    // get transform_kinect_cloud
    merry_pcl_utils.transform_kinect_cloud(A_sensor_wrt_torso);
    transformed_kinect_cloud = merry_pcl_utils.getTransformedKinectCloud();


    // get block height and determin a initial point to extract
    Eigen::Vector3f init_pt = merry_pcl_utils.get_top_point(transformed_kinect_cloud);
    ROS_INFO( "=======================found init_point ...============================" );
    ROS_INFO("init point x: %f, y: %f,z: %f",init_pt[0],init_pt[1],init_pt[2]);

    //check_height_of_each_point(merry_pcl_utils, transformed_kinect_cloud);

    // extract top plane of the block and store it in pclGenPurposeCloud_ptr_ which is private
    merry_pcl_utils.extract_coplanar_pcl_operation(init_pt);
    // output it to DisplayCloud
    merry_pcl_utils.get_general_purpose_cloud(DisplayCloud);
    extracted_plane = merry_pcl_utils.getGenPurposeCloud();
    //check_height_of_each_point(merry_pcl_utils, extracted_plane);


    // compute the three vectors and plane_dist
    plane_normal = merry_pcl_utils.get_plane_normal(extracted_plane);
    major_axis = plane_normal;
    centroid = merry_pcl_utils.get_centroid(extracted_plane);
    double plane_dist = plane_normal.dot(centroid);
    ROS_INFO_STREAM(" normal: " << plane_normal.transpose() << "; dist = " << plane_dist);


    // match colored points in kinect colored cloud
    double z_eps = 0.005; //+/- 5mm tolerance
    double radius = 0.5; // try a 5cm radial search
    //operate on transformed kinect cloud:
    //extract indices of pts within +/- z_eps of height "plane_dist" from transformed 
    // kinect cloud AND within radius "radius" of "centroid";
    // get the indices of qualifying points
    ROS_INFO("getting indices of coplanar points within radius %f of patch centroid",radius);
    merry_pcl_utils.filter_cloud_z(plane_dist, z_eps, radius, centroid, selected_indices);


    // refer to the original colored Kinect pointcloud to get average color of points of interest
    ROS_INFO("computing average color of representative points...");
    avg_color = merry_pcl_utils.find_avg_color_selected_pts(selected_indices);
    Avg_Color = merry_pcl_utils.detect_color(avg_color);
    ROS_INFO("detect color as %d ", Avg_Color);


    // publish cloud
	while(1) {
	    pcl::toROSMsg(DisplayCloud, pcl2_DisplayCloud); //convert datatype to compatible ROS message type for publication
	    pcl2_DisplayCloud.header.frame_id = "torso";
	    pcl2_DisplayCloud.header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain        
	    pubCloud.publish(pcl2_DisplayCloud);

	    ros::Duration(0.5).sleep(); // sleep for half a second
	    ros::spinOnce();
	}
}
