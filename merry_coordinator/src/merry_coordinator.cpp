#include <merry_gripper/merry_gripper.h>
#include <merry_pcl_utils_utils/merry_pcl_utils_utils.h>
#include <merry_hmi/merry_hmi.h>
#include <merry_motionplanner/merry_motionplanner.h>


typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;


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


// match colored points in kinect colored cloud
int determin_block_color(MerryPclutils merry_pcl_utils, double height, Eigen::Vector3f centroid) {
    Eigen::Vector3d avg_rgb_color;
    int block_color;

    // give initial search radius
    double z_eps = 0.001; //+/- 5mm tolerance
    double radius = 0.05; // try a 5cm radial search
    vector<int> selected_indices;

    // operate on transformed kinect cloud:
    // extract indices of pts within +/- z_eps of height "plane_dist" from transformed 
    // kinect cloud AND within radius "radius" of "centroid";
    // get the indices of qualifying points
    //ROS_INFO("getting indices of coplanar points within radius %f of patch centroid",radius);
    merry_pcl_utils.filter_cloud_z(height, z_eps, radius, centroid, selected_indices);

    // refer to the original colored Kinect pointcloud to get average color of points of interest
    ROS_INFO("computing average color of representative points...");
    avg_rgb_color = merry_pcl_utils.find_avg_color_selected_pts(selected_indices);
    block_color = merry_pcl_utils.detect_color(avg_rgb_color);
    int count = 0;
    while(block_color == 7 && count++ < 30) {
        z_eps = z_eps*1.1;
        radius = radius*0.9;
        merry_pcl_utils.filter_cloud_z(init_pt[2], z_eps, radius, centroid, selected_indices);
        ROS_INFO("computing average color of representative points...");
        avg_rgb_color = merry_pcl_utils.find_avg_color_selected_pts(selected_indices);
        block_color = merry_pcl_utils.detect_color(avg_rgb_color);
    }
    ROS_INFO("detect color as %d ", block_color);
    return block_color;
}



int main(int argc, char** argv) {

	ros::init(argc, argv, "merry_coordinator");
	ros::NodeHandle nh;


	ROS_INFO("created node and nodehandle");
	MerryGripper gripper(&nh);
	MerryMotionplanner motionplanner(&nh);
	MerryPclutils merry_pcl_utils(&nh);
	//merry_hmi merry_hmi(&nh);
	ROS_INFO("created instances of each library");


	ROS_INFO("attempting to get kinect cloud");
	while(!merry_pcl_utils.got_kinect_cloud()) {
		ROS_INFO("did not receive pointcloud");
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}
	ROS_INFO("got a pointcloud");


    // initialize variables
    // these are waht we want for motion planning using
    Eigen::Vector3f plane_normal, major_axis, centroid;


    // initialize cloud to display in rviz
    pcl::PointCloud<pcl::PointXYZ> DisplayCloud;
    sensor_msgs::PointCloud2 pcl2_DisplayCloud; //(new sensor_msgs::PointCloud2); //corresponding data type for ROS message
    // import kinect cloud, both uncolored and colored
    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_color_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_kinect_cloud, extracted_plane;
    
    // set up a publisher to display clouds in rviz:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/extracted_points", 1);

    // for color detect and match

    Eigen::Vector3d avg_rgb_color;
    int block_color;

    // get kinect_cloud
    while (!merry_pcl_utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Merry got a pointcloud!");

    // transformation
	// use this to transform sensor frame to torso frame
	tf::StampedTransform tf_sensor_frame_to_torso_frame;

	// start a transform listener and warm it up
	tf::TransformListener tf_listener;
	bool tferr = true;
	ROS_INFO("waiting for tf between kinect_pc_frame/camera_rgb_optical_frame and torso...");
	while(tferr) {
		tferr = false;
		try {
			//tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
			tf_listener.lookupTransform("torso", "camera_rgb_optical_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
		} catch(tf::TransformException &exception) {
			ROS_ERROR("%s", exception.what());
			tferr = true;
			ros::Duration(0.5).sleep();
			ros::spinOnce();
		}
	}
	// tf listener has now found a complete chain from sensor to world
	ROS_INFO("tf is good");

	// convert the tf to an Eigen::Affine
	Eigen::Affine3f A_sensor_wrt_torso;
	Eigen::Affine3d Affine_des_gripper;
	Eigen::Vector3d origin_des;
	geometry_msgs::PoseStamped rt_tool_pose;
	A_sensor_wrt_torso = merry_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
	
	Eigen::Vector3f plane_normal, major_axis, centroid;
	Eigen::Vector3f allzeroVector(0, 0, 0);
	Eigen::Matrix3d Rmat;
	int rtn_val;
	double plane_dist;
	Eigen::Vector3f init_pt;

	// move to predefined pose
    ros::Time begin = ros::Time::now();
    ros::Time current;
    while(current.toSec() - begin.toSec() < 0.5) {
		// send a command to plan a joint space move to predefined pose and then execute that plan
		rtn_val = motionplanner.plan_move_to_pre_pose();
		if(rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
			rtn_val = motionplanner.rt_arm_execute_planned_path();
		}
        ros::Duration(0.05).sleep();
        ros::spinOnce();
        current = ros::Time::now();
    }


	while(ros::ok()) {
		// regain a new kinect cloud
		merry_pcl_utils.merry_pcl_utils(&nh);

		ROS_INFO("attempting to get kinect cloud");
		while(!merry_pcl_utils.got_kinect_cloud()) {
			ROS_INFO("did not receive pointcloud");
			ros::spinOnce();
			ros::Duration(1.0).sleep();
		}
		ROS_INFO("got a pointcloud");

		// the following uses the MerryPclutils library in order to get the centroid, major axis, and plane normal
		kinect_cloud = merry_pcl_utils.getKinectCloud(); // TODO check if this line is needed
		kinect_color_cloud = merry_pcl_utils.getKinectColorCloud(); // TODO check if this line is needed

		merry_pcl_utils.transform_kinect_cloud(A_sensor_wrt_torso);
		transformed_kinect_cloud = merry_pcl_utils.getTransformedKinectCloud();
		init_pt = merry_pcl_utils.get_top_point(transformed_kinect_cloud);


		if(merry_pcl_utils.isBlockExist() /* && !merry_hmi.isObstructed() */) {
			ROS_INFO("block has been found");

			merry_pcl_utils.extract_coplanar_pcl_operation(init_pt);
			merry_pcl_utils.get_general_purpose_cloud(display_cloud);
			extracted_plane = merry_pcl_utils.getGenPurposeCloud();

			// detemine what the centroid, major axis, and plane normal are
			centroid = merry_pcl_utils.get_centroid(extracted_plane);
			major_axis = merry_pcl_utils.get_major_axis(extracted_plane);
			plane_normal = merry_pcl_utils.get_plane_normal(extracted_plane);
			ROS_INFO("Found the centroid, major axis, and plane normal!");
			if(plane_normal == allzeroVector) break;

            //input: centroid, plane_normal, major_axis
            //output: Rmat, origin_des
            origin_des = merry_motionplanner.compute_origin_des(centroid);
            Rmat = merry_motionplanner.compute_orientation(plane_normal, major_axis);

            //construct a goal affine pose:
            Affine_des_gripper = merry_motionplanner.construct_affine_pose(Rmat, origin_des);

			rt_tool_pose.pose = motionplanner.transformEigenAffine3dToPose(Affine_des_gripper);

			// plan path to goal destination and execute path if plan is successful
			rtn_val = motionplanner.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
			if(rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
				rtn_val = motionplanner.rt_arm_execute_planned_path();
				ros::Duration(0.5).sleep();
			} else {
				ROS_WARN("Cartesian path to desired pose is not achievable.");
				ros::Duration(1).sleep();
				break;
			}

			gripper.grasp();

			// the following uses the MerryPclutils library in order to determine what color the block is
			//Eigen::VectorXd q_vec_pose;
			Vectorq7x1 q_vec_pose;
			vector<int> selected_indices;
			Eigen::Vector3d avg_color;

			plane_dist = plane_normal.dot(centroid);
			double z_eps = 0.005;
			double radius = 0.5;
			merry_pcl_utils.filter_cloud_z(plane_dist, z_eps, radius, centroid, selected_indices);
			avg_color = merry_pcl_utils.find_avg_color_selected_pts(selected_indices);
			ROS_INFO_STREAM("r: " << avg_color[0] << " g: " << avg_color[1] << " b: " << avg_color[2] << "\n");
			
			int color = merry_pcl_utils.detect_color(avg_color);
			//std::cin>>color;
			// depending on color of block, will assign a different goal destination
			if(color == 0 /*MerryPclutils::COLORS::RED*/) {
				q_vec_pose << 0.8, -0.3, 0, 1, 0, 0.8, 0; //move to center

			} else if(color == 1 /*MerryPclutils::COLORS::GREEN*/) {
				q_vec_pose << 1.2, -0.2, 0, 0.8, 0, 0.8, 0; //move to left

			} else if(color == 2 /*MerryPclutils::COLORS::BLUE*/) {
				q_vec_pose << 0.4, -0.2, 0, 0.8, 0, 0.8, 0; //move to right

			} else if(color == 3 /*MerryPclutils::COLORS::BLACK*/) {
				q_vec_pose << 0.8, -0.6, 0, 2, 0, 0.6, 0; //move to center behind

			} else if(color == 4 /*MerryPclutils::COLORS::WHITE*/) {
				q_vec_pose << 1.2, -0.4, 0, 1.5, 0.6, 1, 0; //move to left behind

			} else if(color == 5 /*MerryPclutils::COLORS::WOODCOLOR*/) {
				q_vec_pose << 0.4, -0.4, 0, 1.5, -0.6, 1, 0; //move to right behind
			} else if (color == 6){
				q_vec_pose <<  1.2, -0.2, 0, 0.8, 0, 0.8, 0;
			} else {
				ROS_WARN("Color of block could not be determined.");
				ros::Duration(0.5).sleep();
				break;
			}
			
			// plan path to goal destination that was determined above
			// execute motion if path was determined to be successful
			rtn_val = motionplanner.rt_arm_plan_jspace_path_current_to_qgoal(q_vec_pose);
			if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
				rtn_val = motionplanner.rt_arm_execute_planned_path();
				ros::Duration(0.5).sleep();
			} else {
				ROS_WARN("Joint space path to desired pose is not achievable.");
				ros::Duration(1).sleep();
				break;
			}

			gripper.release();

			// go back to pre pose
			rtn_val = motionplanner.plan_move_to_pre_pose();
			if(rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
				rtn_val = motionplanner.rt_arm_execute_planned_path();
				ros::Duration(0.5).sleep();
			} else {
				ROS_WARN("Move to pre pose is not achievable.");
				break;
			}
		} else {
			ROS_WARN("Don't kidding me. There is no block on the table!");
		}

		ros::Duration(0.5).sleep();
		ros::spinOnce();
    }

	return 1;
}