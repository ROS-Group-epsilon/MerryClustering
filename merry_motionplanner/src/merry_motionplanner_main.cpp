#include <merry_gripper/merry_gripper.h>
#include <merry_pcl_utils/merry_pcl_utils.h>
//#include <merry_hmi/merry_hmi.h>
#include <merry_motionplanner/merry_motionplanner.h>
#include <cwru_pcl_utils/cwru_pcl_utils.h>

typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;

int main(int argc, char** argv) {
	ROS_INFO("began running main method");
	ros::init(argc, argv, "merry_motionplanner_action_client");
	ros::NodeHandle nh;

	ROS_INFO("created node and nodehandle");

	MerryGripper gripper(&nh);
	MerryMotionplanner motionplanner(&nh);
	MerryPclutils merry_pcl(&nh);
	//merry_hmi merry_hmi(&nh);
	CwruPclUtils cwru_pcl_utils(&nh);
	ROS_INFO("created instances of each library");

	ROS_INFO("attempting to get kinect cloud");
	while(!cwru_pcl_utils.got_kinect_cloud()) {
		ROS_INFO("did not receive pointcloud");
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}
	ROS_INFO("got a pointcloud");



	// use this to transform sensor frame to torso frame
	tf::StampedTransform tf_sensor_frame_to_torso_frame;

	// start a transform listener and warm it up
	tf::TransformListener tf_listener;
	bool tferr = true;
	ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
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
	Eigen::Vector3d xvec_des, yvec_des, zvec_des, origin_des;
	geometry_msgs::PoseStamped rt_tool_pose;

	A_sensor_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
	Eigen::Vector3f plane_normal, major_axis, centroid;
	Eigen::Matrix3d Rmat;
	int rtn_val;
	double plane_dist;

	// send a command to plan a joint space move to predefined pose and then execute that plan
	rtn_val = motionplanner.plan_move_to_pre_pose();
	if(rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
		rtn_val = motionplanner.rt_arm_execute_planned_path();
	}

	while(ros::ok()) {
		// the following uses the MerryPclutils library in order to get the centroid, major axis, and plane normal
		pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_cloud = merry_pcl.getKinectCloud(); // TODO check if this line is needed
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_color_cloud = merry_pcl.getKinectColorCloud(); // TODO check if this line is needed
		pcl::PointCloud<pcl::PointXYZ> display_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_kinect_cloud, extracted_plane;

		merry_pcl.transform_kinect_cloud(A_sensor_wrt_torso);
		transformed_kinect_cloud = merry_pcl.getTransformedKinectCloud();
		Eigen::Vector3f init_pt = merry_pcl.get_top_point(transformed_kinect_cloud);


		if(merry_pcl.isBlockExist() /* && !merry_hmi.isObstructed() */) {
			ROS_INFO("block has been found");

			merry_pcl.extract_coplanar_pcl_operation(init_pt);
			merry_pcl.get_general_purpose_cloud(display_cloud);
			extracted_plane = merry_pcl.getGenPurposeCloud();

			// detemine what the centroid, major axis, and plane normal are
			centroid = merry_pcl.get_centroid(extracted_plane);
			major_axis = merry_pcl.get_major_axis(extracted_plane);
			plane_normal = merry_pcl.get_plane_normal(extracted_plane);
			ROS_INFO("Found the centroid, major axis, and plane normal!");

			// use the centroid, major axis, and plane normal to determine a goal destination for right arm
			for(int i = 0; i < 3; i++) {
				origin_des[i] = centroid[i];
				zvec_des[i] = -plane_normal[i]; // want tool to point opposite to surface normal
				xvec_des[i] = major_axis[i];
			}

			origin_des[2] += 0.02; // raise up hand by 2cm

			yvec_des = zvec_des.cross(xvec_des); // construct consistent right-hand triad

			Rmat.col(0) = xvec_des;
			Rmat.col(1) = yvec_des;
			Rmat.col(2) = zvec_des;
			Affine_des_gripper.linear() = Rmat;
			Affine_des_gripper.translation() = origin_des;
			std::cout<<"orientation: "<<std::endl;
            std::cout<<Rmat<<std::endl;

			rt_tool_pose.pose = motionplanner.transformEigenAffine3dToPose(Affine_des_gripper);

			// plan path to goal destination and execute path if plan is successful
			rtn_val = motionplanner.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
			if(rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
				rtn_val = motionplanner.rt_arm_execute_planned_path();
			} else {
				ROS_WARN("Cartesian path to desired pose is not achievable.");
				return 0;
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
			merry_pcl.filter_cloud_z(plane_dist, z_eps, radius, centroid, selected_indices);
			avg_color = merry_pcl.find_avg_color_selected_pts(selected_indices);
			ROS_INFO_STREAM("r: " << avg_color[0] << " g: " << avg_color[1] << " b: " << avg_color[2] << "\n");
			
			int color = merry_pcl.detect_color(avg_color);
			std::cin>>color;
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
				return 0;
			}
			
			// plan path to goal destination that was determined above
			// execute motion if path was determined to be successful
			rtn_val = motionplanner.rt_arm_plan_jspace_path_current_to_qgoal(q_vec_pose);
			if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
				rtn_val = motionplanner.rt_arm_execute_planned_path();
			} else {
				ROS_WARN("Joint space path to desired pose is not achievable.");
				return 0;
			}

			gripper.release();

			// go back to pre pose
			rtn_val = motionplanner.plan_move_to_pre_pose();
			if(rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
				rtn_val = motionplanner.rt_arm_execute_planned_path();
			} else {
				ROS_WARN("Move to pre pose is not achievable.");
				return 0;
			}
		}

		ros::Duration(0.5).sleep();
		ros::spinOnce();
    }

	return 1;
}