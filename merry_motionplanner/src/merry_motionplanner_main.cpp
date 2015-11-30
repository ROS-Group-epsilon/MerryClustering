#include <merry_gripper/merry_gripper.h>
#include <merry_motionplanner/merry_motionplanner.h>
#include <cwru_pcl_utils/cwru_pcl_utils.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "merry_motionplanner_action_client");
	ros::NodeHandle nh;

	MerryGripper gripper(&nh);
	MerryMotionplanner motionplanner(&nh);
	CwruPclUtils cwru_pcl_utils(&nh);

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
			tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
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
		// TODO CHANGE THIS IF STATEMENT CONDITION TO BE IF BLOCK HAS BEEN FOUND AND NO OBSTRUCTION HAS BEEN FOUND
		if(true) {
			ROS_INFO("block has been found");


			//ASSUMING THAT I AM RECEIVING A CENTROID, MAJOR AXIS, AND PLANE_NORMAL FROM PCL LIBRARY


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

			rt_tool_pose.pose = motionplanner.transformEigenAffine3dToPose(Affine_des_gripper);

			rtn_val = motionplanner.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
			if(rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
				rtn_val = motionplanner.rt_arm_execute_planned_path();
			} else {
				ROS_WARN("Cartesian path to desired pose is not achievable.");
				return 0;
			}

			gripper.grasp();

			/*
			if(color = _____) {
				//get these numbers from my_interesting_moves code
				//hard code in positions for each block
			} else if() {

			} else if() {

			} else if() {

			} else if() {

			} else if() {

			} else {
				ROS_WARN("color of block could not be determined.");
				return 0;
			}
			*/
			//go to hard coded position with:
			/*
			plan path
			if rtn_val = SUCCESS{
				execute motion plan
			} else {
				return
			}
			*/

			gripper.release();

			rtn_val = motionplanner.plan_move_to_pre_pose();
			if(rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
				rtn_val = motionplanner.rt_arm_execute_planned_path();
			}
		}

		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}

	return 1;
}