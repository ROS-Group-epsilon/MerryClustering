#include <merry_motionplanner/merry_motionplanner.h>
#include <cwru_pcl_utils/cwru_pcl_utils.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "merry_motionplanner_action_client");
	ros::NodeHandle nh;

	Merry_Motionplanner motionplanner(&nh);
	CwruPclUtils cwru_pcl_utils(&nh);

	while(!cwru_pcl_utils.got_kinect_cloud()) {
		ROS_INFO("did not receive pointcloud");
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}
	ROS_INFO("got a pointcloud");

	// use this to transform sensor frame to torso frame
	tf::Stamped Transform tf_sensor_frame_to_torso_frame;

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
	rtn_val = motionplanner.rt_arm_execute_planned_path();

	while(ros::ok()) {
		/*
		if found block{
			get pointcloud containing the plane at the top of the block (find plane_normal and plane_dist for the plane)
			from this, get major axis and centroid of plane (use cwru_pcl_utils fxns)
			construct a goal affine pose from this (see sample code for how to do this)
			convert the affine back into a geometry_msgs::PoseStamped
			send motion plan request from current to goal
			if (rtn_val = SUCCESS){
				execute motion plan
			} else {
				return with error message
			}

			close gripper on block
			use case statements to determine color
			if color = _____ {
				determine necessary placement of arm for this color and hard code in this position
				plan path to desired joint angles
				if rtn_val = SUCCESS{
					execute motion plan
				} else {
					return
				}
				open gripper to release block once position has been achieved
			}

		plan and execute move to pre pose
		ros::Duration(0.5).sleep();
		ros::spinOnce();
		}

		return 0;

		*/
	}
}