#ifndef MERRY_MOTIONPLANNER_H
#define MERRY_MOTIONPLANNER_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cwru_action/cwru_baxter_cart_moveAction.h>

class MerryMotionplanner {
public:
	MerryMotionplanner(ros::NodeHandle* nodehandle);
	~MerryMotionplanner() {
	}

	void send_test_goal();

	/**
	 * send a command to plan a joint space move to a pre-defined pose
	 * @return a code with the completion status
	 */
	int plan_move_to_pre_pose();

	/**
	 * send command to execute a planned motion
	 * @return a code with the completion status
	 */
	int rt_arm_execute_planned_path();

	/**
	 * sends goal command to request right arm joint angles, which will be stored in an internal variable
	 * @return a code with the completion status
	 */
	int rt_arm_request_q_data();

	/**
	 * find the right arm tool pose with respect to the torso
	 * @return a code with the completion status
	 */
	int rt_arm_request_tool_pose_wrt_torso();

	geometry_msgs::PoseStamped get_rt_tool_pose_stamped(){
		return tool_pose_stamped_;
	};

	/**
	 * get the start angles for a joint space move of the right arm
	 * @return the current joint angles of the right arm
	 */
	Eigen::VectorXd get_right_arm_joint_angles();

	/**
	 * plan a joint space motion of the right arm to a joint space pose
	 * @param q_des_vec: a vector containing the destination joint angles
	 * @return a code with the completion status
	 */
	int rt_arm_plan_jspace_path_current_to_qgoal(Eigen::VectorXd q_des_vec);

	/**
	 * send a command to move the right arm to the specified goal pose from current arm position
	 * @param des_pose: the desired destination pose for the right arm
	 * @return a code with the completion status
	 */
	int rt_arm_plan_path_current_to_goal_pose(geometry_msgs::PoseStamped des_pose);

	/**
	 * vector cartesian displacement at fixed orientation
	 * @param dp_displacement: the displacement of the vector
	 * @return a code with the completion status
	 */
	int rt_arm_plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement);

	// utilities to convert between affine and pose
	Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose);
	geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);

	// most relevant for project
	Eigen::Matrix3d compute_orientation(Eigen::Vector3f plane_normal, Eigen::Vector3f major_axis);
	Eigen::Vector3d compute_origin_des(Eigen::Vector3f centroid);
	Eigen::Affine3d construct_affine_pose(Eigen::Matrix3d orientation, Eigen::Vector3d des);


private:
	ros::NodeHandle nh_;

	// messages to send/receive cartesin goals/results
	cwru_action::cwru_baxter_cart_moveGoal cart_goal_;
	cwru_action::cwru_baxter_cart_moveResult cart_result_;

	// holder for right-arm angles
	std::vector <double> q_vec_;

	geometry_msgs::PoseStamped tool_pose_stamped_;

	// an action client to send goals to cartesian move action server
	actionlib::SimpleActionClient<cwru_action::cwru_baxter_cart_moveAction> cart_move_action_client_;

	double computed_arrival_time_;
	bool finished_before_timeout_;

	// callback for cartesian action server to return result to this node
	void doneCb_(const actionlib::SimpleClientGoalState& state, const cwru_action::cwru_baxter_cart_moveResultConstPtr& result);
};

#endif