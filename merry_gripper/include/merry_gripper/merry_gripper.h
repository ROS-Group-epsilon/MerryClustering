#ifndef MERRY_GRIPPER_H
#define MERRY_GRIPPER_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <math.h>

class MerryGripper {
public:
	MerryGripper(ros::NodeHandle* nodehandle);
	void grasp();
	void release();
	void grasp_and_release();

private:
	ros::NodeHandle nh_;
	ros::Publisher dyn_pub_;
	ros::Publisher gripper_cmd_;

	void initialize_motor();
};

#endif