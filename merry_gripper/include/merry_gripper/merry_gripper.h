#ifndef MERRY_GRIPPER_H
#define MERRY_GRIPPER_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <math.h>

class MerryGripper {
public:
	MerryGripper(ros::NodeHandle* nodehandle);
	void open();
	void close();

private:
	ros::NodeHandle nh_;
	ros::Publisher  gripPub_;
	
};

#endif