#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <math.h>

class gripper_control{
public:
	gripper_control(ros::NodeHandle* nodehandle);
	void open();
	void close();
private:
	ros::NodeHandle nh_;
	ros::Publisher  gripPub_;
	
};

#endif