
#include <merry_gripper/merry_gripper.h>

MerryGripper::MerryGripper(ros::NodeHandle* nodehandle):
		nh_(*nodehandle){
	// initialize ... 
    gripPub_ = nh_.advertise<std_msgs::Int16>("dynamixel_motor1_cmd", 1);
}

void MerryGripper::open(){
    std_msgs::Int16 int_angle;
    int_angle.data = 3100;
    gripPub_.publish(int_angle);
}

void MerryGripper::close(){
    std_msgs::Int16 int_angle;
    int_angle.data = 3700;
    gripPub_.publish(int_angle);
}