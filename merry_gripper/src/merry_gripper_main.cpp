#include <merry_gripper/merry_gripper.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "merry_gripper"); 
    ros::NodeHandle nh; 

 	MerryGripper merry_gripper(&nh);

 	merry_gripper.open();
 	ros::Duration(5).sleep(); // sleep for half a second
 	merry_gripper.close();

 	ROS_INFO("my work here is done!");
}
