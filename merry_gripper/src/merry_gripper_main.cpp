#include <merry_gripper/merry_gripper.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "merry_gripper_main"); 
    ros::NodeHandle nh; 

 	MerryGripper merry_gripper(&nh);

 	int command;
 	while(1) {
 		std::cout << "MerryGripper is waiting for your command:"
 				  << std::endl
 				  << "1: grasp 2: release 3: grasp_and_release 0: quit"
 			      << std::endl;
  		std::cin >> command;
  		if(command == 0) break;		
	 	switch(command) {
	 		case 1:
	 			merry_gripper.grasp();
	 			break;
	 		case 2:
	 			merry_gripper.release();
	 			break;
	 		case 3:
	 			merry_gripper.grasp_and_release();
	 			break;
	 		default:
	 			break; 			
	 	}
	}

 	ROS_INFO("my work here is done!");
}
