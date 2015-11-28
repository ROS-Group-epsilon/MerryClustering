
#include <merry_gripper/merry_gripper.h>

MerryGripper::MerryGripper(ros::NodeHandle* nodehandle):
		nh_(*nodehandle){
	// initialize ... 
    initialize_motor();
}

// initialize motor
void MerryGripper::initialize_motor() {
    //std::cout<<"enter motor_id to test: ";
    int motor_id = 1; // hard coded for Yale hand
    //std::cin>>motor_id;
    char cmd_topic_name[50];
    sprintf(cmd_topic_name,"dynamixel_motor%d_cmd",motor_id);
    ROS_INFO("using command topic: %s",cmd_topic_name);

    dyn_pub_ = nh_.advertise<std_msgs::Int16>(cmd_topic_name, 1);
}

void MerryGripper::grasp() {
    std_msgs::Int16 int_angle;
    int_angle.data = 3800.0;

    ros::Time begin = ros::Time::now();
    ros::Time current;
    while(current.toSec() - begin.toSec() < 0.5) {
    	dyn_pub_.publish(int_angle);
        ros::Duration(0.05).sleep();
        current = ros::Time::now();
    }
    ROS_INFO("grasped!");
}

void MerryGripper::release() {
    std_msgs::Int16 int_angle;
    int_angle.data = 3000.0;

    ros::Time begin = ros::Time::now();
    ros::Time current;
    while(current.toSec() - begin.toSec() < 0.5) {
    	dyn_pub_.publish(int_angle);
        ros::Duration(0.05).sleep();
        current = ros::Time::now();
    }
    ROS_INFO("released!");
}

void MerryGripper::grasp_and_release() {
    std_msgs::Int16 int_angle; 
    double dt = 0.02; // repeat at freq 1/dt

    int_angle.data = 0.0;

    double mid_angle = 3400;
    double amp = 400;
    double omega = 0.1*2*M_PI; // in Hz 
    double phase = 0;
    double dbl_ang=0.0;
    short int int_ang=0;
    
    ros::Time begin = ros::Time::now();
    ros::Time current;
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (current.toSec() - begin.toSec() < 20) {
        phase += omega*dt;
        dbl_ang = amp*sin(phase)+mid_angle;
        int_ang = (short int) dbl_ang;
        int_angle.data = int_ang;
        ROS_INFO("sending: %d",int_ang);
        dyn_pub_.publish(int_angle); // publish the value--of type Float64-- 
        ros::Duration(0.05).sleep();
        current = ros::Time::now();
    }
}