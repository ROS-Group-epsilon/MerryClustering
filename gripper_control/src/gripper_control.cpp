
#include <gripper_control/gripper_control.h>

gripper_control::gripper_control(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    gripPub_ = nh_.advertise<std_msgs::Int16>("dynamixel_motor1_cmd", 1);
}

void gripper_control::open(){
    std_msgs::Int16 int_angle;
    int_angle.data = 3100;
    gripPub_.publish(int_angle);
}
void gripper_control::close(){
    std_msgs::Int16 int_angle;
    int_angle.data = 3700;
    gripPub_.publish(int_angle);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_control"); 
    ros::NodeHandle n; 


}



