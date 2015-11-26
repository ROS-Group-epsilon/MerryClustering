
#include <merry_gripper/merry_gripper.h>

merry_gripper::merry_gripper(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    gripPub_ = nh_.advertise<std_msgs::Int16>("dynamixel_motor1_cmd", 1);
}

void merry_gripper::open(){
    std_msgs::Int16 int_angle;
    int_angle.data = 3100;
    gripPub_.publish(int_angle);
}
void merry_gripper::close(){
    std_msgs::Int16 int_angle;
    int_angle.data = 3700;
    gripPub_.publish(int_angle);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "merry_gripper"); 
    ros::NodeHandle n; 


}



