#include <merry_motionplanner/merry_motionplanner.h>
#include <cwru_pcl_utils/cwru_pcl_utils.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "merry_motionplanner_main"); // name this node 
    ros::NodeHandle nh; //standard ros node handle

    MerryMotionplanner merry_motionplanner(&nh);
    CwruPclUtils cwru_pcl_utils(&nh);

    Eigen::Affine3d Affine_des_gripper;
    Eigen::Vector3d xvec_des,yvec_des,zvec_des,origin_des;
    geometry_msgs::PoseStamped rt_tool_pose;

    Eigen::Vector3f plane_normal, major_axis, centroid;
    Eigen::Matrix3d Rmat;
    int rtn_val; 
    /* motion planning and executing */
    //send a command to plan a joint-space move to pre-defined pose:
    rtn_val = merry_motionplanner.plan_move_to_pre_pose();  
    //send command to execute planned motion
    rtn_val = merry_motionplanner.rt_arm_execute_planned_path();
    
    while (ros::ok()) {
            

            major_axis = cwru_pcl_utils.get_major_axis();

            plane_normal = major_axis;
            
            centroid = cwru_pcl_utils.get_centroid();

            //construct a goal affine pose:
            for (int i=0;i<3;i++) {
                origin_des[i] = 0;//centroid[i]; // convert to double precision
                zvec_des[i] = 0;-plane_normal[i]; //want tool z pointing OPPOSITE surface normal
                xvec_des[i] = major_axis[i];
            }
            origin_des[2]+=0.02; //raise up 2cm
            yvec_des = zvec_des.cross(xvec_des); //construct consistent right-hand triad
            Rmat.col(0)= xvec_des;
            Rmat.col(1)= yvec_des;
            Rmat.col(2)= zvec_des;


            Affine_des_gripper.linear() = Rmat;
            Affine_des_gripper.translation() = origin_des;
            cout<<"des origin: "<<Affine_des_gripper.translation().transpose()<<endl;
            cout<<"orientation: "<<endl;
            cout<<Rmat<<endl;


            //convert des pose from Eigen::Affine to geometry_msgs::PoseStamped
            rt_tool_pose.pose = merry_motionplanner.transformEigenAffine3dToPose(Affine_des_gripper);
            //could fill out the header as well...

            
            // send move plan request:
            rtn_val=merry_motionplanner.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
            if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)  { 
                //send command to execute planned motion
                rtn_val=merry_motionplanner.rt_arm_execute_planned_path();
            }
            else {
                ROS_WARN("Cartesian path to desired pose not achievable");
            }

        ros::Duration(0.5).sleep(); // sleep for half a second
        ros::spinOnce();
    }
 
    return 0;
}

