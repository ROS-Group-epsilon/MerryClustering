#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include <merry_motionplanner/merry_motionplanner.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "merry_motionplanner_test"); // name this node 
    ros::NodeHandle nh; //standard ros node handle

    MerryMotionplanner merry_motionplanner(&nh);
    CwruPclUtils cwru_pcl_utils(&nh);


    while (!cwru_pcl_utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("got a pointcloud");
    tf::StampedTransform tf_sensor_frame_to_torso_frame; //use this to transform sensor frame to torso frame
    tf::TransformListener tf_listener; //start a transform listener

    //let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from sensor to world; ready to roll
    //convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_sensor_wrt_torso;
    Eigen::Affine3d Affine_des_gripper;
    Eigen::Vector3d origin_des;
    geometry_msgs::PoseStamped rt_tool_pose;
    
    A_sensor_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    Eigen::Vector3f plane_normal, major_axis, centroid;
    Eigen::Matrix3d Rmat;
    int rtn_val;    
    double plane_dist;
    
    //send a command to plan a joint-space move to pre-defined pose:
    rtn_val = merry_motionplanner.plan_move_to_pre_pose();
    
    //send command to execute planned motion
    rtn_val = merry_motionplanner.rt_arm_execute_planned_path();
    
    while (ros::ok()) {
        if (cwru_pcl_utils.got_selected_points()) {
            ROS_INFO("transforming selected points");
            cwru_pcl_utils.transform_selected_points_cloud(A_sensor_wrt_torso);

            //fit a plane to these selected points:
            cwru_pcl_utils.fit_xformed_selected_pts_to_plane(plane_normal, plane_dist);
            ROS_INFO_STREAM(" normal: " << plane_normal.transpose() << "; dist = " << plane_dist);
            major_axis= cwru_pcl_utils.get_major_axis();
            centroid = cwru_pcl_utils.get_centroid();
            cwru_pcl_utils.reset_got_selected_points();   // reset the selected-points trigger

            //input: centroid, plane_normal, major_axis
            //output: Rmat, origin_des
            origin_des = merry_motionplanner.compute_origin_des(centroid);
            Rmat = merry_motionplanner.compute_orientation(plane_normal, major_axis);

            //construct a goal affine pose:
            Affine_des_gripper = merry_motionplanner.construct_affine_pose(Rmat, origin_des);

            //convert des pose from Eigen::Affine to geometry_msgs::PoseStamped
            rt_tool_pose.pose = merry_motionplanner.transformEigenAffine3dToPose(Affine_des_gripper);
            //could fill out the header as well...

            // send move plan request:
            rtn_val = merry_motionplanner.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
            if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)  { 
                //send command to execute planned motion
                rtn_val = merry_motionplanner.rt_arm_execute_planned_path();
            }
            else {
                ROS_WARN("Cartesian path to desired pose not achievable");
            }
        }
        ros::Duration(0.5).sleep(); // sleep for half a second
        ros::spinOnce();
    }

    return 0;
}

