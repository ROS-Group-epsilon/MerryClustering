#include <merry_motionplanner/merry_motionplanner.h>

/* public member functions */
MerryMotionplanner::MerryMotionplanner(ros::NodeHandle *nodehandle):nh_(*nodehandle), cart_move_action_client_("cartMoveActionServer", true) {
    ROS_INFO("in constructor of motion planner");

    ROS_INFO("waiting for server");
    bool server_exists = false;
    while ((!server_exists) && (ros::ok())) {
        server_exists = cart_move_action_client_.waitForServer(ros::Duration(0.5));
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }

    ROS_INFO("connected to action server");

}


void MerryMotionplanner::send_test_goal() {
    ROS_INFO("sending a test goal");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::ARM_TEST_MODE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&MerryMotionplanner::doneCb_, this, _1, _2));

    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));

    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
    } else {
        ROS_INFO("finished before timeout");
        ROS_INFO("return code: %d",cart_result_.return_code);
    }
}


int MerryMotionplanner::plan_move_to_pre_pose() {
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&MerryMotionplanner::doneCb_, this, _1, _2));
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);

    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    } 
    ROS_INFO("finished before timeout");

    if (cart_result_.return_code == cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }

    if (cart_result_.return_code != cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    // here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_ = cart_result_.computed_arrival_time; // action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;
}


int MerryMotionplanner::rt_arm_execute_planned_path() {
    ROS_INFO("requesting execution of panned path");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_EXECUTE_PLANNED_PATH;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&MerryMotionplanner::doneCb_, this, _1, _2));

    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_ + 2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not complete move in expected time");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }

    if (cart_result_.return_code != cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d", cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    ROS_INFO("move returned success");
    return (int) cart_result_.return_code;
}


int MerryMotionplanner::rt_arm_request_q_data() {
    ROS_INFO("requesting right-arm joint angles");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_GET_Q_DATA;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&MerryMotionplanner::doneCb_, this, _1, _2));

    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;  
    }

    if (cart_result_.return_code != cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }
    
    q_vec_ = cart_result_.q_arm_right;
    ROS_INFO("move returned success; right arm angles: ");
    ROS_INFO("%f; %f; %f; %f; %f; %f; %f",q_vec_[0],q_vec_[1],q_vec_[2],q_vec_[3],q_vec_[4],q_vec_[5],q_vec_[6]);
    return (int) cart_result_.return_code;
}


int MerryMotionplanner::rt_arm_request_tool_pose_wrt_torso() {
    ROS_INFO("requesting right-arm tool pose");    
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_GET_TOOL_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&MerryMotionplanner::doneCb_, this, _1, _2));

    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;  
    }

    if (cart_result_.return_code != cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }    
    
    tool_pose_stamped_ = cart_result_.current_pose_gripper_right;
    ROS_INFO("move returned success; right arm tool pose: ");

    ROS_INFO("origin w/rt torso = %f, %f, %f ",tool_pose_stamped_.pose.position.x,
    tool_pose_stamped_.pose.position.y,tool_pose_stamped_.pose.position.z);
    ROS_INFO("quaternion x,y,z,w: %f, %f, %f, %f",tool_pose_stamped_.pose.orientation.x,
    tool_pose_stamped_.pose.orientation.y,tool_pose_stamped_.pose.orientation.z,
    tool_pose_stamped_.pose.orientation.w);

    return (int) cart_result_.return_code;
}


Eigen::VectorXd MerryMotionplanner::get_right_arm_joint_angles() {
    rt_arm_request_q_data();
    Eigen::VectorXd rt_arm_angs_vecXd;

    rt_arm_angs_vecXd.resize(7);
    for (int i = 0; i < 7; i++) {
        rt_arm_angs_vecXd[i] = q_vec_[i];
    }

    return rt_arm_angs_vecXd;
}


int MerryMotionplanner::rt_arm_plan_jspace_path_current_to_qgoal(Eigen::VectorXd q_des_vec) {
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL;

    cart_goal_.q_goal_right.resize(7);
    for (int i = 0; i < 7; i++) {
        cart_goal_.q_goal_right[i] = q_des_vec[i]; //specify the goal js pose
    }

    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&MerryMotionplanner::doneCb_, this, _1, _2));

    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    } 
    ROS_INFO("finished before timeout");

    if (cart_result_.return_code == cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }

    if (cart_result_.return_code != cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_ = cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code; 
}


int MerryMotionplanner::rt_arm_plan_path_current_to_goal_pose(geometry_msgs::PoseStamped des_pose) {
    ROS_INFO("requesting a cartesian-space motion plan");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE;
    cart_goal_.des_pose_gripper_right = des_pose;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&MerryMotionplanner::doneCb_, this, _1, _2));

    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    } 
    ROS_INFO("finished before timeout");

    if (cart_result_.return_code == cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }

    if (cart_result_.return_code != cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_ = cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code; 
}


int MerryMotionplanner::rt_arm_plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement) {
    ROS_INFO("requesting a cartesian-space motion plan along vector");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ;

    // must fill in desired vector displacement
    cart_goal_.arm_dp_right.resize(3);
    for (int i = 0; i < 3; i++) {
        cart_goal_.arm_dp_right[i] = dp_displacement[i];
    }

    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&MerryMotionplanner::doneCb_, this, _1, _2));

    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    } 
    ROS_INFO("finished before timeout");

    if (cart_result_.return_code == cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }

    if (cart_result_.return_code != cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_ = cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;
}


Eigen::Affine3d MerryMotionplanner::transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
    Eigen::Affine3d affine;
    Eigen::Vector3d Oe;

    Oe(0) = pose.position.x;
    Oe(1) = pose.position.y;
    Oe(2) = pose.position.z;
    affine.translation() = Oe;

    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q);

    affine.linear() = Re;

    return affine;
}


geometry_msgs::Pose MerryMotionplanner::transformEigenAffine3dToPose(Eigen::Affine3d e) {
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    geometry_msgs::Pose pose;

    Oe = e.translation();
    Re = e.linear();

    // convert rotation matrix Re to a quaternion q
    Eigen::Quaterniond q(Re);
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

/* most relevant */
Eigen::Matrix3d MerryMotionplanner::compute_orientation(Eigen::Vector3f plane_normal, Eigen::Vector3f major_axis) {
    Eigen::Vector3d xvec_des, yvec_des, zvec_des;
    Eigen::Matrix3d Rmat;
    for (int i = 0; i < 3; i++) {
        zvec_des[i] = -plane_normal[i]; //want tool z pointing OPPOSITE surface normal
        xvec_des[i] = major_axis[i];
    }
    yvec_des = zvec_des.cross(xvec_des); //construct consistent right-hand triad
    Rmat.col(0)= xvec_des;
    Rmat.col(1)= yvec_des;
    Rmat.col(2)= zvec_des;
    return Rmat;
}


Eigen::Vector3d MerryMotionplanner::compute_origin_des(Eigen::Vector3f centroid) {
    Eigen::Vector3d origin_des;
    for (int i = 0; i < 3; i++) {
        origin_des[i] = centroid[i]; // convert to double precision
    }
    origin_des[2] += 0.02; //raise up 2cm
}


//construct a goal affine pose:
Eigen::Affine3d MerryMotionplanner::construct_affine_pose(Eigen::Matrix3d orientation, Eigen::Vector3d des) {
    Eigen::Affine3d Affine_des_gripper;       
    Affine_des_gripper.linear() = orientation;
    Affine_des_gripper.translation() = des;
    std::cout << "des origin: " << Affine_des_gripper.translation().transpose() << "\n";
    std::cout << "orientation: " << "\n";
    std::cout << orientation << "\n";
    return Affine_des_gripper;
}




/* private member functions */

void MerryMotionplanner::doneCb_(const actionlib::SimpleClientGoalState& state, const cwru_action::cwru_baxter_cart_moveResultConstPtr& result) {
    ROS_INFO("doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return value = %d", result->return_code);
    cart_result_= *result;
}



