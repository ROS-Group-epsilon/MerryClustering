// Copyright 2015 <Copyright Peng Xu>
// "include" path--should just be <baxter_kinematics/baxter_kinematics.h>, at least for modules outside this package
#include <my_interesting_moves/my_interesting_moves.h>
#include <my_interesting_moves/trajAction.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iostream>
#include <string>
#include <vector>
using namespace std;
// make these member vars of class
vector<string> g_all_jnt_names; 
vector<int> right_arm_joint_indices;

// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
My_interesting_moves::My_interesting_moves(ros::NodeHandle* nodehandle) {
    initializeSubscribers();  // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();    
    left_cmd_.mode = 1;  // set the command modes to "position"
    right_cmd_.mode = 1;

    // define the joint angles 0-6 to be right arm, from shoulder out to wrist;
    right_cmd_.names.push_back("right_s0");
    right_cmd_.names.push_back("right_s1");
    right_cmd_.names.push_back("right_e0");
    right_cmd_.names.push_back("right_e1");
    right_cmd_.names.push_back("right_w0");
    right_cmd_.names.push_back("right_w1");
    right_cmd_.names.push_back("right_w2");
    // same order for left arm
    left_cmd_.names.push_back("left_s0");
    left_cmd_.names.push_back("left_s1");
    left_cmd_.names.push_back("left_e0");
    left_cmd_.names.push_back("left_e1");
    left_cmd_.names.push_back("left_w0");
    left_cmd_.names.push_back("left_w1");
    left_cmd_.names.push_back("left_w2");

    // do push-backs to establish desired vector size with valid joint angles
    for (int i=0;i<7;i++) {
        right_cmd_.command.push_back(0.0);  // start commanding 0 angle for right-arm 7 joints
        left_cmd_.command.push_back(0.0);  // start commanding 0 angle for left-arm 7 joints
    }    

    qdot_max_vec<<q0dotmax,q1dotmax,q2dotmax,q3dotmax,q4dotmax,q5dotmax,q6dotmax;
    qdot_max_vec *=SPEED_SCALE_FACTOR;

    traj_interp_stat_client_ = nh_.serviceClient<cwru_srv::simple_bool_service_message>("trajInterpStatusSvc");

}

// member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void My_interesting_moves::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    joint_state_sub_ = nh_.subscribe("robot/joint_states", 1, &My_interesting_moves::jointStatesCb,this);  
    // add more subscribers here, as needed
}

// member helper function to set up publishers;
void My_interesting_moves::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    joint_cmd_pub_right_ = nh_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1, true); 
    joint_cmd_pub_left_  = nh_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1, true);
    right_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("right_arm_joint_path_command", 1);
    // add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

// member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void My_interesting_moves::initializeServices()
{
    // ROS_INFO("Initializing Services");
    // traj_interp_stat_client_ = nh_.advertiseService("exampleMinimalService",
    //                                               &my_interesting_moves::serviceCallback,
    //                                               this);  
    // add more services here, as needed
}

void My_interesting_moves::map_right_arm_joint_indices(vector<string> joint_names) {
    // vector<string> joint_names = joint_state->name;
    vector<string> rt_limb_jnt_names;


    right_arm_joint_indices.clear();
    int index;
    int n_jnts = joint_names.size();
    cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;
    std::string j_s0_name ("right_s0");
    rt_limb_jnt_names.push_back(j_s0_name);
    std::string j_s1_name ("right_s1");
    rt_limb_jnt_names.push_back(j_s1_name);
    std::string j_e0_name ("right_e0");
    rt_limb_jnt_names.push_back(j_e0_name);
    std::string j_e1_name ("right_e1");
    rt_limb_jnt_names.push_back(j_e1_name);
    std::string j_w0_name ("right_w0");
    rt_limb_jnt_names.push_back(j_w0_name);
    std::string j_w1_name ("right_w1");   
    rt_limb_jnt_names.push_back(j_w1_name);
    std::string j_w2_name ("right_w2");  
    rt_limb_jnt_names.push_back(j_w2_name);

    for (int j=0;j<7;j++) {
        j_name = rt_limb_jnt_names[j];
        for (int i=0;i<n_jnts;i++) {
            if (j_name.compare(joint_names[i])==0) {
                index = i;
                right_arm_joint_indices.push_back(index);
                break;
            }
        }
    }   
    cout<<"indices of right-arm joints: "<<endl;
    for (int i=0;i<7;i++) {
     cout<<right_arm_joint_indices[i]<<", ";
 }
 cout<<endl;
}


// NOTE: this is not separately threaded.  this callback only responds with the parent node allows a ros spin.
void My_interesting_moves::jointStatesCb(const sensor_msgs::JointState& js_msg) {
    joint_states_ = js_msg;  // copy this to member var
    if (right_arm_joint_indices.size()<1) {
       // g_all_jnt_names = js_msg.name;
     map_right_arm_joint_indices(js_msg.name);
 }
    // copy right-arm angles to global vec
 for (int i=0;i<7;i++)
 {
        // should do this better; manually remap from joint_states indices to right-arm joint angles
        q_vec_right_arm_[i] = js_msg.position[right_arm_joint_indices[i]]; //w2         
    }
    // cout<<"CB: q_vec_right_arm: "<<q_vec_right_arm_.transpose()<<endl;
    
}  

Vectorq7x1 My_interesting_moves::get_qvec_right_arm() {
    return q_vec_right_arm_;
}

double My_interesting_moves::transition_time(Vectorq7x1 dqvec) {
    double t_max = fabs(dqvec[0])/qdot_max_vec[0];
    cout<<"qdot max: "<<qdot_max_vec.transpose()<<endl;
    double ti;
    for (int i=1;i<7;i++) {
        ti = fabs(dqvec[i])/qdot_max_vec[i];
        if (ti>t_max) t_max= ti;
    }
    return t_max;
}

double My_interesting_moves::transition_time(Eigen::VectorXd dqvec) {
    double t_max = fabs(dqvec[0])/qdot_max_vec[0];
    cout<<"qdot max: "<<qdot_max_vec.transpose()<<endl;
    double ti;
    for (int i=1;i<7;i++) {
        ti = fabs(dqvec[i])/qdot_max_vec[i];
        if (ti>t_max) t_max= ti;
    }
    return t_max;
}

// command robot to move to "qvec" using a trajectory message, sent via ROS-I
// OBSOLETE:  USE ALT FNC BELOW
void My_interesting_moves::stuff_trajectory( std::vector<Vectorq7x1> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    new_trajectory.points.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_msgs::JointTrajectoryPoint trajectory_point2;
    
    trajectory_point1.positions.clear();
    trajectory_point2.positions.clear();
    
    for (int i=0;i<7;i++) {  // pre-size these vectors, so can access w/ indices
        trajectory_point1.positions.push_back(0.0);
        trajectory_point2.positions.push_back(0.0);
    }
    new_trajectory.points.clear();
    new_trajectory.joint_names.clear();
    
    new_trajectory.joint_names.push_back("right_s0");
    new_trajectory.joint_names.push_back("right_s1");
    new_trajectory.joint_names.push_back("right_e0");
    new_trajectory.joint_names.push_back("right_e1");
    new_trajectory.joint_names.push_back("right_w0");
    new_trajectory.joint_names.push_back("right_w1");
    new_trajectory.joint_names.push_back("right_w2");

    new_trajectory.header.stamp = ros::Time::now();
    Vectorq7x1 q_start,q_end,dqvec;
    double del_time;
    double net_time=0.0;
    q_start = qvecs[0];
    q_end = qvecs[0];
    cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl;
    for (int i=0;i<7;i++) {  //pre-size these vectors, so can access w/ indices
        trajectory_point1.positions[i]=q_start[i];
    }    
    trajectory_point1.time_from_start = ros::Duration(net_time); 
    new_trajectory.points.push_back(trajectory_point1);  // first point of the trajectory
    // add the rest of the points from qvecs
    for (int iq=1;iq<qvecs.size();iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end-q_start;
        cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time< dt_traj)
            del_time = dt_traj;
        cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl;
        net_time+= del_time;
        ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);
        for (int i=0;i<7;i++) {  // pre-size these vectors, so can access w/ indices
            trajectory_point1.positions[i]=q_end[i];
        }
        trajectory_point1.time_from_start = ros::Duration(net_time);
        new_trajectory.points.push_back(trajectory_point1);
    }

}

// this version uses vector of VectorXd for input
void My_interesting_moves::stuff_trajectory( std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //  trajectory_msgs::JointTrajectoryPoint trajectory_point2;
    
    trajectory_point1.positions.clear();
    
    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();
    
    new_trajectory.joint_names.push_back("right_s0");
    new_trajectory.joint_names.push_back("right_s1");
    new_trajectory.joint_names.push_back("right_e0");
    new_trajectory.joint_names.push_back("right_e1");
    new_trajectory.joint_names.push_back("right_w0");
    new_trajectory.joint_names.push_back("right_w1");
    new_trajectory.joint_names.push_back("right_w2");

    new_trajectory.header.stamp = ros::Time::now();  
    Eigen::VectorXd q_start,q_end,dqvec;
    double del_time;
    double net_time=0.0;
    q_start = qvecs[0];
    q_end = qvecs[0];
    cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 

    // trajectory_point1.positions = qvecs[0];

    trajectory_point1.time_from_start = ros::Duration(net_time); 
    for (int i=0;i<7;i++) {  // pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_start[i]);
    } 
    new_trajectory.points.push_back(trajectory_point1);  // first point of the trajectory
    // add the rest of the points from qvecs


    for (int iq=1;iq<qvecs.size();iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end-q_start;
        cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time< dt_traj)
            del_time = dt_traj;
        cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl;
        net_time+= del_time;
        ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time); 
        for (int i=0;i<7;i++) { //copy over the joint-command values
            trajectory_point1.positions[i]=q_end[i];
        }
        //trajectory_point1.positions = q_end;
        trajectory_point1.time_from_start = ros::Duration(net_time);
        new_trajectory.points.push_back(trajectory_point1);
    }
}

// command a single pose: cmd_pose_right(Vectorq7x1 qvec );
void My_interesting_moves::cmd_pose_right(Vectorq7x1 qvec ) {
    //member var right_cmd_ already has joint names populated
    for (int i=0;i<7;i++) {
        right_cmd_.command[i]=qvec[i];
    }
    joint_cmd_pub_right_.publish(right_cmd_);
}

// publish a trajectory; to be processed by a separate interpolating joint commander
void My_interesting_moves::pub_right_arm_trajectory(trajectory_msgs::JointTrajectory &new_trajectory) {
    right_traj_pub_.publish(new_trajectory);
    cout << "publishing trajectory with npts = " << new_trajectory.points.size() << endl;
}

// get current arm joint angles, and send some trajectories with these angles, just to get listener warmed up
void My_interesting_moves::pub_right_arm_trajectory_init() {
    std::vector<Vectorq7x1> qvecs;
    trajectory_msgs::JointTrajectory new_trajectory;
    Vectorq7x1 q_snapshot = q_vec_right_arm_;
    qvecs.push_back(q_snapshot);
    qvecs.push_back(q_snapshot);    
    stuff_trajectory(qvecs, new_trajectory);
    bool working_on_traj=true;
    while (working_on_traj) {
      traj_interp_stat_client_.call(traj_status_srv_); // communicate w/ trajectory interpolator node status service
      working_on_traj = traj_status_srv_.response.resp;
      cout<<"waiting for ready from interp node..."<<endl;
      ros::spinOnce();
  }
  cout<<"interp node is ready for traj; sending"<<endl;
  working_on_traj=true;


  right_traj_pub_.publish(new_trajectory);
  cout<<"publishing trajectory with npts = "<<new_trajectory.points.size()<<endl;
  cout<<"cmd: "<<q_snapshot.transpose()<<endl;
     ros::Duration(0.5).sleep(); // sleep for half a second
     while(working_on_traj) {
      traj_interp_stat_client_.call(traj_status_srv_); // communicate w/ trajectory interpolator node status service
      working_on_traj = traj_status_srv_.response.resp;
      cout<<"waiting for interp node to finish trajectory..."<<endl;
      ros::spinOnce();
  }   
}

// my code here
void My_interesting_moves::set_goal_salute(trajectory_msgs::JointTrajectory &des_trajectory) {
    int g_count = 0;
    Vectorq7x1 q_pre_pose;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;

    std::vector<Eigen::VectorXd> des_path;
    // cout<<"creating des_path vector; enter 1:";
    //cin>>ans;

    cout<<"instantiating a traj streamer"<<endl; // enter 1:";

    cout<<"getting current right-arm pose:"<<endl;
    ros::spinOnce();
    q_vec_right_arm =  get_qvec_right_arm();
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    // double s0,s1,e0,e1,w0,w1,w2;
    // std::cin>>s0>>s1>>e0>>e1>>w0>>w1>>w2;
    // q_pre_pose<< s0, s1, e0, e1, w0, w1, w2;
    // // q_pre_pose<< -0.8, 0.6, 0, 0.6, 0, 0.6, 0;
    // q_in_vecxd = q_pre_pose; // conversion;
    // des_path.push_back(q_in_vecxd); //twice, to define a trajectory

    {
        cout << "creating des_path vector; enter: s0,s1,e0,e1,w0,w1,w2" << endl;
        double s0,s1,e0,e1,w0,w1,w2;
        std::cin>>s0>>s1>>e0>>e1>>w0>>w1>>w2;
        q_pre_pose<< s0,s1,e0,e1,w0,w1,w2;
        q_in_vecxd = q_pre_pose;  // conversion;
        des_path.push_back(q_in_vecxd);  //twice, to define a trajectory
    }
    
    // q_pre_pose<< -0.8, 0, 3, 0, 0, 0, 0;
    // q_in_vecxd = q_pre_pose;  // conversion;
    // des_path.push_back(q_in_vecxd);  //twice, to define a trajectory
    // q_pre_pose<< -0.8, 0, 3, 2, 0, 0, 0;
    // q_in_vecxd = q_pre_pose;  // conversion;
    // des_path.push_back(q_in_vecxd); //twice, to define a trajectory
    // q_pre_pose<< -0.8, 0, 3, 2, 0, 0.8, 0;
    // q_in_vecxd = q_pre_pose;  // conversion;
    // des_path.push_back(q_in_vecxd);  //twice, to define a trajectory


    cout << "stuffing traj: " << endl;
    stuff_trajectory(des_path, des_trajectory);  // convert from vector of 7dof poses to trajectory message        
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
}

void My_interesting_moves::move_to_origin(trajectory_msgs::JointTrajectory &des_trajectory) {
    int g_count = 0;
    Vectorq7x1 q_pre_pose;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;

    std::vector<Eigen::VectorXd> des_path;
    // cout<<"creating des_path vector; enter 1:";
    // cin>>ans;

    cout<<"instantiating a traj streamer"<<endl;  // enter 1:";

    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  get_qvec_right_arm();  
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;    
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    // q_pre_pose<< 0.8, -0.4, 0, 1, 0, 0.8, 0;
    // q_in_vecxd = q_pre_pose;  // conversion;
    // des_path.push_back(q_in_vecxd);  //twice, to define a trajectory
    q_pre_pose<< 0.8, -0.8, 0, 1, 0, 0.8, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  //twice, to define a trajectory

    cout << "stuffing traj: " << endl;
    stuff_trajectory(des_path, des_trajectory);  //convert from vector of 7dof poses to trajectory message     
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
}

void My_interesting_moves::move_to_center(trajectory_msgs::JointTrajectory &des_trajectory) {
    int g_count = 0;
    Vectorq7x1 q_pre_pose;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;

    std::vector<Eigen::VectorXd> des_path;
    // cout<<"creating des_path vector; enter 1:";
    // cin>>ans;

    cout<<"instantiating a traj streamer"<<endl;  // enter 1:";

    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  get_qvec_right_arm();  
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;    
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    q_pre_pose<< 0.8, -0.8, 0, 1, 0, 0.8, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  //twice, to define a trajectory
    q_pre_pose<< 0.8, -0.3, 0, 1, 0, 0.8, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  // twice, to define a trajectory

    cout << "stuffing traj: " << endl;
    stuff_trajectory(des_path, des_trajectory);  //convert from vector of 7dof poses to trajectory message     
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
}

void My_interesting_moves::move_to_left(trajectory_msgs::JointTrajectory &des_trajectory) {
    int g_count = 0;
    Vectorq7x1 q_pre_pose;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;

    std::vector<Eigen::VectorXd> des_path;
    // cout<<"creating des_path vector; enter 1:";
    // cin>>ans;

    cout<<"instantiating a traj streamer"<<endl;  // enter 1:";

    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  get_qvec_right_arm();  
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;    
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    q_pre_pose<< 0.8, -0.8, 0, 1, 0, 0.8, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  //twice, to define a trajectory
    q_pre_pose<< 1.2, -0.2, 0, 0.8, 0, 0.8, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  // twice, to define a trajectory

    cout << "stuffing traj: " << endl;
    stuff_trajectory(des_path, des_trajectory);  //convert from vector of 7dof poses to trajectory message     
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
}

void My_interesting_moves::move_to_right(trajectory_msgs::JointTrajectory &des_trajectory) {
    int g_count = 0;
    Vectorq7x1 q_pre_pose;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;

    std::vector<Eigen::VectorXd> des_path;
    // cout<<"creating des_path vector; enter 1:";
    // cin>>ans;

    cout<<"instantiating a traj streamer"<<endl;  // enter 1:";

    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  get_qvec_right_arm();  
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;    
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    q_pre_pose<< 0.8, -0.8, 0, 1, 0, 0.8, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  //twice, to define a trajectory
    q_pre_pose<< 0.4, -0.2, 0, 0.8, 0, 0.8, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  // twice, to define a trajectory

    cout << "stuffing traj: " << endl;
    stuff_trajectory(des_path, des_trajectory);  //convert from vector of 7dof poses to trajectory message     
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
}

void My_interesting_moves::move_to_centerbehind(trajectory_msgs::JointTrajectory &des_trajectory) {
    int g_count = 0;
    Vectorq7x1 q_pre_pose;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;

    std::vector<Eigen::VectorXd> des_path;
    // cout<<"creating des_path vector; enter 1:";
    // cin>>ans;

    cout<<"instantiating a traj streamer"<<endl;  // enter 1:";

    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  get_qvec_right_arm();  
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;    
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    q_pre_pose<< 0.8, -0.6, 0, 1.5, 0, 0.6, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  //twice, to define a trajectory
    q_pre_pose<< 0.8, -0.6, 0, 2, 0, 0.6, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  // twice, to define a trajectory

    cout << "stuffing traj: " << endl;
    stuff_trajectory(des_path, des_trajectory);  //convert from vector of 7dof poses to trajectory message     
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
}

void My_interesting_moves::move_to_leftbehind(trajectory_msgs::JointTrajectory &des_trajectory) {
    int g_count = 0;
    Vectorq7x1 q_pre_pose;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;

    std::vector<Eigen::VectorXd> des_path;
    // cout<<"creating des_path vector; enter 1:";
    // cin>>ans;

    cout<<"instantiating a traj streamer"<<endl;  // enter 1:";

    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  get_qvec_right_arm();  
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;    
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    q_pre_pose<< 1.2, -0.8, 0, 1.5, 0.6, 0.4, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  //twice, to define a trajectory
    q_pre_pose<< 1.2, -0.4, 0, 1.5, 0.6, 1, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  // twice, to define a trajectory

    cout << "stuffing traj: " << endl;
    stuff_trajectory(des_path, des_trajectory);  //convert from vector of 7dof poses to trajectory message     
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
}

void My_interesting_moves::move_to_rightbehind(trajectory_msgs::JointTrajectory &des_trajectory) {
    int g_count = 0;
    Vectorq7x1 q_pre_pose;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;

    std::vector<Eigen::VectorXd> des_path;
    // cout<<"creating des_path vector; enter 1:";
    // cin>>ans;

    cout<<"instantiating a traj streamer"<<endl;  // enter 1:";

    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  get_qvec_right_arm();  
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;    
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

    q_pre_pose<< 0.4, -0.8, 0, 1.5, -0.6, 0.4, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  //twice, to define a trajectory
    q_pre_pose<< 0.4, -0.4, 0, 1.5, -0.6, 1, 0;
    q_in_vecxd = q_pre_pose;  // conversion;
    des_path.push_back(q_in_vecxd);  // twice, to define a trajectory

    cout << "stuffing traj: " << endl;
    stuff_trajectory(des_path, des_trajectory);  //convert from vector of 7dof poses to trajectory message     
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
}