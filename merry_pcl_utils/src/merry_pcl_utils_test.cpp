#include <merry_pcl_utils/merry_pcl_utils.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "merry_pcl_utils_test"); //node name
    ros::NodeHandle nh;

	ROS_INFO("CONSTRUCTING START");
    // initialize merry_pcl object
    MerryPclutils merry_pcl_utils(&nh);

    Eigen::Vector3f centroid;

    pcl::PointCloud<pcl::PointXYZ> outputCloud;
    sensor_msgs::PointCloud2 pcl2_outputCloud; //(new sensor_msgs::PointCloud2); //corresponding data type for ROS message

    //set up a publisher to display clouds in rviz:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/extracted_points", 1);

    ROS_INFO("CONSTRUCTING DONE");

    // get kinect_cloud
    while (!merry_pcl_utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Merry got a pointcloud!");
    ROS_INFO("Merry is saving pointcloud ...");
    merry_pcl_utils.save_kinect_snapshot();
    merry_pcl_utils.save_kinect_clr_snapshot();  // save color version of pointcloud as well

    // read and import the cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_cloud = merry_pcl_utils.getKinectCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_color_cloud = merry_pcl_utils.getKinectColorCloud();

    while (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("kinect_clr_snapshot.pcd", *kinect_color_cloud) == -1) { //* load the file
        PCL_ERROR ("Couldn't read file kinect_clr_snapshot.pcd \n");
    }

    // check color of each points
    Eigen::Vector3d pt_color;
    int color;
    int num;
    for (size_t i = 0; i < kinect_color_cloud->points.size (); ++i) {
        // std::cout << " " << (int) kinect_color_cloud->points[i].r
        //           << " " << (int) kinect_color_cloud->points[i].g
        //           << " " << (int) kinect_color_cloud->points[i].b << std::endl;

        pt_color[0] = (int) kinect_color_cloud->points[i].r;
        pt_color[1] = (int) kinect_color_cloud->points[i].g;
        pt_color[2] = (int) kinect_color_cloud->points[i].b;
        color = merry_pcl_utils.get_point_color(pt_color);
        //cout << color << endl;
        if(color == 3) { //BLACK
            //centroid = kinect_color_cloud->points[i].getVector3fMap();
            num = i;
            break;
        }
    }

    // transfoprm
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

    A_sensor_wrt_torso = merry_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    merry_pcl_utils.transform_kinect_cloud(A_sensor_wrt_torso);

    Eigen::Vector3f plane_normal;
    double plane_dist;

    // get colored cloud



    merry_pcl_utils.extract_coplanar_pcl_operation(merry_pcl_utils.getTransformedKinectCloud()->points[num].getVector3fMap());

    merry_pcl_utils.get_general_purpose_cloud(outputCloud);

    pcl::toROSMsg(outputCloud, pcl2_outputCloud); //convert datatype to compatible ROS message type for publication
    pcl2_outputCloud.header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain        
    pubCloud.publish(pcl2_outputCloud);

    ros::Duration(1000).sleep(); // sleep for half a second
    ros::spinOnce();

}
