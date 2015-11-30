#include <merry_hmi/merry_hmi.h>

merry_hmi::merry_hmi(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    cloudSub_ = nh_.subscribe("kinect/depth/points",1,updateKinectCB);
}

void merry_hmi::updateKinectCB(const sensor_msgs::PointCloud2 & message_holder){

	int COUNTTHRESH = 200;
	double HEIGHTTHRESH = 0.7;

	pcl::PointCloud<pcl::PointXYZ> sample;
	pcl::fromROSMsg(message_holder,sample);

	int counter = 0;
	obstructed_ = false;
	int npts = sample.points.size();
	for (int i = 0; i < npts; ++i) {
        if( sample.points[i].getVector3fMap()[2]>HEIGHTTHRESH){
        	counter++
        	if (counter>COUNTTHRESH){
        		obstructed_= true;
        		break;
        	}

        }
        
    }

}

bool merry_hmi::isObstructed(){
	return obstructed_;
}