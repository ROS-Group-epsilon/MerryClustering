#include "../include/block_sorter.h"

what_my_block identify_block(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud){
	
	float xdirc = 0;
	float ydirc = 0;
	float xydirc = 0;
	float z_direction_z = -DBL_MAX;
	int npts = inputCloud->width * inputCloud->height;
	pcl::PointXYZRGB pointpoint;
	
	// code to determine the height of the top surface of the block //
	for(int i = 0; i < npts; i++){
		if(inputCloud->points[i].z > z_direction_z){
			pointpoint = inputCloud->points[i];
			z_direction_z = pointpoint.z;
		}
	}
	ROS_INFO("Block at height %f found.", z_direction_z);
	// end code to determine the height of the top surface of the block //
	
		
	// code to determine centroid of block //
	Eigen::Vector3f block_centroid = compute_centroid(rest_of_block);
	Eigen::Vector3f  compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
    Eigen::Vector3f centroid;
    //Eigen::Vector3f cloud_pt;   
    int npts = cloud_ptr->points.size();    
    centroid<<0,0,0;
    //add all the points together:
    for (int ipt = 0; ipt < npts; ipt++) {
        //cloud_pt = input_cloud_ptr->points[ipt].getVector3fMap();
        centroid += cloud_ptr; //add all the column vectors together
    }
    centroid/= npts; //divide by the number of points to get the centroid
    return centroid;
	}
    // end code to determine centroid of block //
	
	
	
	
	// code to get avg. color
	// comb through kinect colors and compute average color
	// disregard color=0,0,0 
	Eigen::Vector3d find_avg_color() {
    Eigen::Vector3d avg_color;
    Eigen::Vector3d pt_color;
    Eigen::Vector3d ref_color;
    indices_.clear();
    ref_color<<147,147,147;
    int npts2 = pclKinect_clr_ptr_->points.size();
    int npts2_colored = 0;
    for (int i=0;i<npts2;i++) {
        pt_color(0) = (double) pclKinect_clr_ptr_->points[i].r;
        pt_color(1) = (double) pclKinect_clr_ptr_->points[i].g;
        pt_color(2) = (double) pclKinect_clr_ptr_->points[i].b;

    if ((pt_color-ref_color).norm() > 1) {
        avg_color+= pt_color;
        npts2_colored++;
        indices_.push_back(i); // save this points as "interesting" color
    }
    }
    ROS_INFO("found %d points with interesting color",npts2_colored);
    avg_color/=npts2_colored;
    ROS_INFO("avg interesting color = %f, %f, %f",avg_color(0),avg_color(1),avg_color(2));
    return avg_color;
	}
	// code to get avg. color
	
	
	what_my_block binfo;
		binfo.centroid  = block_centroid;
		binfo.r = pt_color(0);
		binfo.g = pt_color(1);
		binfo.b = pt_color(2);
		binfo.top_plane_z = z_direction_z;
		
		return binfo;
}


















/*void find_indices_color_match(vector<int> &input_indices,
                    Eigen::Vector3d normalized_avg_color,
                    double color_match_thresh, vector<int> &output_indices) {
     Eigen::Vector3d pt_color;

    int npts = input_indices.size();
    output_indices.clear();
    int index;
    int npts_matching = 0;

    for (int i=0;i<npts;i++) {
        index = input_indices[i];
        pt_color(0) = (double) pclKinect_clr_ptr_->points[index].r;
        pt_color(1) = (double) pclKinect_clr_ptr_->points[index].g;
        pt_color(2) = (double) pclKinect_clr_ptr_->points[index].b;
        pt_color = pt_color/pt_color.norm(); //compute normalized color
        if ((normalized_avg_color-pt_color).norm()<color_match_thresh) {
            output_indices.push_back(index);  //color match, so save this point index
            npts_matching++;
        }
    }   
    ROS_INFO("found %d color-match points from indexed set",npts_matching);
    
} 

//find points that are both (approx) coplanar at height z_nom AND within "radius" of "centroid"
void CwruPclUtils::filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, 
                double radius, Eigen::Vector3f centroid, vector<int> &indices)  {
    int npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    double dz;
    int ans;
    for (int i = 0; i < npts; ++i) {
        pt = inputCloud->points[i].getVector3fMap();
        //cout<<"pt: "<<pt.transpose()<<endl;
        dz = pt[2] - z_nom;
        if (fabs(dz) < z_eps) {
            //passed z-test; do radius test:
            if ((pt-centroid).norm()<radius) {
               indices.push_back(i);
            }
            //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
            //cin>>ans;
        }
    }
    int n_extracted = indices.size();
    cout << " number of points in range = " << n_extracted << endl;    
    
}

void CwruPclUtils::analyze_selected_points_color() {
    int npts = pclTransformedSelectedPoints_ptr_->points.size(); //number of points
    //copy_cloud(pclTransformedSelectedPoints_ptr_,pclGenPurposeCloud_ptr_); //now have a copy of the selected points in gen-purpose object
    //Eigen::Vector3f offset;
    //offset<<0,0,0.05;
    int npts_clr = pclSelectedPtsClr_ptr_->points.size();
    cout<<"color pts size = "<<npts_clr<<endl;
        pcl::PointXYZRGB p;
        // unpack rgb into r/g/b
        uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
        uint8_t r,g,b;
        int r_int;
    
    for (int i = 0; i < npts; ++i) {
        p = pclSelectedPtsClr_ptr_->points[i];
        r = (rgb >> 16) & 0x0000ff;
        r_int = (int) r;
        // g = (rgb >> 8)  & 0x0000ff;
        // b = (rgb)       & 0x0000ff;
        cout<<"r_int: "<<r_int<<endl;
        cout<<"r1: "<<r<<endl;
        r=pclSelectedPtsClr_ptr_->points[i].r;
        cout<<"r2 = "<<r<<endl;
 
        //cout<<" ipt, r,g,b = "<<i<<","<<pclSelectedPtsClr_ptr_->points[i].r<<", "<<
        //        pclSelectedPtsClr_ptr_->points[i].g<<", "<<pclSelectedPtsClr_ptr_->points[i].b<<endl;
        //pclGenPurposeCloud_ptr_->points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap()+offset;   
    }    
        cout<<"done combing through selected pts"<<endl;
        got_kinect_cloud_=false; // get a new snapshot
} 
*////////////////////////////////
