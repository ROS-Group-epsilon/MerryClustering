#include "../include/block_sorter.h"

what_my_block identify_block(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud){
	
	float block_height = -DBL_MAX;
	float color_checker = DBL_MAX;
	int npts = inputCloud->width * inputCloud->height;
	pcl::PointXYZRGB so_many_points;
		
	what_my_block binfo;
		binfo.centroid  = block_centroid;
		binfo.r = pt_color(0);
		binfo.g = pt_color(1);
		binfo.b = pt_color(2);
		binfo.block_color = perceived_color;
		binfo.top_plane_z = block_height;
	return binfo;
}

// code to determine the height of the top surface of the block
void block_sorter::top_height(){
	for(int i = 0; i < npts; i++){
		if(inputCloud->points[i].z > block_height){
			so_many_points = inputCloud->points[i];
			block_height = so_many_points.z;
		}
	}
	ROS_INFO("Block at height %f found.", block_height);
	PCLPub_.publish(block_height); //need to change data type of block_height in order to publish?
}


// code to determine centroid of block //
void block_sorter::computes_centroid(){
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
	PCLPub_.publish(centroid); //need to change data type of avg_color in order to publish?
    return centroid;
	}
	}


// code to get avg. color. Comb through kinect colors and compute average color. (Disregard color=0,0,0) 
void block_sorter::find_avg_color(){
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
	PCLPub_.publish(avg_color); //need to change data type of avg_color in order to publish?
	return avg_color;
}


//code to try and match found color to a specific color
void block_sorter::color_detection(){
	float my_red = pt_color(0);
	float my_green = pt_color(1);
	float my_blue = pt_color(2);
	float ccompare;
	enum maybe_color{red_block, green_block, blue_block, black_block, white_block, wood_block};
	maybe_color perceived_color; 
	
	float red_color_compare1 = my_red - 1;
	float red_color_compare2 = my_green - 0;
	float red_color_compare3 = my_blue - 0;
	
	float green_color_compare1 = my_red - 0;
	float green_color_compare2 = my_green - 1;
	float green_color_compare3 = my_blue - 0;
	
	float blue_color_compare1 = my_red - 0;
	float blue_color_compare2 = my_green - 0;
	float blue_color_compare3 = my_blue - 1;
	
	float black_color_compare1 = my_red - 0;
	float black_color_compare2 = my_green - 0;
	float black_color_compare3 = my_blue - 0;
	
	float white_color_compare1 = my_red - 1;
	float white_color_compare2 = my_green - 1;
	float white_color_compare3 = my_blue - 1;
	
	float wood_color_compare1 = my_red - 1;
	float wood_color_compare2 = my_green - 1;
	float wood_color_compare3 = my_blue - 0;
    
	ccompare = sqrt((red_color_compare1^2)+(red_color_compare2^2)+(red_color_compare3^2));
	if(ccompare < color_checker){
		color_checker = ccompare;
		perceived_color = red_block;
		}
	
	ccompare = sqrt((green_color_compare1^2)+(green_color_compare2^2)+(green_color_compare3^2));
	if(ccompare < color_checker){
		color_checker = ccompare;
		perceived_color = green_block;
		}
	
	ccompare = sqrt((blue_color_compare1^2)+(blue_color_compare2^2)+(blue_color_compare3^2));
	if(ccompare < color_checker){
		color_checker = ccompare;
		perceived_color = blue_block;
		}
	
	ccompare = sqrt((black_color_compare1^2)+(black_color_compare2^2)+(black_color_compare3^2));
	if(ccompare < color_checker){
		color_checker = ccompare;
		perceived_color = black_block;
		}
	
	ccompare = sqrt((white_color_compare1^2)+(white_color_compare2^2)+(white_color_compare3^2));
	if(ccompare < color_checker){
		color_checker = ccompare;
		perceived_color = white_block;
		}
	
	ccompare = sqrt((wood_color_compare1^2)+(wood_color_compare2^2)+(wood_color_compare3^2));
	if(ccompare < color_checker){
		color_checker = ccompare;
		perceived_color = wood_block;
		}
	
	PCLPub_.publish(perceived_color); //need to change data type of perceived_color in order to publish?
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
