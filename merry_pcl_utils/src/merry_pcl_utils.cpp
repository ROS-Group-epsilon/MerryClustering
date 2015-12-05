#include <merry_pcl_utils/merry_pcl_utils.h>

MerryPclutils::MerryPclutils(ros::NodeHandle* nodehandle):
        nh_(*nodehandle),
        pclKinect_ptr_(new PointCloud<pcl::PointXYZ>),
        pclKinect_clr_ptr_(new PointCloud<pcl::PointXYZRGB>),
        pclTransformed_ptr_(new PointCloud<pcl::PointXYZ>),
        pclExtractedPoints_ptr_(new PointCloud<pcl::PointXYZ>),
        pclExtractedPtsClr_ptr_(new PointCloud<pcl::PointXYZRGB>),
        pclTransformedExtractedPoints_ptr_(new PointCloud<pcl::PointXYZ>),
		pclGenPurposeCloud_ptr_(new PointCloud<pcl::PointXYZ>) {
    // initialize ... 
    ROS_INFO("In Constructor ... ");    
    initializeSubscribers();
    initializePublishers();
    got_kinect_cloud_ = false;
    got_extracted_points_ = false;
}

// member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void MerryPclutils::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");

    pointcloud_subscriber_ = nh_.subscribe("/kinect/depth/points", 1, &MerryPclutils::kinectCB, this);
    // subscribe to "extracted_points", which is published by Rviz tool
    extracted_points_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2> ("/extracted_points", 1, &MerryPclutils::extractCB, this);
}

//member helper function to set up publishers;
void MerryPclutils::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    //pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("merry_pcl_pointcloud", 1, true);
    //patch_publisher_ = nh_.advertise<cwru_msgs::PatchParams>("pcl_patch_params", 1, true);
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

/**
 * callback fnc: receives transmissions of Kinect data; if got_kinect_cloud is false, copy current transmission to internal variable
 * @param cloud [in] messages received from Kinect
 */
void MerryPclutils::kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (!got_kinect_cloud_) {
        pcl::fromROSMsg(*cloud, *pclKinect_ptr_);
        pcl::fromROSMsg(*cloud, *pclKinect_clr_ptr_);
        ROS_INFO("kinectCB: got cloud with %d * %d points", (int) pclKinect_ptr_->width, (int) pclKinect_ptr_->height);
        got_kinect_cloud_ = true; //cue to "main" that callback received and saved a pointcloud 
    }
}

void MerryPclutils::extractCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    ROS_INFO("Extracting ... ");
    printf ("Cloud: width = %d, height = %d\n", cloud->width, cloud->height);
}

Eigen::Vector3f MerryPclutils::get_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
    return compute_centroid(cloud_ptr);
}

Eigen::Vector3f MerryPclutils::get_plane_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
    compute_plane_normal_and_major_axis(cloud_ptr);
    return plane_normal_;
}

Eigen::Vector3f MerryPclutils::get_major_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
    compute_plane_normal_and_major_axis(cloud_ptr);
    return major_axis_;
}


void MerryPclutils::get_transformed_extracted_points(pcl::PointCloud<pcl::PointXYZ> & outputCloud) {
    int npts = pclTransformedExtractedPoints_ptr_->points.size(); //how many points to extract
    outputCloud.header = pclTransformedExtractedPoints_ptr_->header;
    outputCloud.is_dense = pclTransformedExtractedPoints_ptr_->is_dense;
    outputCloud.width = npts;
    outputCloud.height = 1;

    //cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud.points[i].getVector3fMap() = pclTransformedExtractedPoints_ptr_->points[i].getVector3fMap();   
    }
}


//same as above, but for general-purpose cloud
void MerryPclutils::get_general_purpose_cloud(pcl::PointCloud<pcl::PointXYZ> & outputCloud) {
    int npts = pclGenPurposeCloud_ptr_->points.size(); //how many points to extract
    outputCloud.header = pclGenPurposeCloud_ptr_->header;
    outputCloud.is_dense = pclGenPurposeCloud_ptr_->is_dense;
    outputCloud.width = npts;
    outputCloud.height = 1;

    //cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud.points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap();
        //cout << "copying point num = " << i << endl;  
    }    
} 


Eigen::Affine3f MerryPclutils::transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3f e;
    // treat the Eigen::Affine as a 4x4 matrix:
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i]; //copy the origin from tf to Eigen
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j]; //and copy 3x3 rotation matrix
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}

/**here is a function that transforms a cloud of points into an alternative frame;
 * it assumes use of pclKinect_ptr_ from kinect sensor as input, to pclTransformed_ptr_ , the cloud in output frame
 * 
 * @param A [in] supply an Eigen::Affine3f, such that output_points = A*input_points
 */
void MerryPclutils::transform_kinect_cloud(Eigen::Affine3f A) {
    transform_cloud(A, pclKinect_ptr_, pclTransformed_ptr_);
}

void MerryPclutils::transform_selected_points_cloud(Eigen::Affine3f A) {
    transform_cloud(A, pclExtractedPoints_ptr_, pclTransformedExtractedPoints_ptr_);
}


// code to determine the height of the top surface of the block
Eigen::Vector3f MerryPclutils::get_top_point(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud){
	float top_height = -DBL_MAX;
    //float max = -DBL_MAX, min = DBL_MAX;
	int npts = inputCloud->points.size();//->width*inputCloud->height;
	pcl::PointXYZ pt, top_point;
    //Eigen::Vector3f centroid = compute_centroid(inputCloud);
	for(int i = 0; i < npts; i++){
        pt = inputCloud->points[i];
		if(pt.z > top_height && pt.x > 0.5) {
            //cout<<i<<endl;
			top_height = pt.z;
            top_point = pt;
            //cout<<"top_height = "<< top_height <<endl;
		}
	}
	ROS_INFO("Top height %f found.", top_height);
    ROS_INFO("top_point.x = %f; top_point.y = %f; top_point.z = %f", top_point.x, top_point.y, top_point.z);
	return top_point.getVector3fMap();
}


// use it after transforming the kinect cloud
bool MerryPclutils::isBlockExist() {
    //get_top_height
    Eigen::Vector3f pt = get_top_point(pclTransformed_ptr_);
    int npts = pclTransformed_ptr_->points.size();//->width*inputCloud->height;
    int count = 0;
    cout<< "isBlockExist ... " << endl;

    for (int i = 0; i < npts; ++i) {
        if( distance_between(pt, pclTransformed_ptr_->points[i].getVector3fMap()) < 0.5 //&& pclTransformed_ptr_->points[i].x > 0.5) {
                && fabs(pt[2] - pclTransformed_ptr_->points[i].getVector3fMap()[2]) < 0.001 ) {
            count++;
        }
    }
    if(count < pclTransformed_ptr_->points.size()*0.3) return true;
    return false;
}


/**
    This function is to extract the plane that colanar with the extracted patch.
*/
void MerryPclutils::extract_coplanar_pcl_operation(Eigen::Vector3f pt) {
    int npts = pclTransformed_ptr_->points.size(); //number of points in kinect point cloud
    //pclGenPurposeCloud_ptr_->points.resize(npts);

    cout<< "coplanar ... " << endl;

    for (int i = 0; i < npts; ++i) {
        if( distance_between(pt, pclTransformed_ptr_->points[i].getVector3fMap()) < 0.5 //&& pclTransformed_ptr_->points[i].x > 0.5) {
                && fabs(pt[2] - pclTransformed_ptr_->points[i].getVector3fMap()[2]) < 0.001 ) {

            //cout << "height of centroid = " << centroid[2] << endl;
            cout << "the height of point " << i << "= " << pclTransformed_ptr_->points[i].getVector3fMap()[2] << endl;
            pclGenPurposeCloud_ptr_->points.push_back(pclTransformed_ptr_->points[i]);
        }  
    }

    pclGenPurposeCloud_ptr_->header = pclTransformedExtractedPoints_ptr_->header;
    pclGenPurposeCloud_ptr_->is_dense = pclTransformedExtractedPoints_ptr_->is_dense;
    pclGenPurposeCloud_ptr_->width = npts;
    pclGenPurposeCloud_ptr_->height = 1; 

}


//code to try and match found color to a specific color
int MerryPclutils::detect_color(Eigen::Vector3d pt_color){
    // process the input color message
    int r = pt_color(0);
    int g = pt_color(1);
    int b = pt_color(2);
    // tolerance
    int tolerance = 100;

    if(isRed(r,g,b,tolerance)) return RED;
    if(isGreen(r,g,b,tolerance)) return GREEN;
    if(isBlue(r,g,b,tolerance)) return BLUE;
    if(isBlack(r,g,b,tolerance)) return BLACK;
    if(isWhite(r,g,b,tolerance)) return WHITE;
    if(isWoodcolor(r,g,b,tolerance)) return WOODCOLOR;

    //ROS_INFO("color undefined");
    return NONE;
}


void MerryPclutils::find_indices_color_match(vector<int> &input_indices,
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


//operate on transformed Kinect pointcloud:
void MerryPclutils::find_coplanar_pts_z_height(double plane_height,double z_eps,vector<int> &indices) {
    filter_cloud_z(pclTransformed_ptr_,plane_height,z_eps,indices);
}

//special case of above for transformed Kinect pointcloud:
void MerryPclutils::filter_cloud_z(double z_nom, double z_eps, 
                double radius, Eigen::Vector3f centroid, vector<int> &indices) {
   filter_cloud_z(pclTransformed_ptr_, z_nom, z_eps, radius, centroid, indices); // go to the following   
}

//find points that are both (approx) coplanar at height z_nom AND within "radius" of "centroid"
void MerryPclutils::filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, 
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
            if (distance_between(pt, pclTransformed_ptr_->points[i].getVector3fMap()) < radius) {
               indices.push_back(i);
            }
            //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
            //cin>>ans;
        }
    }
    int n_extracted = indices.size();
    cout << " number of points in range = " << n_extracted << endl;  
}


void MerryPclutils::filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, vector<int> &indices) {
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
            indices.push_back(i);
            //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
            //cin>>ans;
        }
    }
    int n_extracted = indices.size();
    cout << " number of points in range = " << n_extracted << endl;
}


// code to get avg. color. Comb through kinect colors and compute average color. (Disregard color=0,0,0) 
Eigen::Vector3d MerryPclutils::find_avg_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr_){
    Eigen::Vector3d avg_color;
    Eigen::Vector3d ref_color;
    ref_color << 147,147,147;
    int npts2 = pclKinect_clr_ptr_->points.size();
    int npts2_colored = 0;
    for (int i=0;i<npts2;i++) {
        pt_color(0) = (double) pclKinect_clr_ptr_->points[i].r;
        pt_color(1) = (double) pclKinect_clr_ptr_->points[i].g;
        pt_color(2) = (double) pclKinect_clr_ptr_->points[i].b;

    if ((pt_color-ref_color).norm() > 1) {
        avg_color+= pt_color;
        npts2_colored++;
        //indices_.push_back(i); // save this points as "interesting" color
    }
    }
    ROS_INFO("found %d points with interesting color",npts2_colored);
    avg_color/=npts2_colored;
    ROS_INFO("avg interesting color = %f, %f, %f",avg_color(0),avg_color(1),avg_color(2));
    return avg_color;
}



Eigen::Vector3d MerryPclutils::find_avg_color_selected_pts(vector<int> &indices) {
    Eigen::Vector3d avg_color;
    Eigen::Vector3d pt_color;
    //Eigen::Vector3d ref_color;

    int npts = indices.size();
    int index;

    for (int i=0;i<npts;i++) {
        index = indices[i];
        pt_color(0) = (double) pclKinect_clr_ptr_->points[index].r;
        pt_color(1) = (double) pclKinect_clr_ptr_->points[index].g;
        pt_color(2) = (double) pclKinect_clr_ptr_->points[index].b;
        avg_color += pt_color;
    }
    avg_color /= npts;
    ROS_INFO("avg color = %f, %f, %f",avg_color(0),avg_color(1),avg_color(2));
    return avg_color;
}





/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* ++++++++++++++ following are private member functions ++++++++++++++ */

// code to determine centroid of block
Eigen::Vector3f MerryPclutils::compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
    Eigen::Vector3f cloud_pt;   
    int npts = cloud_ptr->points.size();    
    centroid_ = Eigen::MatrixXf::Zero(3, 1); //<<0,0,0; both right
    //add all the points together:
    for (int ipt = 0; ipt < npts; ipt++) {
        cloud_pt = cloud_ptr->points[ipt].getVector3fMap();
        centroid_ += cloud_pt; //add all the column vectors together
    }
    centroid_ /= npts; //divide by the number of points to get the centroid
    return centroid_;
}


// code to determine major axis
void MerryPclutils::compute_plane_normal_and_major_axis(Eigen::MatrixXf points_mat) {
    ROS_INFO("starting computation of plane_normal_and_major_axis from data: ");
    int npts = points_mat.cols(); // number of points = number of columns in matrix; check the size

    // centroid_ has been computed before
    // subtract this centroid from all points in points_mat:
    Eigen::MatrixXf points_offset_mat = points_mat;
    for (int ipt = 0; ipt < npts; ipt++) {
        points_offset_mat.col(ipt) = points_offset_mat.col(ipt) - centroid_;
    }
    //compute the covariance matrix w/rt x,y,z:
    Eigen::Matrix3f CoVar;
    CoVar = points_offset_mat * (points_offset_mat.transpose()); //3xN matrix times Nx3 matrix is 3x3
    //cout<<"covariance: "<<endl;
    //cout<<CoVar<<endl;

    // here is a more complex object: a solver for eigenvalues/eigenvectors;
    // we will initialize it with our covariance matrix, which will induce computing eval/evec pairs
    Eigen::EigenSolver<Eigen::Matrix3f> es3f(CoVar);

    Eigen::VectorXf evals; //we'll extract the eigenvalues to here
    evals = es3f.eigenvalues().real(); // grab just the real parts

    double min_lambda = evals[0]; //initialize the hunt for min eval
    double max_lambda = evals[0]; // and for max eval

    plane_normal_ = es3f.eigenvectors().col(0).real(); //complex_vec.real(); //strip off the real part
    major_axis_ = es3f.eigenvectors().col(0).real(); // starting assumptions

    double lambda_test;
    int i_normal = 0;
    int i_major_axis=0;
    //loop through "all" ("both", in this 3-D case) the rest of the solns, seeking min e-val
    for (int ivec = 1; ivec < 3; ivec++) {
        lambda_test = evals[ivec];
        if (lambda_test < min_lambda) {
            min_lambda = lambda_test;
            i_normal = ivec; //this index is closer to index of min eval
            plane_normal_ = es3f.eigenvectors().col(i_normal).real();
        }
        if (lambda_test > max_lambda) {
            max_lambda = lambda_test;
            i_major_axis = ivec; //this index is closer to index of min eval
            major_axis_ = es3f.eigenvectors().col(i_major_axis).real();
        }        
    }
    // at this point, we have the minimum eval in "min_lambda", and the plane normal
    // (corresponding evec) in "est_plane_normal"/
    // these correspond to the ith entry of i_normal
    cout<<"min eval is "<<min_lambda<<", corresponding to component "<<i_normal<<endl;
    cout<<"corresponding evec (est plane normal): "<<plane_normal_.transpose()<<endl;
    cout<<"max eval is "<<max_lambda<<", corresponding to component "<<i_major_axis<<endl;
    cout<<"corresponding evec (est major axis): "<<major_axis_.transpose()<<endl;    
    
    //cout<<"correct answer is: "<<normal_vec.transpose()<<endl;
    //plane_dist = plane_normal.dot(centroid_);
    //cout<<"est plane distance from origin = "<<est_dist<<endl;
    //cout<<"correct answer is: "<<dist<<endl;
    //cout<<endl<<endl;  
}

//get pts from cloud, pack the points into an Eigen::MatrixXf, then use above
void MerryPclutils::compute_plane_normal_and_major_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr) {
    Eigen::MatrixXf points_mat;
    Eigen::Vector3f cloud_pt;
    //populate points_mat from cloud data;

    int npts = input_cloud_ptr->points.size();
    points_mat.resize(3, npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        cloud_pt = input_cloud_ptr->points[i].getVector3fMap();
        points_mat.col(i) = cloud_pt;
    }
    compute_plane_normal_and_major_axis(points_mat);
}

bool MerryPclutils::isRed(int r, int g, int b, int tolerance) {
    // standard red rgb code
    int standard_r = 255;
    int standard_g = 0;
    int standard_b = 0;

    if(abs(standard_r - r) < tolerance && abs(standard_g - g) < tolerance && abs(standard_b - b) < tolerance) {
        ROS_INFO("RED");
        return true;
    }
    return false;
}

bool MerryPclutils::isGreen(int r, int g, int b, int tolerance) {
    // standard red rgb code
    int standard_r = 255;
    int standard_g = 0;
    int standard_b = 0;

    if(abs(standard_r - r) < tolerance && abs(standard_g - g) < tolerance && abs(standard_b - b) < tolerance) {
        ROS_INFO("GREEN");
        return true;
    }
    return false;
}


bool MerryPclutils::isBlue(int r, int g, int b, int tolerance) {
    // standard red rgb code
    int standard_r = 255;
    int standard_g = 0;
    int standard_b = 0;

    if(abs(standard_r - r) < tolerance && abs(standard_g - g) < tolerance && abs(standard_b - b) < tolerance) {
        ROS_INFO("BLUE");
        return true;
    }
    return false;
}

bool MerryPclutils::isBlack(int r, int g, int b, int tolerance) {
    // standard red rgb code
    int standard_r = 0;
    int standard_g = 0;
    int standard_b = 0;

    if(abs(standard_r - r) < tolerance && abs(standard_g - g) < tolerance && abs(standard_b - b) < tolerance) {
        ROS_INFO("BLACK");
        return true;
    }
    return false;
}

bool MerryPclutils::isWhite(int r, int g, int b, int tolerance) {
    // standard red rgb code
    int standard_r = 255;
    int standard_g = 255;
    int standard_b = 255;

    if(abs(standard_r - r) < tolerance && abs(standard_g - g) < tolerance && abs(standard_b - b) < tolerance) {
        ROS_INFO("WHITE");
        return true;
    }
    return false;
}

bool MerryPclutils::isWoodcolor(int r, int g, int b, int tolerance) {
    // standard red rgb code
    int standard_r = 255;
    int standard_g = 228;
    int standard_b = 181;

    if(abs(standard_r - r) < tolerance && abs(standard_g - g) < tolerance && abs(standard_b - b) < tolerance) {
        ROS_INFO("WOODCOLOR");
        return true;
    }
    return false;
}

//need to fix this to put proper frame_id in header
void MerryPclutils::transform_cloud(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr) {
    output_cloud_ptr->header = input_cloud_ptr->header;
    output_cloud_ptr->is_dense = input_cloud_ptr->is_dense;
    output_cloud_ptr->width = input_cloud_ptr->width;
    output_cloud_ptr->height = input_cloud_ptr->height;
    int npts = input_cloud_ptr->points.size();
    cout << "transforming npts = " << npts << endl;
    output_cloud_ptr->points.resize(npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        output_cloud_ptr->points[i].getVector3fMap() = A * input_cloud_ptr->points[i].getVector3fMap();
    }
}


double MerryPclutils::distance_between(Eigen::Vector3f pt1, Eigen::Vector3f pt2) {
    Eigen::Vector3f pt = pt1 - pt2;
    double distance = pt.norm();//pt(0)*pt(0) + pt(1)*pt(1) + pt(2)*pt(2);
    return distance;
}


bool MerryPclutils::isWithinRadius(Eigen::Vector3f pt, Eigen::Vector3f centroid, double radius) {
    if(fabs(pt[0]-centroid[0]) < radius && fabs(pt[1]-centroid[1]) < radius) return true;
    return false;
}




