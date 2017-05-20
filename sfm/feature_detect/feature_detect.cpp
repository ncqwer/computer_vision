#include "../feature_detect.h"
#include "../control_point_detect.h"

void featurePointDetect(const std::vector<std::string>& img_filenames,
						std::vector<std::vector<cv::KeyPoint>>& feature_pts,
						std::vector<cv::Mat>& descriptors)
{
    cv::SurfFeatureDetector detector(400);
    cv::SurfDescriptorExtractor extractor;

    for(auto &img_filename : img_filenames)
    {
    	cv::Mat img = cv::imread(img_filename,CV_LOAD_IMAGE_GRAYSCALE);

    	std::vector<cv::KeyPoint> keypts;
    	detector.detect(img,keypts);

    	cv::Mat descriptor;
    	extractor.compute(img,keypts,descriptor);

    	feature_pts.push_back(keypts);
    	descriptors.push_back(descriptor);
    }
}

void featurePointMatch(const std::vector<cv::Mat>& descriptors,
   					   std::vector<std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_index)
{
	std::map<FeaturePtIndex,
			std::shared_ptr<std::set<FeaturePtIndex>>>
							feature_pts_relation;
	for(size_t i = 0; i < descriptors.size(); ++i)
	{
		featurePointMatch_SingleImg(descriptors[i],i,descriptors,feature_pts_relation);
	}
	flatFeaturePoint(feature_pts_relation,feature_pts_index);
}


void featurePointMatch_SingleImg(const cv::Mat& left_descriptor,
							     size_t single_index,
							     const std::vector<cv::Mat>& descriptors,
							     std::map<FeaturePtIndex,std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_relation)
{
	cv::FlannBasedMatcher matcher;
	for(size_t i = 0; i < descriptors.size(); ++i)
	{
		if(i != single_index)
		{
			// std::cout<<"index:"<<single_index<<" i:"<<i<<std::endl;
			if( i == 9)
				int hsj =1;
			//get good_matches
		 	std::vector<cv::DMatch> matches;
		  	matcher.match( left_descriptor, descriptors[i], matches );

		  	double max_dist = 0; double min_dist = 100;

		  	for( int n = 0; n < left_descriptor.rows; n++ )
		  	{ 
		  		double dist = matches[n].distance;
		    	if( dist < min_dist ) min_dist = dist;
		    	if( dist > max_dist ) max_dist = dist;
		  	}

		  	std::vector<cv::DMatch> good_matches;

		  	for( int n = 0; n < left_descriptor.rows; n++ )
		  	{ 
		  		if( matches[n].distance <= std::max(2*min_dist, 0.02) )
		    	{ 
		    		good_matches.push_back( matches[n]); 
		    	}
		  	}
		  	for(auto &good_match :good_matches)
		  	{
				FeaturePtIndex left = std::make_pair(single_index,good_match.queryIdx);
				FeaturePtIndex right = std::make_pair(i,good_match.trainIdx);
				auto iter_l = feature_pts_relation.find(left);
				auto iter_r = feature_pts_relation.find(right);
				if(iter_r != feature_pts_relation.end())
				{
					// iter_r->second->insert(left);

				}
				else if(iter_l != feature_pts_relation.end())
				{
					iter_l->second->insert(right);
				}
				else
				{
					auto value = std::make_shared<std::set<FeaturePtIndex>>();
					value->insert(right);
					feature_pts_relation[left] = value; 
				}
		  	}
		}
	}
}

void flatFeaturePoint(std::map<FeaturePtIndex,std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_relation,
					  std::vector<std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_index)
{
	size_t count = 0;
	for(auto it = feature_pts_relation.begin();
		it !=  feature_pts_relation.end();
		++it)
	{
		if(count == 16)
			int hsj =1;
		auto key = it->first;
		it->second->insert(key);
		feature_pts_index.push_back(it->second);
		++count;
	}
}

void bundle(const std::vector<std::vector<cv::KeyPoint>>& feature_pts,
			const std::vector<std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_index,
			const double* camera,
			const double* vecs,
			double* grand_pts)
{
	ceres::Problem problem;

    ceres::LossFunction* loss_function = new ceres::HuberLoss(4); 
	size_t count = 0;
	for(auto &feature_pt_index : feature_pts_index)
	{
		auto begin = feature_pt_index->begin();
		auto end = feature_pt_index->end();
		for(;begin != end; ++begin)
		{
			auto img_x = feature_pts[begin->first][begin->second].pt.x;
			auto img_y = feature_pts[begin->first][begin->second].pt.y;
			Eigen::Matrix<double,3,4> essential;
			//in: photo_index
			getEssentialMatrix(begin->first,camera,vecs,essential);

			ceres::CostFunction* cost_function = 
				ReprojectionErrorForward::Create(img_x/100,img_y/100,essential);
			problem.AddResidualBlock(cost_function,
									 loss_function,
									 grand_pts + 3*count);
		}
		++count;
	}
	ceres::Solver::Options ceres_config_options;
    ceres_config_options.minimizer_progress_to_stdout = false;
    ceres_config_options.logging_type = ceres::SILENT;
    // ceres_config_options.num_threads = 1;
    ceres_config_options.preconditioner_type = ceres::JACOBI;
    ceres_config_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // ceres_config_options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;

	ceres::Solver::Summary summary;
	ceres::Solve(ceres_config_options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";
}

void getEssentialMatrix(size_t photo_index,
						const double* camera,
						const double* vecs,
						Eigen::Matrix<double,3,4>& essential)
{
	Eigen::Matrix<double,3,4> intrinsic; 
	intrinsic = Eigen::Matrix<double,3,4>::Zero();
	double u0 = camera[0];
	double v0 = camera[1];
	double focal = camera[2]; 
	intrinsic(0,0) = focal;
	intrinsic(1,1) = focal;
	intrinsic(0,2) = u0;
	intrinsic(1,2) = v0;
	intrinsic(2,2) = 1;

	Eigen::Matrix<double,4,4> external;
	external = Eigen::Matrix<double,4,4>::Zero();
	//get external;
	double X = *(vecs + 6*photo_index + 0);
	double Y = *(vecs + 6*photo_index + 1);
	double Z = *(vecs + 6*photo_index + 2);
	double phi = *(vecs + 6*photo_index + 3);
	double omega = *(vecs + 6*photo_index + 4);
	double kappa = *(vecs + 6*photo_index + 5);
	Eigen::Matrix3d R;
	getR(phi,omega,kappa,R);
	external.block<3,3>(0,0) = R;
	external(0,3) = X;
	external(1,3) = Y;
	external(2,3) = Z;
	external(3,3) = 1;
	essential = intrinsic * external;
}

void getR(double phi,double omega,double kappa,
		  Eigen::Matrix<double,3,3>& R)
{
	using std::sin;
	using std::cos;
	Eigen::Matrix3d phi_m, omega_m, kappa_m;
	omega_m << 1, 0, 0,
	        0, cos(omega), -sin(omega),
	        0, sin(omega), cos(omega);

	phi_m << cos(phi), 0, -sin(phi),
	      0, 1, 0,
	      sin(phi), 0, cos(phi);

	kappa_m << cos(kappa), -sin(kappa), 0,
	        sin(kappa), cos(kappa), 0,
	        0, 0, 1;
	R = phi_m * omega_m * kappa_m;
}

void getInitialValue(const std::vector<std::vector<cv::KeyPoint>>& feature_pts,
			const std::vector<std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_index,
			const double* camera,
			const double* vecs,
			double* grand_pts)
{
	size_t count = 0;
	for(auto &feature_pt_index : feature_pts_index)
	{
		auto left = feature_pt_index->begin();
		auto right = left;
		int half_size = feature_pt_index->size()/2;
		std::advance(right,half_size);
	
		auto left_x = feature_pts[left->first][left->second].pt.x;
		auto left_y = feature_pts[left->first][left->second].pt.y;
		auto right_x = feature_pts[right->first][right->second].pt.x;
		auto right_y = feature_pts[right->first][right->second].pt.y;
		//in: photo_index
		forward(left->first,left_x/100,left_y/100,
				right->first,right_x/100,right_y/100,
				camera,vecs,count,grand_pts);

		++count;
	}	
}

void forward(size_t left_index,double left_x,double left_y,
			 size_t right_index,double right_x,double right_y,
			 const double* camera, const double* vecs,
			 size_t pt_index,
			 double* grand)
{
	double u0 = camera[0];
	double v0 = camera[1];
	double focal = camera[2];

	double left_X = *(vecs + 6*left_index + 0);
	double left_Y = *(vecs + 6*left_index + 1);
	double left_Z = *(vecs + 6*left_index + 2);
	double left_phi = *(vecs + 6*left_index + 3);
	double left_omega = *(vecs + 6*left_index + 4);
	double left_kappa = *(vecs + 6*left_index + 5);
	Eigen::Matrix3d left_R;
	getR(left_phi,left_omega,left_kappa,left_R);

	Eigen::Matrix<double,3,1> l;
	l<<(left_x-u0),(left_y-v0),-focal;

	auto L = left_R*l;

	double right_X = *(vecs + 6*right_index + 0);
	double right_Y = *(vecs + 6*right_index + 1);
	double right_Z = *(vecs + 6*right_index + 2);
	double right_phi = *(vecs + 6*right_index + 3);
	double right_omega = *(vecs + 6*right_index + 4);
	double right_kappa = *(vecs + 6*right_index + 5);
	Eigen::Matrix3d right_R;
	getR(right_phi,right_omega,right_kappa,right_R);

	Eigen::Matrix<double,3,1> r;
	l<<(right_x-u0),(right_y-v0),-focal;

	auto R = right_R*r;

	double B_x = right_X - left_X;
	double B_y = right_Y - left_Y;
	double B_z = right_Z - left_Z;

	double N1 = (B_x*R(2) - B_z*R(0))/(L(0)*R(2) - L(2)*R(0));
	double N2 = (B_x*L(2) - B_z*L(0))/(L(0)*R(2) - L(2)*R(0));

	double X = left_X + B_x + N2*R(0);
	double Y = left_Y + B_y + N2*R(1);	
	double Z = left_Z + B_z + N2*R(2);	

	if(X+1 == X-1 || Y+1 == Y-1 || Z+1 == Z-1)
		std::cout<<"index:"<<pt_index<<"leftx:"<<left_x<<"lefty:"<<left_y<<std::endl;
	*(grand + pt_index*3 + 0) = X;
	*(grand + pt_index*3 + 1) = Y;
	*(grand + pt_index*3 + 2) = Z;

	if(pt_index == 0)
	{
		std::cout<<"N1: "<<N1<<std::endl
				 <<"N2: "<<N2<<std::endl
				 <<"Bx: "<<B_x<<std::endl
				 <<"By: "<<B_y<<std::endl
				 <<"Bz: "<<B_z<<std::endl;
	}
}