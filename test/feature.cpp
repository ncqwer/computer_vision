#include <iostream>
#include <map>
#include <set>
#include <memory>
#include <fstream>
#include <iterator>


#include "opencv2/opencv_modules.hpp"
# include "opencv2/core/core.hpp"
# include "opencv2/features2d/features2d.hpp"
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/nonfree/features2d.hpp"

#include "ceres/ceres.h"
#include <Eigen/Dense>

typedef std::pair<size_t,size_t> FeaturePtIndex;


struct ReprojectionErrorForward{
  ReprojectionErrorForward(double observed_x, 
  					double observed_y,
  					const Eigen::Matrix<double,3,4>& essential)
      :_observed_x(observed_x), 
       _observed_y(observed_y), 
       _essential(essential)
       {}

  template <typename T>
  bool operator()(const T* const grand_pts,
                  T* residuals) const 
  {
  	Eigen::Matrix<T,4,1> grand;
  	grand(0) = *(grand_pts + 0),
  	grand(1) = *(grand_pts + 1),
  	grand(2) = *(grand_pts + 2),
  	grand(3) = T(1);

  	//replace the essential Matix with type T;
  	Eigen::Matrix<T,3,4> essential;
  	for(size_t i = 0; i < 3; ++i)
  	{
  		for(size_t j = 0; j < 4; ++j)
  		{
		  	essential(i,j) = T(_essential(i,j));
  		}
  	}

  	Eigen::Matrix<T,3,1> image;
  	image = essential*grand;

	residuals[0] = image(0) - T(_observed_x);
	residuals[1] = image(1) - T(_observed_y);  
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const double observed_x,
                                      const double observed_y,
                                      const Eigen::Matrix<double,3,4>& essential) {
     return (new ceres::AutoDiffCostFunction<ReprojectionErrorForward, 2, 3>(
                 new ReprojectionErrorForward(observed_x, observed_y,essential)));
   }

  double _observed_x;
  double _observed_y;
  Eigen::Matrix<double,3,4> _essential;
};

void bundle(const std::vector<std::vector<cv::KeyPoint>>& feature_pts,
			const std::vector<std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_index,
			const double* camera,
			const double* vecs,
			double* grand_pts);

void getR(double phi,double omega,double kappa,
		  Eigen::Matrix<double,3,3>& R);

void getEssentialMatrix(size_t photo_index,
						const double* camera,
						const double* vecs,
						Eigen::Matrix<double,3,4>& essential);

void flatFeaturePoint(std::map<FeaturePtIndex,std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_relation,
					  std::vector<std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_index);

void featurePointMatch_SingleImg(const cv::Mat& left_descriptor,
							     size_t single_index,
							     const std::vector<cv::Mat>& descriptors,
							     std::map<FeaturePtIndex,std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_relation);

void featurePointMatch(const std::vector<cv::Mat>& descriptors,
   					   std::vector<std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_index);

void featurePointDetect(const std::vector<std::string>& img_filenames,
						std::vector<std::vector<cv::KeyPoint>>& feature_pts,
						std::vector<cv::Mat>& descriptors);

void getInitialValue(const std::vector<std::vector<cv::KeyPoint>>& feature_pts,
			const std::vector<std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_index,
			const double* camera,
			const double* vecs,
			double* grand_pts);

void forward(size_t left_index,double left_x,double left_y,
			 size_t right_index,double right_x,double right_y,
			 const double* camera, const double* vecs,
			 size_t pt_index,
			 double* grand);


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

int main(int argc, char const *argv[])
{
	std::vector<std::string> img_filenames;
	std::string file_base_1 = "Buddha_00";
	std::string file_base_2 = "Buddha_0";
	for(size_t i = 1; i < 31; ++i)
	{
		std::string file_name;
		if(i < 10){
			file_name = file_base_1 + 
					   std::to_string(i) +
					   std::string(".png");
		}
		else{
			file_name = file_base_2 + 
					   std::to_string(i) +
					   std::string(".png");
		}
		img_filenames.push_back(file_name);
	}
	std::vector<std::vector<cv::KeyPoint>> feature_pts;
	std::vector<cv::Mat> feature_pt_descriptors;
	featurePointDetect(img_filenames,feature_pts,feature_pt_descriptors);

	std::vector<std::shared_ptr<std::set<FeaturePtIndex>>> feature_pts_index;
	featurePointMatch(feature_pt_descriptors,feature_pts_index);
	bool flag = true;
	size_t count = 0;
	std::vector<std::shared_ptr<std::set<FeaturePtIndex>>> good_feature_pts_index;
	for(auto &pts_index : feature_pts_index)
	{
		// if(pts_index->size() > 30 || pts_index->size() <2){
			// std::cout<<count<<"error occured!"<<"size:"<<pts_index->size()<<std::endl;
		// std::cout<<"size:"<<pts_index->size()<<std::endl;
			// auto begin = pts_index->begin();
			// auto end = pts_index->end();
			// if(flag){
			// 	for(;begin != end; ++begin)
			// 	{
			// 		std::cout<<"("<<begin->first<<","<<begin->second<<")"<<std::endl;
			// 	}
			// 	flag = false;
			// }
		// }
		if(pts_index->size() > 5 && pts_index->size()<15){
			good_feature_pts_index.push_back(pts_index);
			++count;
		}
	}
	std::cout<<count<<std::endl;

	int gcp_size = good_feature_pts_index.size();

	for(auto begin = good_feature_pts_index[1]->begin();
		begin != good_feature_pts_index[1]->end();
		++begin)
	{
		std::cout<<"image: "<<begin->first<<" point: "<<begin->second<<std::endl;
	}

	double* grand = new double[gcp_size*3];

	//read data;
	double* vecs = new double[6*30];
	double* camera = new double[3];

	camera[0] = 1936/100;
	// camera[0] = 0;
	camera[1] = 1296/100;
	// camera[1] = 0;
	camera[2] = 7935.786962/100; 
	std::ifstream fin_vecs("external.txt");
	double total;
	fin_vecs>>total;
	std::string temp;
	for(int i = 0; i < 30 ; ++i)
	{
		fin_vecs>>temp
				>>*(vecs + i*6 +0)
				>>*(vecs + i*6 +1)
				>>*(vecs + i*6 +2)
				>>*(vecs + i*6 +3)
				>>*(vecs + i*6 +4)
				>>*(vecs + i*6 +5);
	}
	getInitialValue(feature_pts,good_feature_pts_index,camera,vecs,grand);
	bundle(feature_pts,good_feature_pts_index,camera,vecs,grand);

	for(size_t i = 0 ; i < gcp_size; i++)
	{
		std::cout<<" X: "<<*(grand + i*3 + 0)
				 <<" Y: "<<*(grand + i*3 + 1)
				 <<" Z: "<<*(grand + i*3 + 2)<<std::endl;
	}
	delete []grand;
	delete []vecs;
	delete []camera;
	return 0;
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