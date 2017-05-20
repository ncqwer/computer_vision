#ifndef _FEATURE_DETECT_
#define _FEATURE_DETECT_ 

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

#endif