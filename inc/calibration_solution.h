#ifndef _CALIBRATION_SOLUTION_H_
#define _CALIBRATION_SOLUTION_H_ 

#include <vector>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "ceres/ceres.h"
#include <Eigen/Dense>

#include "module.h"
#include "control_point_detect.h"


struct ReprojectionError {
  ReprojectionError(double observed_x, 
  					double observed_y,
  					double X,
  					double Y,
  					double Z)
      :_observed_x(observed_x), 
       _observed_y(observed_y), 
       _X(X),
       _Y(Y),
       _Z(Z)
       {}

  template <typename T>
  bool operator()(const T* const camera,
  				  const T* const vecs,
                  T* residuals) const 
  {
	using std::sin;
	using std::cos;
	T X_s = vecs[0];
	T Y_s = vecs[1];
	T Z_s = vecs[2];

	Eigen::Matrix<T,3,1> v_p;
	v_p(0)=X_s;v_p(1)=Y_s;v_p(2)=Z_s;
	Eigen::Matrix<T,3,1> v;
	v(0)=T(_X);v(1)=T(_Y);v(2)=T(_Z);
	v = v - v_p;
	T phi = vecs[3];
	T omega = vecs[4];
	T kappa = vecs[5];
	// T phi = 0.353238;
	// T omega = -0.068847;
	// T kappa = 0.0304287;


	Eigen::Matrix<T,3,3> phi_m, omega_m, kappa_m;
	// omega_m << 1, 0, 0,
	//         0, cos(omega), -sin(omega),
	//         0, sin(omega), cos(omega);
	omega_m(0,0)=T(1);omega_m(0,1)=T(0);omega_m(0,2)=T(0);
	omega_m(1,0)=T(0);omega_m(1,1)=T(cos(omega));omega_m(1,2)=T(-sin(omega));
	omega_m(2,0)=T(0);omega_m(2,1)=T(sin(omega));omega_m(2,2)=T(cos(omega));
	// phi_m << cos(phi), 0, -sin(phi),
	//       0, 1, 0,
	//       sin(phi), 0, cos(phi);
	phi_m(0,0)=T(cos(phi));phi_m(0,1)=T(0);phi_m(0,2)=T(-sin(phi));
	phi_m(1,0)=T(0);phi_m(1,1)=T(1);phi_m(1,2)=T(0);
	phi_m(2,0)=T(sin(phi));phi_m(2,1)=T(0);phi_m(2,2)=T(cos(phi));
	// kappa_m << cos(kappa), -sin(kappa), 0,
	//         sin(kappa), cos(kappa), 0,
	//         0, 0, 1;
	kappa_m(0,0)=T(cos(kappa));kappa_m(0,1)=T(-sin(kappa));kappa_m(0,2)=T(0);
	kappa_m(1,0)=T(sin(kappa));kappa_m(1,1)=T(cos(kappa));kappa_m(1,2)=T(0);
	kappa_m(2,0)=T(0);kappa_m(2,1)=T(0);kappa_m(2,2)=T(1);
	Eigen::Matrix<T,3,3> R = phi_m * omega_m * kappa_m;
	// std::cout<<"X_s:"<<X_s<<std::endl;
	// std::cout<<"Y_s:"<<Y_s<<std::endl;
	// std::cout<<"Z_s:"<<Z_s<<std::endl;
	// std::cout<<"X:"<<_X<<std::endl;
	// std::cout<<"Y:"<<_Y<<std::endl;
	// std::cout<<"Z:"<<_Z<<std::endl;
	// std::cout<<"obverse_x:"<<_observed_x<<std::endl;
	// std::cout<<"obverse_y:"<<_observed_y<<std::endl;
	// std::cout<<"omega:"<<omega<<std::endl;
	// std::cout<<"phi:"<<phi<<std::endl;
	// std::cout<<"kappa:"<<kappa<<std::endl;
	// std::cout<<"R:"<<std::endl
	// 		 <<R(0,0)<<" "<<R(0,1)<<" "<<R(0,2)<<std::endl
	// 		 <<R(1,0)<<" "<<R(1,1)<<" "<<R(1,2)<<std::endl
	// 		 <<R(2,0)<<" "<<R(2,1)<<" "<<R(2,2)<<std::endl;
	T focal = camera[0];
	Eigen::Matrix<T,3,1> pt = R.transpose() * v;
	T pt_x = T(-focal)*(pt(0)/pt(2));
	T pt_y = T(-focal)*(pt(1)/pt(2));
	// std::cout<<"pt_x"<<pt_x<<std::endl;
	// std::cout<<"pt_y"<<pt_y<<std::endl;

	T x0 = camera[1];
	T y0 = camera[2];
	T k1 = camera[3];
	T k2 = camera[4];

    T r2 = pt_x*pt_x + pt_y*pt_y;
    T distortion = T(1.0) + r2  * (k1 + k2  * r2);

    // Compute final projected point position.
    T predicted_x = focal * distortion * pt_x;
    T predicted_y = focal * distortion * pt_y;

	residuals[0] = pt_x - T(_observed_x - x0);
	residuals[1] = pt_y - T(_observed_y - y0);

	// std::cout<<"res_x"<<residuals[0]<<std::endl;
	// std::cout<<"res_y"<<residuals[1]<<std::endl;
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const double observed_x,
                                      const double observed_y,
                                      const double X,
                                      const double Y,
                                      const double Z) {
     return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 5, 6>(
                 new ReprojectionError(observed_x, observed_y,X,Y,Z)));
   }

  double _observed_x;
  double _observed_y;
  double _X;
  double _Y;
  double _Z;
};



class CalibrationSolution
{
public:
	CalibrationSolution(){}
	~CalibrationSolution()
	{
		if(_vecs)
		{
			delete []_vecs;
		}
	}

	CalibrationSolution(const CalibrationSolution& rhs)=delete;
	CalibrationSolution(CalibrationSolution&& rhs) noexcept=delete;

	void readFile(const std::string& img_locations,
				  const std::string& cpt_locations);
	CalibrationSolution& operator= (CalibrationSolution rhs_copy)=delete;

	void run();
private:
	void from_file(std::string imgs_loc,std::string gcps_loc);
	void extractControlPoints();
	bool getInitialValue();
	void bundle();

	std::vector<std::string> _img_filenames;
	std::vector<cv::Point3f> _gcps;

	std::vector<std::vector<ControlPoint>> _cptss;

	double* _vecs;
	double _camera[5];
};


#endif