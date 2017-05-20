#include "../inc/calibration_solution.h"

void CalibrationSolution::extractControlPoints()
{
	_cptss.clear();
	size_t process_index = 0;
	//==================extract all cpt in imgs=============
	for(auto &img_filename : _img_filenames)
	{
		cv::Mat img = cv::imread(img_filename,0);
		std::vector<ControlPoint> cpts;
		std::string message = 
						std::string("process image:") + 
						std::to_string(process_index);
		moduleCall(message,std::cout,
				   controlPointDetect,
				   img,cpts);
		for(auto &cpt : cpts)
		{
			cpt.img_index = process_index;
		}
		_cptss.push_back(cpts);
		++process_index;
	}
}

bool CalibrationSolution::getInitialValue()
{
	cv::Size sz = cv::imread(_img_filenames[0]).size();
	cv::Mat camera_m;
	cv::Mat dist_coeffs;
	std::vector<cv::Mat> r_vecs;
	std::vector<cv::Mat> t_vecs;

	//=============prepare data==============
	std::vector<std::vector<cv::Point2f>> img_ptss;
	std::vector<std::vector<cv::Point3f>> grand_ptss;
	for(auto &cpts : _cptss)
	{
		std::vector<cv::Point2f> img_pts;
		std::vector<cv::Point3f> grand_pts;
		for(auto &cpt : cpts)
		{
			img_pts.push_back(cv::Point2f(cpt.circle.x,cpt.circle.y));
			grand_pts.push_back(_gcps[cpt.cpt_index]);
		}
		img_ptss.push_back(img_pts);
		grand_ptss.push_back(grand_pts);
	}

	for(size_t i = 0; i< img_ptss.size();++i)
	{
		std::cout<<" img: "<<img_ptss[i].size()
				 <<" grand: "<<grand_ptss[i].size()<<std::endl;
	}
	calibrateCamera(grand_ptss,img_ptss,sz,
					camera_m,dist_coeffs,
					r_vecs,t_vecs,CV_CALIB_FIX_PRINCIPAL_POINT|CV_CALIB_FIX_ASPECT_RATIO);

	double fx = camera_m.at<double>(0,0);
	double fy = camera_m.at<double>(1,1);
	double u0 = camera_m.at<double>(0,2);
	double v0 = camera_m.at<double>(1,2);

	double k1 = dist_coeffs.at<double>(0);
	double k2 = dist_coeffs.at<double>(1);

	_camera[0] = fx;
	_camera[1] = u0;
	_camera[2] = v0;
	_camera[3] = k1;
	_camera[4] = k2;

	size_t img_size = r_vecs.size();
	_vecs = new double[img_size * 6];
	for(size_t i = 0; i< img_size; ++i)
	{
		double X = t_vecs[i].at<float>(0);
		double Y = t_vecs[i].at<float>(1);
		double Z = t_vecs[i].at<float>(2);

		double phi = r_vecs[i].at<float>(0);
		double omega = r_vecs[i].at<float>(1);
		double kappa = r_vecs[i].at<float>(2);

		*(_vecs + i*6 + 0) = X;
		*(_vecs + i*6 + 1) = Y;
		*(_vecs + i*6 + 2) = Z;
		*(_vecs + i*6 + 3) = phi;
		*(_vecs + i*6 + 4) = omega;
		*(_vecs + i*6 + 5) = kappa;
	}
}

void CalibrationSolution::bundle()
{
	ceres::Problem problem;
	// for(int i=0;i<4;++i)
	// {

 //    ceres::CostFunction* cost_function =
 //        ReprojectionError::Create(x[i]/1000,y[i]/1000,X[i],Y[i],Z[i]);
 //    problem.AddResidualBlock(cost_function,
 //                             NULL /* squared loss */,
 //                             haha);
	// }
	size_t process_index = 0;
	for(auto &cpts : _cptss)
	{
		for(auto &cpt : cpts)
		{
			double obversed_x = cpt.circle.x;
			double obversed_y = cpt.circle.y;
			double X = _gcps[cpt.cpt_index].x;
			double Y = _gcps[cpt.cpt_index].y;
			double Z = _gcps[cpt.cpt_index].z;
			ceres::CostFunction* cost_function = 
				ReprojectionError::Create(obversed_x/1000,obversed_y/1000,
										  X,Y,Z);
			problem.AddResidualBlock(cost_function,
									 NULL,
									 _camera,
									 _vecs + 6*process_index);
		}
		++process_index;
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
}


void CalibrationSolution::readFile(const std::string& img_locations,
			  					   const std::string& cpt_locations)
{
	std::ifstream img_in(img_locations);
	std::string img_location;
	size_t count = 0;
	while(img_in>>img_location)
	{
		// if(count ==) break;
		_img_filenames.push_back(img_location);
		++count;
	}

	std::ifstream gcp_in(cpt_locations);
	double temp1;double temp2;
	gcp_in>>temp1>>temp2;
	double index_temp;double size_temp;double zero_temp;
	for(size_t i = 0; i<60;++i)
	{
		double X;double Y; double Z;
		gcp_in>>index_temp>>X>>Y>>Z>>size_temp>>zero_temp;
		_gcps.push_back(cv::Point3f(X,Y,Z));
	}
}

void CalibrationSolution::run()
{
	extractControlPoints();
	getInitialValue();
	// bundle();
	std::ofstream f_out("dst_in.txt");
	f_out<<"focal:"<<_camera[0]<<std::endl
		 <<"u0:"<<_camera[1]<<std::endl
		 <<"v0:"<<_camera[2]<<std::endl
		 <<"k1:"<<_camera[3]<<std::endl
		 <<"k2:"<<_camera[4]<<std::endl;
	std::ofstream out("dst2.txt");
	for(size_t i=0;i<30;++i)
	{
		out<<" X:"<<*(_vecs + i*6 + 0)
		   <<" Y:"<<*(_vecs + i*6 + 1)
		   <<" Z:"<<*(_vecs + i*6 + 2)
		   <<" phi:"<<*(_vecs + i*6 + 3)
		   <<" omega:"<<*(_vecs + i*6 + 4)
		   <<" kappa:"<<*(_vecs + i*6 + 5)<<std::endl;
	}
}