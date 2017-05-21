#include "sparse_solution.h"

void SparseSolution::readFile(const std::string& intrinsic_file,
				  const std::string& external_file,
				  const std::string& img_locations)
{
	std::ifstream img_in(img_locations);
	std::string img_location;
	while(img_in>>img_location)
	{
		_img_filenames.push_back(img_location);
	}

	_camera = new double[3];
	std::ifstream intrinsic_in(intrinsic_file);
	std::string temp;
	std::string equal;
	intrinsic_in>>temp>>equal>>*(_camera + 0)
				>>temp>>equal>>*(_camera + 1)
				>>temp>>equal>>*(_camera + 2);
	_camera[0] += 1936;
	_camera[1] += 1296;
 	
 	_camera[0] /=100;
 	_camera[1] /=100;
 	_camera[2] /=100;

	std::ifstream external_in(external_file);
	int total;
	external_in>>total;
	total = 30;
	_vecs = new double[6*total];
	for(int i = 0; i < total ; ++i)
	{
		external_in>>temp
				>>*(_vecs + i*6 +0)
				>>*(_vecs + i*6 +1)
				>>*(_vecs + i*6 +2)
				>>*(_vecs + i*6 +3)
				>>*(_vecs + i*6 +4)
				>>*(_vecs + i*6 +5);
	}
}

void SparseSolution::run(const std::string& grand_file)
{
	std::vector<std::vector<cv::KeyPoint>> feature_pts;
	std::vector<cv::Mat> feature_pt_descriptors;
	featurePointDetect(_img_filenames,feature_pts,feature_pt_descriptors);

	std::vector<std::shared_ptr<std::set<FeaturePtIndex>>> feature_pts_index;
	featurePointMatch(feature_pt_descriptors,feature_pts_index);
	std::vector<std::shared_ptr<std::set<FeaturePtIndex>>> good_feature_pts_index;
	for(auto &pts_index : feature_pts_index)
	{
		if(pts_index->size() > 5 && pts_index->size()<15){
			good_feature_pts_index.push_back(pts_index);
		}
	}

	int gcp_size = good_feature_pts_index.size();

	_grand = new double[gcp_size*3];

	// getInitialValue(feature_pts,good_feature_pts_index,_camera,_vecs,_grand);
	bundle(feature_pts,good_feature_pts_index,_camera,_vecs,_grand);
	// write(feature_pts,good_feature_pts_index,_grand,_img_filenames,grand_file);
	std::ofstream f_dst(grand_file);
	for(size_t i = 0 ; i < gcp_size; i++)
	{
		auto ret = good_feature_pts_index[i]->begin();
		size_t img_index = ret->first;
		auto keypoint = feature_pts[img_index][ret->second];
		auto img_x = keypoint.pt.x;
		auto img_y = keypoint.pt.y;
		cv::Mat img = cv::imread(_img_filenames[img_index]);
		auto color = img.at<cv::Vec3b>(img_y,img_x);

		double X =*(_grand + i*3 + 0);
		double Y =*(_grand + i*3 + 1);
		double Z =*(_grand + i*3 + 2);
		f_dst<<std::string(" X: ")<<X
			 <<std::string(" Y: ")<<Y
			 <<std::string(" Z: ")<<Z
			 <<std::string(" B: ")<<color[0]
			 <<std::string(" G: ")<<color[1]
			 <<std::string(" R: ")<<color[2]<<std::endl;
	}
}