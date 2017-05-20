#include <iostream>
#include <string>


#include "../sfm/sfm.h"


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
	std::ifstream fin_vecs("234.txt");
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