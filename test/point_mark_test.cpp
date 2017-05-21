// #include <iostream>
// #include <algorithm>
// #include <opencv2/opencv.hpp>
// #include <string>

// #define PI 3.14159265
// struct ControlPoint
// {
// 	cv::Point3f circle;
// 	double angle;
// 	double distance;
// 	size_t angle_index;
// 	size_t distance_index;
// 	size_t cpt_index;
// 	size_t img_index;
// 	ControlPoint(){}
// 	ControlPoint(const cv::Point3f& c,
// 				 const double a,
// 				 const double d,
// 				 const size_t t_i,
// 				 const size_t d_i,
// 				 const size_t pt_i,
// 				 const size_t img_i):
// 		circle(c),
// 		angle(a),
// 		distance(d),
// 		angle_index(t_i),
// 		distance_index(d_i),
// 		cpt_index(pt_i),
// 		img_index(img_i)
// 	{}
// };

// bool byAngle(const ControlPoint& lhs,
// 		   const ControlPoint& rhs)
// {
// 	return lhs.angle < rhs.angle;
// }

// bool byDistance(const ControlPoint& lhs,
// 				const ControlPoint& rhs)
// {
// 	return lhs.distance < rhs.distance;
// }

// bool byRadius(const ControlPoint& lhs,
// 				const ControlPoint& rhs)
// {
// 	return lhs.circle.z < rhs.circle.z;
// }
// //in:image
// //out:circles(std::vector<cv::Point3f>)
// //	0 -> x
// //	1 -> y
// //	2 -> r
// void getControlPoint(const cv::Mat& image,
// 					 std::vector<cv::Point3f>& circles,
// 					 cv::Point& center)
// {
// 	circles.clear();
//     std::vector<std::vector<cv::Point> > contours;
//     cv::Mat bimage = image >= 87;

//     cv::findContours(bimage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
//     // std::vector<cv::Point> big_eclipse;
//     for(size_t i = 0; i < contours.size(); i++)
//     {
//         size_t count = contours[i].size();
//         if( count < 6 )
//             continue;

//         cv::Mat pointsf;
//         cv::Mat(contours[i]).convertTo(pointsf, CV_32F);
//         cv::RotatedRect box = cv::fitEllipse(pointsf);

//         if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*30 )
//             continue;
//         if( MIN(box.size.width, box.size.height) < 20 || MAX(box.size.width, box.size.height) > 300)
//             continue;
//         cv::Point3f circle;
//         circle.x = box.center.x;
//         circle.y = box.center.y;
//         circle.z = (box.size.width + box.size.height)/2;
//         circles.push_back(circle);
//         // big_eclipse.push_back(cv::Point(box.center.x,box.center.y));
//     }
//     // cv::RotatedRect big = cv::fitEllipse(big_eclipse);
//     // center.x = big.center.x;
//     // center.y = big.center.y;
// }

// //in: image_center
// //in: circles
// //out: control_point_groups
// //reason:
// //tan(a)
// //flag_x
// void getControlPointGroups(const cv::Point2f& image_center,
// 						   const std::vector<cv::Point3f>& circles,
// 						   std::vector<std::vector<ControlPoint>>& control_point_groups)
// {
// 	double thresould = 3;
// 	std::vector<ControlPoint> cpt1;
// 	std::vector<ControlPoint> cpt2;
// 	for(auto &circle :circles)
// 	{
// 		if(circle.x > image_center.x)
// 			cpt1.push_back(ControlPoint(circle,0,0,0,0,0,0));
// 		else
// 			cpt2.push_back(ControlPoint(circle,0,0,0,0,0,0));
// 	}

// 	for(auto &cpt :cpt1)
// 	{
// 		double dx = cpt.circle.x - image_center.x;
// 		double dy = cpt.circle.y - image_center.y;
// 		double distance = dx*dx + dy*dy;
// 		double a = std::atan2(-dy,dx)* 180 / PI;
// 		cpt.angle = a;
// 		cpt.distance = distance;
// 	}

// 	std::sort(cpt1.begin(),cpt1.end(),byAngle);
// 	double last_tan_1 = cpt1[0].angle;
// 	std::vector<ControlPoint> group;
// 	for(auto &cpt :cpt1)
// 	{
// 		std::cout<<"angle is:"<<cpt.angle<<std::endl;
// 		double diff = fabs(cpt.angle - last_tan_1);
// 		if(fabs(cpt.angle - last_tan_1) > thresould)
// 		{
// 			control_point_groups.push_back(group);
// 			group.clear();
// 		}
// 		group.push_back(cpt);
// 		last_tan_1 = cpt.angle;
// 	}
// 	control_point_groups.push_back(group);
// 	std::cout<<"====================="<<std::endl;
// 	for(auto &cpt :cpt2)
// 	{
// 		double dx = cpt.circle.x - image_center.x;
// 		double dy = cpt.circle.y - image_center.y;
// 		double distance = dx*dx + dy*dy;
// 		double a = std::atan2(dy,dx)* 180 / PI;
// 		if( a > -140 && a < -139)
// 			int hsj = 0;
// 		cpt.angle = a;
// 		cpt.distance = distance;
// 	}
	
// 	std::sort(cpt2.begin(),cpt2.end(),byAngle);
// 	double last_tan_2 = cpt2[0].angle;
// 	group.clear();
// 	for(auto &cpt :cpt2)
// 	{
// 		std::cout<<"angle is:"<<cpt.angle<<std::endl;
// 		if(abs(cpt.angle - last_tan_2) > thresould)
// 		{
// 			control_point_groups.push_back(group);
// 			group.clear();
// 		}
// 		group.push_back(cpt);
// 		last_tan_2 = cpt.angle;
// 	}
// 	control_point_groups.push_back(group);

// }

// //in:control_point_groups
// //out:cpts (std::vector<ControlPoint>)
// void getControlPointIndex(std::vector<std::vector<ControlPoint>>& control_point_groups,
// 						  std::vector<ControlPoint>& control_points)
// {
// 	//sort
// 	for(auto &group : control_point_groups)
// 	{
// 		std::sort(group.begin(),group.end(),byDistance);
// 		size_t distance_index = 0;
// 		for(auto &cpt : group)
// 		{
// 			cpt.distance_index = distance_index;
// 			++distance_index;
// 		}
// 		std::sort(group.begin(),group.end(),byAngle);
// 		size_t angle_index = 0;
// 		for(auto &cpt : group)
// 		{
// 			cpt.angle_index = angle_index;
// 			++angle_index;
// 		}
// 	}
// 	//store
// 	control_points.clear();
// 	for(auto &group :control_point_groups)
// 	{
// 		for(auto &cpt : group)
// 		{
// 			control_points.push_back(cpt);
// 		}
// 	}
// }

// int main(int argc, char const *argv[])
// {
// 	cv::Mat img = cv::imread("Buddha_013.JPG",0);

// 	std::vector<cv::Point3f> circles;
// 	cv::Point center;
// 	getControlPoint(img,circles,center);
// 	int cpt_size = circles.size();
// 	double center_x = 0;
// 	double center_y = 0;
// 	for(auto &cpt : circles)
// 	{
// 		center_x += cpt.x/cpt_size;
// 		center_y += cpt.y/cpt_size;
// 	}
// 	std::vector<std::vector<ControlPoint>> cpt_groups;
// 	getControlPointGroups(cv::Point2f(center_x,center_y),circles,cpt_groups);
// 	for(auto &group : cpt_groups)
// 	{
// 		std::cout<<"group'size is:"<<group.size()<<std::endl;
// 		for(auto &cpt : group)
// 		{
// 			std::string str = std::to_string(cpt.angle);
// 			cv::putText(img,str,cv::Point(cpt.circle.x,cpt.circle.y),cv::FONT_HERSHEY_TRIPLEX,1,cv::Scalar(0,255,0));
// 		}
// 	}

// 	std::vector<ControlPoint> cpts;
// 	getControlPointIndex(cpt_groups,cpts);
// 	cv::namedWindow("try",cv::WINDOW_NORMAL);
// 	cv::imshow("try",img);
// 	cv::waitKey();
// 	return 0;
// }
// 


//=======================================================
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include "../sfm/sfm.h"

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
	Mat img = imread("Buddha_001.JPG",0);
	Mat img_show = imread("Buddha_001.JPG");

	std::vector<ControlPoint> cpts;
	string message = "control point detect begin";
	moduleCall(message,std::cout,controlPointDetect,img,cpts);
	for(auto &cpt : cpts)
	{
		std::string str = std::to_string(cpt.cpt_index);
		cv::putText(img_show,str,cv::Point(cpt.circle.x,cpt.circle.y),cv::FONT_HERSHEY_TRIPLEX,1,cv::Scalar(0,255,0));
	}
	namedWindow("dst",WINDOW_NORMAL);
	imshow("dst",img_show);
	imwrite("marked.jpg",img_show);
	waitKey();
	return 0;
}