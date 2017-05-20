#ifndef _CONTROL_POINT_DETECT_
#define _CONTROL_POINT_DETECT_ 

#include <vector>
#include <opencv2/opencv.hpp>

#define PI 3.14159265

struct ControlPoint
{
	cv::Point3f circle;
	double angle;
	double distance;
	size_t radius_index;
	size_t cpt_index;
	size_t img_index;
	std::string pattern;
	ControlPoint(){}
	ControlPoint(const cv::Point3f& c,
				 const double a,
				 const double d,
				 const size_t r_i,
				 const size_t pt_i,
				 const size_t img_i,
				 const std::string& p=""):
		circle(c),
		angle(a),
		distance(d),
		radius_index(r_i),
		cpt_index(pt_i),
		img_index(img_i),
		pattern(p)
	{}
};

bool byDistance(const ControlPoint& lhs,
		   const ControlPoint& rhs);

bool byAngle(const ControlPoint& lhs,
		   const ControlPoint& rhs);

bool byRadius(const ControlPoint& lhs,
		   const ControlPoint& rhs);

size_t calculateIndex(size_t radius_index,const std::string& pattern);


void controlPointDetect(const cv::Mat& img,
					    std::vector<ControlPoint>& cpts);

void getControlPoint(const cv::Mat& image,
					 std::vector<cv::Point3f>& circles);

void getControlPointGroups(const cv::Point2f& image_center,
						   const std::vector<cv::Point3f>& circles,
						   std::vector<std::vector<ControlPoint>>& control_point_groups);

void fillControlPointIndex(std::vector<std::vector<ControlPoint>>& control_point_groups,
						  std::vector<ControlPoint>& control_points);


#endif