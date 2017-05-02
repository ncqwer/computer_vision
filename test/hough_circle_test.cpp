/********************************************************************************
*
*
*  This program is demonstration for ellipse fitting. Program finds
*  contours and approximate it by ellipses.
*
*  Trackbar specify threshold parametr.
*
*  White lines is contours. Red lines is fitting ellipses.
*
*
*  Autor:  Denis Burenkov.
*
*
*
********************************************************************************/
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
using namespace cv;
using namespace std;

// static void help()
// {
//     cout <<
//             "\nThis program is demonstration for ellipse fitting. The program finds\n"
//             "contours and approximate it by ellipses.\n"
//             "Call:\n"
//             "./fitellipse [image_name -- Default stuff.jpg]\n" << endl;
// }

int sliderPos = 100;
int max_w = 1000;
int min_w = 20;
Mat image;

void processImage(int, void*);

int main( int argc, char** argv )
{
    // const char* filename = argc == 2 ? argv[1] : (char*)"stuff.jpg";
    image = imread(argv[1], 0);
    if( image.empty() )
    {
        // cout << "Couldn't open image " << filename << "\nUsage: fitellipse <image_name>\n";
        return 0;
    }
    namedWindow("source", WINDOW_NORMAL);
    imshow("source", image);
    namedWindow("result", WINDOW_NORMAL);

    // Create toolbars. HighGUI use.
    createTrackbar( "threshold", "result", &sliderPos, 255, processImage );
    createTrackbar( "max", "result", &max_w, 2000, processImage );
    createTrackbar( "min", "result", &min_w, 255, processImage );
    processImage(0, 0);

    // Wait for a key stroke; the same function arranges events processing
    waitKey();
    return 0;
}

// Define trackbar callback functon. This function find contours,
// draw it and approximate it by ellipses.
void processImage(int /*h*/, void*)
{
    vector<vector<Point> > contours;
    Mat bimage = image >= sliderPos;

    findContours(bimage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    Mat cimage = Mat::zeros(bimage.size(), CV_8UC3);
    int num=0;
    std::vector<Point> hsj;
    for(size_t i = 0; i < contours.size(); i++)
    {
        size_t count = contours[i].size();
        if( count < 6 )
            continue;

        Mat pointsf;
        Mat(contours[i]).convertTo(pointsf, CV_32F);
        RotatedRect box = fitEllipse(pointsf);

        if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*30 )
            continue;
        if( MAX(box.size.width, box.size.height) > max_w || MIN(box.size.width, box.size.height) < min_w)
            continue;
        // drawContours(cimage, contours, (int)i, Scalar::all(255), 1, 8);
        ++num;
        // ellipse(cimage, box, Scalar(0,0,255), 1, CV_AA);
        ellipse(cimage, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,0), 1, CV_AA);
        // Point2f vtx[4];
        // box.points(vtx);
        // for( int j = 0; j < 4; j++ )
        //     line(cimage, vtx[j], vtx[(j+1)%4], Scalar(0,255,0), 1, CV_AA);
        cv::Point2f pt(box.center.x,box.center.y);
        hsj.push_back(pt);
    }
    RotatedRect hsj_box = fitEllipse(hsj);
    ellipse(cimage, hsj_box.center, hsj_box.size*0.5f, hsj_box.angle, 0, 360, Scalar(0,255,0), 1, CV_AA);
    int r = cimage.rows;
    int w = cimage.cols;
    int half_r = r/2;
    int half_w = w/2;
    line(cimage,Point(half_w,0),Point(half_w,r),Scalar(0,255,0));
    cout<<"point's number:"<<num<<endl;
    imshow("result", cimage);
}