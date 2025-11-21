#ifndef STEREO_METHOD_H
#define STEREO_METHOD_H

#include "myopencv.h"

class Stereo_method
{
public:
    Stereo_method();
    float position[3];
    void Undisortion(cv::Mat&  , cv::Mat& );
    void Triangulation(cv::Point , cv::Point );

private:
    cv::Mat R, T, Coffe_L, Coffe_R, IL, IR;
    cv::Size img_size = cv:: Size(640,480);
    cv::Mat R1, P1, R2, P2;
    cv::Mat Q;
    cv::Rect roi1, roi2;
    cv::Mat map11, map12, map21, map22;
    cv::Mat imgLTemp, imgRTemp;

};

#endif // STEREO_METHOD_H
