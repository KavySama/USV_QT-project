#include "stereo_method.h"

#include<math.h>

Stereo_method::Stereo_method()
{
    IL = cv::Mat(3,3,CV_64F);
    IR = cv::Mat(3,3,CV_64F);
    //相机1内参，已转置
    IL.at<double>(0,0) = 480.8642; IL.at<double>(0,1) = 0.0730;          IL.at<double>(0,2) = 313.3252;
    IL.at<double>(1,0) = 0;        IL.at<double>(1,1) = 480.6071;   IL.at<double>(1,2) = 233.4685;
    IL.at<double>(2,0) = 0;        IL.at<double>(2,1) = 0;          IL.at<double>(2,2) = 1;
    //相机2内参，已转置
    IR.at<double>(0,0) = 484.4812; IR.at<double>(0,1) = 0.3170;          IR.at<double>(0,2) = 307.2714;
    IR.at<double>(1,0) = 0;        IR.at<double>(1,1) = 484.5331;   IR.at<double>(1,2) = 232.7530;
    IR.at<double>(2,0) = 0;        IR.at<double>(2,1) = 0;          IR.at<double>(2,2) = 1;

    Coffe_L = cv::Mat(1,8,CV_64F);
    Coffe_R = cv::Mat(1,8,CV_64F);

    //相机1畸变参数
    Coffe_L.at<double>(0,0) = 0.0659;//K1 径向畸变
    Coffe_L.at<double>(0,1) = 0.2319;//K2
    Coffe_L.at<double>(0,2) = -0.0018;//P1 切向畸变
    Coffe_L.at<double>(0,3) = -0.0012;//P2
    Coffe_L.at<double>(0,4) = 0;//K3
    Coffe_L.at<double>(0,5) = 0;//K4
    Coffe_L.at<double>(0,6) = 0;//K5
    Coffe_L.at<double>(0,7) = 0;//K6
    //相机2畸变参数
    Coffe_R.at<double>(0,0) = 0.0535;//K1 径向畸变
    Coffe_R.at<double>(0,1) = 0.1720;//K2
    Coffe_R.at<double>(0,2) = -0.003;//P1 切向畸变
    Coffe_R.at<double>(0,3) = 0.0026;//P2
    Coffe_R.at<double>(0,4) = 0;//K3
    Coffe_R.at<double>(0,5) = 0;//K4
    Coffe_R.at<double>(0,6) = 0;//K5
    Coffe_R.at<double>(0,7) = 0;//K6

    R = cv::Mat(3,3,CV_64F);
    T = cv::Mat(3,1,CV_64F);
    //旋转矩阵 3x3，已转置
    R.at<double>(0,0) = 1.0000; R.at<double>(0,1) = -0.0013; R.at<double>(0,2) = -0.0066;
    R.at<double>(1,0) = 0.0013; R.at<double>(1,1) = 1.0000; R.at<double>(1,2) = 0.0027;
    R.at<double>(2,0) = 0.0066; R.at<double>(2,1) = -0.0027; R.at<double>(2,2) = 1.0000;
    //平移矩阵 3x1
    T.at<double>(0,0) = -91.3083;
    T.at<double>(1,0) = -0.0802;
    T.at<double>(2,0) = -0.2582;

    cv::stereoRectify(IL, Coffe_L, IR, Coffe_R, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
    cv::initUndistortRectifyMap(IL, Coffe_L, R1, P1, img_size, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(IR, Coffe_R, R2, P2, img_size, CV_16SC2, map21, map22);

}

void Stereo_method::Undisortion(cv::Mat &left_img, cv::Mat &right_img)
{
    cv::remap(left_img, imgLTemp, map11, map12, cv::INTER_LINEAR);
    cv::remap(right_img, imgRTemp, map21, map22, cv::INTER_LINEAR);
    imgLTemp(roi1).copyTo(left_img);
    imgRTemp(roi2).copyTo(right_img);
    //    imgLTemp.copyTo(left_img);
    //    imgRTemp.copyTo(right_img);
}

void Stereo_method::Triangulation(cv::Point left_temp, cv::Point right_temp)
{
    CvMat* Point_left = cvCreateMat(2,1,CV_64F);
    CvMat* Point_right = cvCreateMat(2,1,CV_64F);
    cvmSet(Point_left, 0,0,left_temp.x);
    cvmSet(Point_left,1,0,left_temp.y);
    cvmSet(Point_right, 0,0,right_temp.x);
    cvmSet(Point_right,1,0,right_temp.y);
    //CvMat tp1 = P1;
    //CvMat tp2 = P2;
    //CvMat *p1 = &tp1;
    //CvMat *p2 = &tp2;
    CvMat *p1;
    cvCopy(&P1,&p1);//深拷贝才能成功转换。
    CvMat *p2;
    cvCopy(&P2,&p2);

    CvMat* points3d = cvCreateMat(4,1,CV_64F);

    cvTriangulatePoints(p1,p2,Point_left,Point_right,points3d);

    double x = cvmGet(points3d,0,0)/cvmGet(points3d,3,0);
    double y = cvmGet(points3d,1,0)/cvmGet(points3d,3,0);
    double z = cvmGet(points3d,2,0)/cvmGet(points3d,3,0);
    position[0] = x;
    position[1] = y;
    position[2] = z;
}
