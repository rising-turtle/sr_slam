/*
 *  June. 28, 2016, David Z 
 *
 *  test function getOptimalNewMatrix and initUndistortRectifyMap
 *  given intrinsic camera matrix 
 *
 * */

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <iostream>

using namespace std; 
using namespace cv;


int main(int argc, char* argv[])
{
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_32F); 
  distCoeffs.at<float>(0,0) = -0.8466; 
  distCoeffs.at<float>(1,0) = 0.537;
  
  Mat originalK_ = Mat(3, 3, CV_64F, cv::Scalar(0)); 
  originalK_.at<double>(0, 0) = 250.5773; 
  originalK_.at<double>(1, 1) = 250.5773; 
  originalK_.at<double>(2, 2) = 1; 
  originalK_.at<double>(0, 2) = 90; 
  originalK_.at<double>(1, 2) = 70 ;

  int in_w = 176; 
  int in_h = 144;
  int ou_w = in_w; 
  int ou_h = in_h;
  Mat K_ = cv::getOptimalNewCameraMatrix(originalK_, distCoeffs, cv::Size(in_w, in_h), 0, cv::Size(ou_w, ou_h), NULL, false); 
  Mat map1, map2;
  initUndistortRectifyMap(originalK_, distCoeffs, cv::Mat(), K_, cv::Size(ou_w, ou_h), CV_16SC2, map1, map2);

  originalK_ = originalK_.t();
  K_= K_.t();
  cout<<"test_cv_K.cpp: originalK_: "<<endl<<originalK_<<endl;
  cout<<"test_cv_K.cpp: K_: "<<endl<<K_<<endl;

  return 1;
}


