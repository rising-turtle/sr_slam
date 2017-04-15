
#include <cv.h>
#if CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION >= 4
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#endif
#include <iostream>
#include <vector>
#include <ros/ros.h>

extern cv::FeatureDetector* createDetector( const std::string& detectorType );

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_sift");
  ros::NodeHandle n;

  //Variables
  cv::Ptr<cv::FeatureDetector> detector_;
  // cv::Ptr<cv::DescriptorExtractor> extractor_;
  
  string feature_name("SIFT");
  // detector_ = createDetector(feature_name);
  // extractor_ = createDescriptorExtractor("SIFT");
  int S = 5;
  double inv_S = 1./(double)(S);
  double sigma0 = 1.6*pow(2., inv_S);
  detector_ = new SiftFeatureDetector(0 /*max_features*/, S /*default lvls/octave*/, 0.04*inv_S*0.5 , 10, 1.6);

  Mat img = imread("image_15.png"); 
  
  // detector->detect( gray_img, feature_locations_2d_, detection_mask);// fill 2d locations
  vector<KeyPoint> feature_locations_2d_;
  detector_->detect( img, feature_locations_2d_);
  
  Mat output;
  ROS_INFO("test_sift.cpp: %u sift features are detected!", feature_locations_2d_.size());

  // drawKeypoints(img, keypoints, output, Scalar::all(-1));
  drawKeypoints(img, feature_locations_2d_, output, Scalar::all(-1));

  namedWindow("sift_test", CV_WINDOW_AUTOSIZE);
  imshow("sift_test", output);
  waitKey(0);

  return 0; 
}

