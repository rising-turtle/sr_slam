/*
 *  Oct. 14, 2015, David Z
 *  
 *  Feature Detector, to alter the parameters of SIFT feature detector 
 *
 * */

#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include "feature_adjuster.h"

class CDetectorWrapper : public DetectorAdjuster
{
public:
   // Initial values are for SURF detector
  CDetectorWrapper(const char* detector_name, double initial_thresh=200.f, double min_thresh=2, double max_thresh=10000, double increase_factor=1.3, double decrease_factor=0.7);

  // this is for SIFT detector , not including edgeThreshold, and sigma right now 
  CDetectorWrapper(const char* detector_name, int nReturned_Features, int nOctaves, double contrastThreshold); 

  virtual ~CDetectorWrapper(); 

protected:
  virtual void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const;
  
  // SIFT parameters 
  int nReturnedFeatures_;     // returned number of features 
  int nOctaveLayers_;         // 
  double contrastThreshold_;  //

};

extern cv::FeatureDetector* myCreateDetector( const std::string& detectorName);

#endif
