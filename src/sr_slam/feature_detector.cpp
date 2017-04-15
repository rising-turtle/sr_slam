
#include "feature_detector.h"
#include <ros/ros.h>
#include "parameter_server.h"
#include <string.h>
#include "opencv2/nonfree/features2d.hpp"

using namespace cv;

extern cv::FeatureDetector* createDetector( const std::string& detectorType );
extern StatefulFeatureDetector* adjustedGridWrapper(cv::Ptr<DetectorAdjuster> detadj);
extern StatefulFeatureDetector* adjusterWrapper(cv::Ptr<DetectorAdjuster> detadj, int min, int max);

FeatureDetector* myCreateDetector( const std::string& detectorName)
{
  DetectorAdjuster* detAdj = NULL;

  ROS_WARN_STREAM(" feature_detector.cpp: Using " << detectorName << " keypoint detector.");
  if( detectorName == "SIFTGPU" ) {
    return NULL;// Will not be used
  } 
  else if(detectorName == "FAST") {
    // detAdj = new DetectorAdjuster("FAST", 20);
    detAdj = new CDetectorWrapper("FAST", 20);
  }
  else if(detectorName == "SURF" || detectorName == "SURF128") {
    // detAdj = new DetectorAdjuster("SURF", 200);
    detAdj = new CDetectorWrapper("SURF", 200);
  }
  else if(detectorName == "SIFT") 
  {
    ros::NodeHandle nh("~"); 
    int returnedFeatures, nOctaveLayers; 
    double threshold, inv_OctaveLayers; 
    nh.param<int>("sift_num_features", returnedFeatures, 0); 
    nh.param<int>("sift_octave_layers", nOctaveLayers, 3); 
    nh.param<double>("sift_contrast_threshold", threshold, 0.04);
    inv_OctaveLayers = 1./(double)(nOctaveLayers*2);

    // detAdj = new DetectorAdjuster("SIFT", 0.04, 0.0001);
    detAdj = new CDetectorWrapper("SIFT", returnedFeatures, nOctaveLayers, threshold * inv_OctaveLayers); // according to the sift  matlab version
  }
  else if(detectorName == "ORB") {
    // detAdj = new DetectorAdjuster("AORB", 20);
    detAdj = new CDetectorWrapper("AORB", 20);
  } 
  else {
    ROS_ERROR("Unsupported Keypoint Detector. Using GridDynamicORB as fallback.");
    return createDetector("GridDynamicORB");
    // return create("GridDynamicORB");
  }
  assert(detAdj != NULL);

  ParameterServer* params = ParameterServer::instance();
  bool gridWrap = (params->get<int>("detector_grid_resolution") > 1);
  bool dynaWrap = (params->get<int>("adjuster_max_iterations") > 0);

  if(dynaWrap && gridWrap){
    return adjustedGridWrapper(detAdj);
  }
  else if(dynaWrap){
    int min = params->get<int>("max_keypoints");
    int max = min * 1.5; //params->get<int>("max_keypoints");
    return adjusterWrapper(detAdj, min, max);
  }
  else return detAdj;
}


CDetectorWrapper::CDetectorWrapper(const char* detector_name, double initial_thresh, double min_thresh, double max_thresh, double increase_factor, double decrease_factor):
  DetectorAdjuster(detector_name, initial_thresh, min_thresh, max_thresh, increase_factor, decrease_factor),
  nReturnedFeatures_(0), 
  nOctaveLayers_(3),
  contrastThreshold_(0.04)
{
}

CDetectorWrapper::CDetectorWrapper(const char* detector_name, int nReturnedFeatures, int nOctaveLayers, double contrastThreshold):
  DetectorAdjuster(detector_name), 
  nReturnedFeatures_(nReturnedFeatures),
  nOctaveLayers_(nOctaveLayers),
  contrastThreshold_(contrastThreshold)
{
}

CDetectorWrapper::~CDetectorWrapper(){}

void CDetectorWrapper::detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask ) const
{
  Ptr<FeatureDetector> detector; 
  if(strcmp(detector_name_, "SIFT") == 0)
  {
    detector = new SiftFeatureDetector( nReturnedFeatures_/*max_features*/, nOctaveLayers_ /*default lvls/octave*/, contrastThreshold_);
    detector->detect(image, keypoints, mask);
    return ;
  }

  // as rgbdslam do 
  DetectorAdjuster::detectImpl(image, keypoints, mask);
}


