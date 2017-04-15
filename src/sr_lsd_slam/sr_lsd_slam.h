/*
 *  June, 21, 2016 David Z
 *
 *  implementation of lsd-slam using SwissRanger 4000
 *
 * */

#ifndef SR_LSD_SLAM_H
#define SR_LSD_SLAM_H

#include "SlamSystem.h"

using namespace lsd_slam; 

class CSRSlamSystem : public SlamSystem
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CSRSlamSystem(int w, int h, Eigen::Matrix3f K, bool enableSLAM = true);
    ~CSRSlamSystem(); 
    
    // using SR4000's data to initialize the first depth map
    void srInit(uchar* image, float* depth, double timeStamp, int id); 

};



#endif
