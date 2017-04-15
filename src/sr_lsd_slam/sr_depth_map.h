/*
 *  June, 22, 2016, David Z 
 *  
 *  try to use depth image to initialize depth map 
 *
 * */


#ifndef SR_DEPTH_MAP_H
#define SR_DEPTH_MAP_H

#include "DepthEstimation/DepthMap.h"

using namespace lsd_slam; 

class CSRDepthMap : public DepthMap
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CSRDepthMap(int w, int h, const Eigen::Matrix3f& K); 
    ~CSRDepthMap(); 

    void initializeFromDepthImg(Frame* new_frame, float * dpt); 

};


#endif
