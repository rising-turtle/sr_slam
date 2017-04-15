#include "sr_depth_map.h"
#include "DepthEstimation/DepthMapPixelHypothesis.h"
#include "DataStructures/Frame.h"
#include "util/globalFuncs.h"
#include "IOWrapper/ImageDisplay.h"
#include "GlobalMapping/KeyFrameGraph.h"

CSRDepthMap::CSRDepthMap(int w, int h, const Eigen::Matrix3f& K):
  DepthMap(w, h, K)
{}

CSRDepthMap::~CSRDepthMap()
{}


void CSRDepthMap::initializeFromDepthImg(Frame* new_frame, float* dpt)
{
  activeKeyFramelock = new_frame->getActiveLock();
  activeKeyFrame = new_frame;
  activeKeyFrameImageData = activeKeyFrame->image(0);
  activeKeyFrameIsReactivated = false;

  const float* maxGradients = new_frame->maxGradients();
  const float* depth_value = dpt; 

  for(int y=1;y<height-1;y++)
  {
    for(int x=1;x<width-1;x++)
    {
      float depth = depth_value[x+y*width]; 
      if(maxGradients[x+y*width] > MIN_ABS_GRAD_CREATE && depth > 0.2 && depth < 10.)
      {
        // float idepth = 0.5f + 1.0f * ((rand() % 100001) / 100000.0f);
        float idepth = 1./depth;     
        currentDepthMap[x+y*width] = DepthMapPixelHypothesis(
            idepth,
            idepth,
            VAR_GT_INIT_INITIAL, // VAR_RANDOM_INIT_INITIAL,
            VAR_GT_INIT_INITIAL, // VAR_RANDOM_INIT_INITIAL,
            20);
      }
      else
      {
        currentDepthMap[x+y*width].isValid = false;
        currentDepthMap[x+y*width].blacklisted = 0;
      }
    }
  }


  activeKeyFrame->setDepth(currentDepthMap);

}
