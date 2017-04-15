#include "sr_lsd_slam.h"
#include "sr_depth_map.h"
#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "IOWrapper/Output3DWrapper.h"

CSRSlamSystem::CSRSlamSystem(int w, int h, Eigen::Matrix3f K, bool enableSLAM): 
  SlamSystem(w, h, K, enableSLAM)
{

}

CSRSlamSystem::~CSRSlamSystem()
{
}


void CSRSlamSystem::srInit(uchar* image, float* depth, double timeStamp, int id)
{
  printf("Doing Depth Image initialization!\n");

  if(!doMapping)
    printf("WARNING: mapping is disabled, but we just initialized... THIS WILL NOT WORK! Set doMapping to true.\n");


  currentKeyFrameMutex.lock();

  currentKeyFrame.reset(new Frame(id, width, height, K, timeStamp, image));
  // map->initializeRandomly(currentKeyFrame.get());
  ((CSRDepthMap*)map)->initializeFromDepthImg(currentKeyFrame.get(), depth);
  keyFrameGraph->addFrame(currentKeyFrame.get());

  currentKeyFrameMutex.unlock();

  if(doSlam)
  {
    keyFrameGraph->idToKeyFrameMutex.lock();
    keyFrameGraph->idToKeyFrame.insert(std::make_pair(currentKeyFrame->id(), currentKeyFrame));
    keyFrameGraph->idToKeyFrameMutex.unlock();
  }
  if(continuousPCOutput && outputWrapper != 0) outputWrapper->publishKeyframe(currentKeyFrame.get());


  if (displayDepthMap || depthMapScreenshotFlag)
    debugDisplayDepthMap();


  printf("Done Depth Image initialization!\n");
}
