#include "graph_plane.h"

void CGraphPlane::togglePause()
{
  pause_ = !pause_; 
  ROS_INFO("Pause toggled to: %s", pause_? "true":"false");
  if(pause_) Q_EMIT setGUIStatus("Processing Thread Stopped");
  else Q_EMIT setGUIStatus("Processing Thread Running");
}

void CGraphPlane::getOneFrame()
{
  getOneFrame_ = true;
}
