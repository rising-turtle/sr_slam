#include "graph_wrapper.h"    // transmit slam data 
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_g2o"); 
  ros::NodeHandle n; 
  
  CGraphWrapper g; 
  g.publishG2O(); 
  ROS_INFO("test_publish_g2o.cpp: finish publish g2o!"); 

  return 0; 
}
