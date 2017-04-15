/*  
 *  Jan. 13, 2016 David Z 
 *  
 *  main file, to test publish from file 
 *
 * */

#include "ros/ros.h"
#include "pos_msg_pub.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pos_msg");
  ros::NodeHandle n; 

  C2DPosPub pose_pub; 
  pose_pub.publishPosOneLoop(10); 

  return 0; 
}
