/*
 *  July. 1, 2016, David Z
 *  
 *  read the pose file and publish it out to display 
 *
 * */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <fstream>
#include <iostream>

using namespace std; 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_publish_pose3d");
  ros::NodeHandle n; 
  ros::NodeHandle nh("~"); 

  string f_dir("pose_estimation.log"); 
  int publish_rate = 10;
  nh.param("pose_dir", f_dir, f_dir); 
  nh.param("publish_rate", publish_rate, publish_rate);

  ifstream inf(f_dir.c_str()); 
  if(!inf.is_open())
  {
    ROS_ERROR("test_publish_pose3d.cpp: failed to open file %s", f_dir.c_str());
    return -1; 
  }
  ROS_WARN("test_publish_pose3d.cpp: succeed to open file %s", f_dir.c_str());

  ros::Publisher pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/log_pose_3d",1);
  ros::Time msg_timestamp; 
  ros::Rate r(publish_rate); // broadcast rate 10hz

  char tmp_key; 
  cout<<"test_publish_pose3d.cpp: start to publish pose data, press a key!"<<endl;
  cin>>tmp_key;

  double x, y, z, qx, qy, qz, qw; 
  int id; 
  while(!inf.eof())
  {
    inf>>id>>x>>y>>z>>qx>>qy>>qz>>qw;
    geometry_msgs::PoseStamped pMsg;
    pMsg.pose.position.x = x;
    pMsg.pose.position.y = y; 
    pMsg.pose.position.z = z; 
    pMsg.pose.orientation.x = qx; 
    pMsg.pose.orientation.y = qy;
    pMsg.pose.orientation.z = qz;
    pMsg.pose.orientation.w = qw; 

    if (pMsg.pose.orientation.w < 0)
    {
      pMsg.pose.orientation.x *= -1;
      pMsg.pose.orientation.y *= -1;
      pMsg.pose.orientation.z *= -1;
      pMsg.pose.orientation.w *= -1;
    }

    pMsg.header.stamp = ros::Time::now();
    pMsg.header.frame_id = "world";
    ROS_INFO("test_publish_pose3d.cpp: frame %d publish pos_3d : %f %f %f %f %f %f %f", id, pMsg.pose.position.x, pMsg.pose.position.y, pMsg.pose.position.z,pMsg.pose.orientation.x, pMsg.pose.orientation.y, pMsg.pose.orientation.z, pMsg.pose.orientation.w);
    pose_publisher.publish(pMsg);

    ros::spinOnce();
    r.sleep();
  }
  return 0; 

}
