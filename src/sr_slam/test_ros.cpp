
#include <ros/ros.h>
// #include "qtros.h"
// #include "parameter_server.h"
// #include "graph_wrapper.h"

#include <QApplication>
#include <QObject>

#include "graph_plane.h"
#include "global_def.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_ros");
  ros::NodeHandle n;
  // CGraphPlane gp;
  // CGraphWrapper gp;
  ROS_INFO("test_ros.cpp: Hello World!");
  return 0;
}
