#include "ros_gl_cube.h"
#include "global_def.h"
#include <GL/glut.h>

CRosGLCube::CRosGLCube()
{
  ros::NodeHandle n; 
  euler_sub_ = new message_filters::Subscriber<std_msgs::Float32MultiArray>(n, "/euler_msg", 10); 
  euler_sub_->registerCallback(boost::bind(&CRosGLCube::eulerCb, this, _1)); 
}

CRosGLCube::~CRosGLCube(){}

void CRosGLCube::eulerCb(const std_msgs::Float32MultiArray::ConstPtr& euler_ptr)
{
  static int cnt = 0; 
  roll_ = R2D(euler_ptr->data[0]); 
  pitch_ = R2D(euler_ptr->data[1]); 
  yaw_ = R2D(euler_ptr->data[2]); 
  ROS_INFO("ros_gl_cube.cpp: recv %d euler r: %f, p: %f, y: %f",cnt++, roll_, pitch_, yaw_);
  glutPostRedisplay();  // repaint the screen
  // CGLCubic::drawCallback();
  // b_needed_to_repaint_ = true;
}
