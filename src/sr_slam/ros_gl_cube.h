/*
 *  David Z April 17, 2016 
 *  
 *  Subcribe euler msg, and display it 
 *
 * */

#ifndef ROS_GL_CUBE_H
#define ROS_GL_CUBE_H

#include "gl_cubic.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <message_filters/subscriber.h>

class CRosGLCube : public CGLCubic
{
  public:
    CRosGLCube(); 
    virtual ~CRosGLCube(); 

    // subscriber for euler angle msg 
    message_filters::Subscriber<std_msgs::Float32MultiArray> * euler_sub_;  // subscriber to receive euler angle msg
    void eulerCb(const std_msgs::Float32MultiArray::ConstPtr& );            // euler msg Callback func

};

#endif
