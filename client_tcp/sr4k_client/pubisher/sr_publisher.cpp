/*
 * David Z, Apr 10, 2015 
 *
 * broadcast sr_information, through ROS tools
 *
 * */

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "SR_reader.h"
#include <sstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include "SR_interface.h"

using namespace std;

bool g_ack_syn = false; 
void ackCallback(const std_msgs::BoolPtr& ack)
{
  g_ack_syn = ack->data;
  cout<<"talker.cpp: get ack!"<<endl;
}

int g_rece_num = -1;
void numCallback(const std_msgs::Int32Ptr& num)
{
  g_rece_num = num->data;
  ROS_WARN("sr_publisher.cpp: get num callback %d", g_rece_num);
}

cv::Mat from_SR_to_mat(sr_data& d)
{
    unsigned char* p =  (unsigned char*)(&d.intensity_[0]);
    cv::Mat i_img(sr_data::_sr_height, sr_data::_sr_width, CV_16UC1, p, 
        sr_data::_sr_width*sizeof(sr_data::_sr_type));
    return i_img;
}

void file_test(ros::NodeHandle& n);
void cam_test(ros::NodeHandle& n);
void first_syn(ros::Publisher& );

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sr_publisher");
  ros::NodeHandle n;
  // file_test(n);
  cam_test(n);
  return 0;
}

void cam_test(ros::NodeHandle& n)
{
  ros::Publisher syn_pub_ = n.advertise<std_msgs::Bool>("/syn", 1);
  ros::Subscriber ack_sub_ = n.subscribe("/ack", 1, ackCallback); 

  // ros::Publisher exit_pub_ = n.advertise<std_msgs::Bool>("/exit", 1);
  // ros::Subscriber num_sub_ = n.subscribe("/num_rece", 1, numCallback);
  
  // publish sw_array 
  ros::Publisher sr_array_pub = n.advertise<std_msgs::UInt8MultiArray>("/sr_array", 1); 
  
  ros::NodeHandle nh_p("~");
  // nh_p.setParam("sr_source", "SR_FILE"); 
  // nh_p.setParam("sr_source", "SR_CAM");
  // nh_p.setParam("sr_end_frame", 10); 
  // nh_p.setParam("sr_new_file_version", false); // test old file version  
  string sr_source;
  int sr_publish_rate; 
  nh_p.param("sr_source", sr_source, std::string("SR_FILE"));
  nh_p.param("sr_publish_rate", sr_publish_rate, 2);

  // 1, generate sr data; 
  CSRInterface sr_instance; 
  std_msgs::UInt8MultiArray sr_array; 

  int N = 500; // total send 100 frames 
  int count = 0; 
  if(!sr_instance.open())
  {
    ROS_ERROR("sr_publisher.cpp: open sr_instance fail!");
    return ;
  }
  
  // 2, syn at first step
  first_syn(syn_pub_);
  ros::Rate r(sr_publish_rate); // broadcast rate 10hz

  while(ros::ok())
  {
    sr_array.data.clear();
    if(!sr_instance.get(sr_array))
    {
      ROS_WARN("sr_publisher.cpp: failed to get next frame, exit!");
      break; 
    }
    
    ROS_INFO("sr_publisher.cpp: publish data size: %d", sr_array.data.size());
    sr_array_pub.publish(sr_array);
    
    ros::spinOnce();
    if(++count > N) break; 
    ROS_INFO("sr_publisher.cpp: publish %d frame", count);
    if(sr_source == "SR_FILE")
      r.sleep(); 
  }
  ROS_INFO("sr_publisher.cpp: exit after task!");
  // exit_pub_.publish(b_syn_ok);
}

// syn preparation for the next process
void first_syn(ros::Publisher& syn_pub_)
{
  ros::Rate loop_rate(50);
  std_msgs::BoolPtr b_syn_ok(new std_msgs::Bool);
  b_syn_ok->data = true;
 
  ROS_INFO("talker.cpp: wait for syn ack!");
    // syn first 
  while(ros::ok() && !g_ack_syn)
  {
    syn_pub_.publish(b_syn_ok);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("talker.cpp: after syn ack!");
}


void file_test(ros::NodeHandle& n)
{
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher syn_pub_ = n.advertise<std_msgs::Bool>("/syn", 1);
  ros::Publisher exit_pub_ = n.advertise<std_msgs::Bool>("/exit", 1);
  ros::Subscriber ack_sub_ = n.subscribe("/ack", 1, ackCallback); 
  ros::Subscriber num_sub_ = n.subscribe("/num_rece", 1, numCallback);
  
  // publish sw_image 
  ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("/camera/sr_image", 1);

  // publish sw_array 
  ros::Publisher sr_array_pub = n.advertise<std_msgs::UInt8MultiArray>("/sr_array", 1); 

  ros::Rate loop_rate(50);
  std_msgs::BoolPtr b_syn_ok(new std_msgs::Bool);
  b_syn_ok->data = true;
  
  // load SR data first 
  // CSReader r; 
  // if(!r.loadAllData())
  // {
  //  ROS_ERROR("talker.cpp: no SR data is loaded!");
  //  return -1;
  // }
  
    ROS_INFO("talker.cpp: wait for syn ack!");
    // syn first 
    while(ros::ok() && !g_ack_syn)
    {
      syn_pub_.publish(b_syn_ok);
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("talker.cpp: after syn ack!");
       
    // while(ros::ok() && g_rece_num < 20)
    // send the SR_image 
    int count = 0; // used to get ack that the image has been received
    ros::Time msg_timestamp; // ros time, used in the msg.header
    // CSReader::iterator it = r.begin();
    cv_bridge::CvImage out_msg; 
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1; // image type

    // construct sr_array structure 
    std_msgs::UInt8MultiArray sr_array; 
    int N = 5; 
    int T = 5;
    srand(NULL);

    while(ros::ok()) //&& it != r.end())
    {
      
      /*  // then send the string
      std_msgs::String msg;
      std::stringstream ss;
      ss << "send number " << count;
      msg.data = ss.str();
      ROS_INFO("%s", msg.data.c_str());
      chatter_pub.publish(msg);
      */
      
      /*
      sr_data d = *it; 
      cv::Mat cv_img = from_SR_to_mat(d); 
      msg_timestamp = ros::Time();
      out_msg.header.stamp = msg_timestamp;
      out_msg.header.seq = count+1;
      out_msg.image = cv_img;
      img_pub.publish(out_msg);
      */
      
      // randomly construct data 
      int total_size = N*sizeof(unsigned short) + N*sizeof(float);
      vector<unsigned short> tS(N, 0); 
      vector<float> tF(N, 0);
      sr_array.data.resize(total_size);
      cout<<"send: ";
      for(int i=0; i<N; i++)
      {
        unsigned short tmp = rand()%65536;
        tS[i] = tmp;
        // sr_array.data[i] = tmp; 
        cout<<" "<<tmp;
      }
      unsigned char* pS = sr_array.data.data(); 
      memcpy((void*)pS, (const void*)&tS[0], N*sizeof(unsigned short));
      
      for(int i=0; i<N; i++)
      {
        float tmp = (rand()%65536)*0.01;
        tF[i] = tmp; 
        cout<<" "<<tmp;
      }
      cout<<endl;
      memcpy((void*)(pS + N*sizeof(unsigned short)), &tF[0], N*sizeof(float));
      sr_array_pub.publish(sr_array);

      ++count;
      while(g_rece_num != count && ros::ok())
      {
        ros::spinOnce();
        loop_rate.sleep();
      }
      ROS_INFO("talker.cpp: number of communicate data: %d , while count = %d", g_rece_num, count);
      // ++count;
      // ++it;

      // for termination 
      if(count > T) break;
    }
  exit_pub_.publish(b_syn_ok);
}



