/*
 * David Z, Apr 10, 2015
 * 
 * receive sr_information, through ROS tools
 *
 * */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"

#include <pcl/io/pcd_io.h>

#include "libMesaSR.h"
#include "cam_model.h"

#include <iostream>
#include <fstream>
#include <pthread.h>
using namespace std;

typedef pcl::PointXYZ point_type; 
typedef pcl::PointCloud<point_type> cloud_type; 
typedef typename cloud_type::Ptr  cloudPtr; 

#define S2F 0.001

bool g_cloud_is_ready = false;
static SRCAM g_cam;
cloudPtr g_point_cloud(new cloud_type(176, 144));
pthread_mutex_t g_mut_;
vector<unsigned char> g_buf_;

bool g_exit = false; 
bool g_syn_flag = false;
bool g_get_msg = false;
int g_count = 0;

// call back function
void exitCallback(const std_msgs::BoolPtr& e);
void synCallback(const std_msgs::BoolPtr& b);
void chatterCallback(const std_msgs::String::ConstPtr& msg);
void sr_imgCB(const sensor_msgs::ImageConstPtr& img_msg);
void sr_arrayCB(const std_msgs::UInt8MultiArray::ConstPtr& sr_array);
void sr_arrayCB_pcl(const std_msgs::UInt8MultiArray::ConstPtr& sr_array);

// test function 
void file_test(ros::NodeHandle&);
void cam_test(ros::NodeHandle&);
void first_syn(ros::NodeHandle& n, ros::Publisher& ack_pub_);

// get a new point cloud
bool getUpdateCloud(cloudPtr& pc);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sr_listener");
  ros::NodeHandle n;
  pthread_mutex_init(&g_mut_, NULL);
  
  cam_test(n);

  return 0;
}

// get a new intensity image and depth image
bool getUpdateImage(cv::Mat& i_img, cv::Mat& d_img)
{
  // get raw data from sr_publisher 
  if(!g_cloud_is_ready) 
  {
    // ROS_INFO("sr_subscriber.cpp: cloud is not ready, wait!");
    return false; 
  }
  static bool once = false;
  static vector<unsigned char> local_buf;
  if(!once)
  {
    local_buf.resize(g_buf_.size());
    once = true; 
  } 
  {
    pthread_mutex_lock(&g_mut_); 
      // pc = g_point_cloud->makeShared(); 
      memcpy(local_buf.data(), g_buf_.data(), local_buf.size());
      g_cloud_is_ready = false;
    pthread_mutex_unlock(&g_mut_);
  }
  
  // get distance image first 
  unsigned char* pDis = local_buf.data(); 

  // compute the point cloud
  int rows = 144; 
  int cols = 176;
  unsigned int total = rows*cols; 
  unsigned int sr_n  = total;
  unsigned short* pD ; 
  unsigned char* pI ; 

  cv::Mat intensity_img;   
  cv::Mat distance_img; 
  const static unsigned int distance_size = sr_n*(sizeof(unsigned short) + sizeof(unsigned char));
  const static unsigned int coordinate_size = sr_n*(sizeof(float)*3 + sizeof(unsigned char));
  if(local_buf.size() == distance_size)
  {
    pD = (unsigned short*)(pDis); 
    pI = (pDis + total*sizeof(unsigned short));
    intensity_img = cv::Mat(rows, cols, CV_8UC1, pI); 
    distance_img = cv::Mat(rows, cols, CV_16UC1, pD); 
  }else
  {
    pI = pDis; 
    intensity_img = cv::Mat(rows, cols, CV_8UC1, pI);
  }

  // copy out 
  i_img = intensity_img.clone(); 
  // d_img = distance_img.clone(); 
  // transpose(intensity_img, i_img); 
  // transpose(distance_img, d_img);
  // cout<<"i_img size: "<<i_img.size()<<endl; 
  // cout<<"d_img size: "<<d_img.size()<<endl;
  return true;
}

// get a new point cloud
bool getUpdateCloud(cloudPtr& pc)
{
  if(!g_cloud_is_ready) 
  {
    // ROS_INFO("sr_subscriber.cpp: cloud is not ready, wait!");
    return false; 
  }
  static bool once = false;
  static vector<unsigned char> local_buf;
  
  // camera model without distortion parameters
  static CamModel cam_model(223.9758, 226.7442, 89.361, 75.8112);
  if(!once)
  {
    local_buf.resize(g_buf_.size());
    once = true; 
  }
  {
    pthread_mutex_lock(&g_mut_); 
      // pc = g_point_cloud->makeShared(); 
      memcpy(local_buf.data(), g_buf_.data(), local_buf.size());
      g_cloud_is_ready = false;
    pthread_mutex_unlock(&g_mut_);
  }
  {
    // construct a point cloud 
    ROS_INFO("sr_subscriber.cpp: get SRCAM handle!");
    unsigned char* pDis = local_buf.data(); 

    // compute the point cloud
    int rows = 144; 
    int cols = 176;
    unsigned int total = rows*cols; 
    unsigned int sr_n = total;

    const static unsigned int distance_size = sr_n*(sizeof(unsigned short) + sizeof(unsigned char));
    const static unsigned int coordinate_size = sr_n*(sizeof(float)*3 + sizeof(unsigned char));
    if(local_buf.size() == distance_size)
    {
      unsigned short* pD = (unsigned short*)(pDis); 
      unsigned char* pI = (pDis + total*sizeof(unsigned short));

      double ox, oy, oz;
      if(pc->points.size() != total)
      {
        pc->points.resize(total); 
      }
      for(int i=0; i<rows; i++)
      {
        for(int j=0; j<cols; j++)
        {
          int k = i*cols + j; 
          float z = (*pD)*0.001; 
          cam_model.convertUVZ2XYZ(j+0.5, i+0.5, z, ox, oy, oz); 
          point_type& pt = pc->points[k]; 
          pt.x = -ox; pt.y = -oy; pt.z = oz - 0.01;
          ++pD;
          ++pI;
        }
      }
    }else{
      unsigned char* pI = pDis;
      float * px = (float*)(pDis + sr_n*sizeof(unsigned char)); 
      float * py = (float*)(pDis + sr_n*(sizeof(unsigned char) + sizeof(float))); 
      float * pz = (float*)(pDis + sr_n*(sizeof(unsigned char) + sizeof(float)*2));
      for(int i=0; i<total; i++)
      {
        point_type& pt = pc->points[i]; 
        pt.x = *px; pt.y = *py; pt.z = *pz; 
        ++px; ++py; ++pz;
      }
    }
    // pcl::io::savePCDFile("chachacha.pcd", *pc);

    /*   this does not work, because the handle SRCAM is just a pointer
    int sr_cam_offset = sizeof(SRCAM);
    // obtain the camera handle
    memcpy(&g_cam, pDis, sr_cam_offset);
    int rows = SR_GetRows(g_cam); 
    int cols = SR_GetCols(g_cam); 
    int total = rows*cols;
    unsigned short * pD = (unsigned short*)SR_GetImage(g_cam, 0); 
    // vector<unsigned short> tmp(total, 0); 
    // unsigned short * pD = tmp.data(); 

    ROS_INFO("sr_subscriber.cpp: get distance info!");
    // copy input buf into distance buf 
    memcpy(pD, (pDis+sr_cam_offset), total*sizeof(unsigned short)); 

    ROS_INFO("sr_subscriber.cpp: finish get distance info!");
    float *ptrXYZ = (float*)(&pc->front());
    int s = sizeof(point_type); 

    static int count = 0; 
    ROS_INFO("sr_subscriber.cpp: recv %d frame!", count++);

    // critical segment 
    SR_CoordTrfFlt(g_cam, &ptrXYZ[0], &ptrXYZ[1], &ptrXYZ[2], s,s,s);
    pcl::io::savePCDFile("chachacha.pcd", *pc);
    */

  }
  return true;
}

void depth8UC1(cv::Mat& depth_img, cv::Mat& mono8_img)
{
  mono8_img = cv::Mat(depth_img.size(), CV_8UC1);
  cv::Mat float_img;
  depth_img.convertTo(mono8_img, CV_8UC1, 0.05, -25); //scale to 2cm
  // depth_img.convertTo(float_img, CV_32FC1, 0.001, 0);//From mm to m(scale of depth_img matters)
  // depth_img = float_img;
}

void cam_test(ros::NodeHandle& n)
{
  ros::Subscriber syn_sub_ = n.subscribe("/syn", 1, synCallback);
  ros::Publisher  ack_pub_ = n.advertise<std_msgs::Bool>("/ack", 1);
  // subscribe sw_array 
  ros::Subscriber sr_array_sub = n.subscribe<std_msgs::UInt8MultiArray>("/sr_array", 1, sr_arrayCB_pcl);
  
  // 1, syn 
  first_syn(n, ack_pub_);

  // PCL point cloud 
  pcl::PointCloud<pcl::PointXYZ>::Ptr 
    cloud(new pcl::PointCloud<point_type>(144, 176));
  
  /*
  // visualization 
  pcl::visualization::CloudViewer viewer("MesaSR PCL Viewer"); 

  while(!viewer.wasStopped() && n.ok())
  {
    if(getUpdateCloud(cloud))
    {
      ROS_WARN("sr_subscriber.cpp: great I get a new cloud, show it!");
      viewer.showCloud(cloud); 
    }else
    {
      // ROS_INFO("sr_subscriber.cpp: failed to getUpdateCloud()!");
    }
    ros::spinOnce();
    usleep(10000); // sleep 10 ms
  }*/
  
  cv::Mat I_img; 
  cv::Mat D_img;
  cv::Mat D8_img;
  // while(n.ok())
  // ofstream disf("distance.log");
  // ofstream imgf("intensity.log");
  // ofstream dis8f("distance8.log");
  while(n.ok())
  {
    if(getUpdateImage(I_img, D_img))
    {
      ROS_WARN("sr_subscriber.cpp: great I get a new image, show it!");
      cv::namedWindow( "intensity window", cv::WINDOW_AUTOSIZE );// Create a window for display.
      cv::imshow( "intensity window", I_img );                   // Show our image inside it.
      // depth8UC1(D_img, D8_img);
      // cv::namedWindow( "distance window", cv::WINDOW_AUTOSIZE );// Create a window for display.
      // cv::imshow( "distance window", D8_img );                   // Show our image inside it.
      cv::waitKey(30); 
    }
    // disf<<D_img;
    // imgf<<I_img;
    // dis8f<<D8_img;
    ros::spinOnce(); 
    usleep(10000); // sleep 10 ms
  }
}

void first_syn(ros::NodeHandle& n, ros::Publisher& ack_pub_)
{
  ros::Rate r(50); 
  ROS_INFO("listener.cpp: wait for talker to send syn!");
  while(n.ok() && !g_syn_flag)
  {
    ros::spinOnce();
    r.sleep();
  }
  std_msgs::BoolPtr ack_f(new std_msgs::Bool); 
  ack_f->data = true; 
  ack_pub_.publish(ack_f);
  ROS_INFO("listener.cpp: after syn ack!");
}

void sr_arrayCB_pcl(const std_msgs::UInt8MultiArray::ConstPtr& sr_array)
{
  static bool once = false; 
  if(!once)
  {
    g_buf_.resize(sr_array->data.size()); 
    once = true; 
  }
  {
    pthread_mutex_lock(&g_mut_); 
      memcpy(g_buf_.data(), sr_array->data.data(), g_buf_.size());
      g_cloud_is_ready = true;
    pthread_mutex_unlock(&g_mut_);
  }
}


/*
void sr_arrayCB_pcl(const std_msgs::UInt8MultiArray::ConstPtr& sr_array)
{
  static int rows = 144; //SR_GetRows(g_cam); 
  static int cols = 176; //SR_GetCols(g_cam);
  static int total = rows*cols;
  const unsigned char * pDis = sr_array->data.data();
  
  int sr_cam_offset = sizeof(SRCAM);
  
  ROS_INFO("sr_subscriber.cpp: obtain an array with size: %d", sr_array->data.size());

  static bool once = false;
  SRCAM tmp_cam;
  memcpy(&tmp_cam, pDis, sr_cam_offset);

  if(!once)
  {
    ROS_INFO("sr_subscriber.cpp: get SRCAM handle!");
    // obtain the camera handle
    memcpy(&g_cam, pDis, sr_cam_offset);
    once = true;
  }
  unsigned short * pD = (unsigned short*)SR_GetImage(tmp_cam, 0); 
  // vector<unsigned short> tmp(total, 0); 
  // unsigned short * pD = tmp.data(); 

  ROS_INFO("sr_subscriber.cpp: get distance info!");
  // copy input buf into distance buf 
  memcpy(pD, (pDis+sr_cam_offset), total*sizeof(unsigned short)); 
  ROS_INFO("sr_subscriber.cpp: finish get distance info!");
  float *ptrXYZ = (float*)(&g_point_cloud->front());
  int s = sizeof(point_type); 
  
  static int count = 0; 
  ROS_INFO("sr_subscriber.cpp: recv %d frame!", count++);

  // critical segment 
  {
    pthread_mutex_lock(&g_mut_); 
      SR_CoordTrfFlt(tmp_cam, &ptrXYZ[0], &ptrXYZ[1], &ptrXYZ[2], s,s,s);
      g_cloud_is_ready = true;
      // pcl::io::savePCDFile("chachacha.pcd", *g_point_cloud);
    pthread_mutex_unlock(&g_mut_);
  }
}*/


void file_test(ros::NodeHandle& n)
{
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::Subscriber syn_sub_ = n.subscribe("/syn", 1, synCallback);
  ros::Subscriber exit_sub_ = n.subscribe("/exit", 1, exitCallback);
  ros::Publisher  num_pub_ = n.advertise<std_msgs::Int32>("/num_rece", 1); 
  ros::Publisher  ack_pub_ = n.advertise<std_msgs::Bool>("/ack", 1);
  
  // subscribe sw_image 
  ros::Subscriber img_sub_ = n.subscribe<sensor_msgs::Image>("/camera/sr_image", 1, sr_imgCB);

  // subscribe sw_array 
  ros::Subscriber sr_array_sub = n.subscribe<std_msgs::UInt8MultiArray>("/sr_array", 100, sr_arrayCB);

  ros::Rate r(50); 
  ROS_INFO("listener.cpp: wait for talker to send syn!");
  while(n.ok() && !g_syn_flag)
  {
    ros::spinOnce();
    r.sleep();
  }
  std_msgs::BoolPtr ack_f(new std_msgs::Bool); 
  ack_f->data = true; 
  ack_pub_.publish(ack_f);
  ROS_INFO("listener.cpp: after syn ack!");
  while(n.ok() && !g_exit)
  {
    if(g_get_msg)
    {
      // get msg 
      std_msgs::Int32Ptr num(new std_msgs::Int32); 
      num->data = g_count;
      num_pub_.publish(num);
      g_get_msg = false;
    }
    ros::spinOnce();
    r.sleep();
  }
}


void sr_arrayCB(const std_msgs::UInt8MultiArray::ConstPtr& sr_array)
{
  ROS_INFO("I receive array %d", ++g_count); 
  // display what I have received, 
  /*std::vector<unsigned char>::const_iterator it = sr_array->data.begin(); 
  cout<<"received: ";
  while(it != sr_array->data.end())
  {
    cout<<" "<<(int)(*it);
    ++it; 
  }
  cout<<endl;*/
  
  int N = 5; 
  vector<unsigned short> tS(N, 0); 
  vector<float> tF(N, 0); 
  const unsigned char* pS = sr_array->data.data(); 
  memcpy(&tS[0], pS, N*sizeof(unsigned short)); 
  memcpy(&tF[0], pS+N*sizeof(unsigned short), N*sizeof(float));
  
  cout<<"rece: ";
  for(int i=0; i<N; i++)
  {
    cout<<" "<<tS[i];
  }
  for(int i=0; i<N; i++)
  {
    cout<<" "<<tF[i];
  }
  cout<<endl;
  g_get_msg = true;
}

void exitCallback(const std_msgs::BoolPtr& e)
{
  g_exit = true;
  ROS_INFO("listener.cpp: let' exit! ");
}

void synCallback(const std_msgs::BoolPtr& b)
{
  g_syn_flag = b->data;
  ROS_INFO("listener.cpp: listen to the syn!");
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  g_get_msg = true;
  ++g_count; 
  // std_msgs::IntPtr num(new std_msgs::Int);
}

void sr_imgCB(const sensor_msgs::ImageConstPtr& img_msg)
{
  ROS_INFO("I receive img %d", ++g_count); 
  // display it? ok, let' do it 
  cv_bridge::CvImagePtr cv_ptr; 
  cv_ptr = cv_bridge::toCvCopy(img_msg); 
  cv::Mat cv_img = cv_ptr->image ; 
  cv::imshow("receive and show, ", cv_img);
  cv::waitKey(20); 
  // ROS_INFO("listener.cpp: now g_count = %d", g_count);
  g_get_msg = true;
}
