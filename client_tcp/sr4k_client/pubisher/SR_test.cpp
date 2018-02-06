#include <ros/ros.h>
#include "SR_reader.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <Eigen/Eigen>

void img2CV32FC1(cv::Mat& depth_img, cv::Mat& float_img)
{
  float_img = cv::Mat(depth_img.size(), CV_32FC1);
  depth_img.convertTo(float_img, CV_32FC1, 0.001, 0);//From mm to m(scale of depth_img matters)
}

void img2CV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img)
{
  mono8_img = cv::Mat(depth_img.size(), CV_8UC1);
  depth_img.convertTo(mono8_img, CV_8UC1, 0.05, -25); //scale to 2cm
}

void test_camera_model(sr_data&);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "SR_test");
  ros::NodeHandle n; 
  CSReader r; 
  r.loadAllData(); 
  CSReader::iterator it = r.begin(); 
  
  bool flag = false;
  while(it != r.end())
  {
    // sr_data d = *it; 
    sr_data d = r.get_current_frame(flag); 
    if(flag) break;
    unsigned char* p =  (unsigned char*)(&d.intensity_[0]);
    // cv::Mat i_img(SR_HEIGHT, SR_WIDTH, CV_16UC1, p, SR_WIDTH*sizeof(SR_IMG_TYPE));
    cv::Mat i_img(SR_HEIGHT, SR_WIDTH, CV_16UC1, p);
    // cv::Mat mono_img; 
    // img2CV8UC1(i_img, mono_img); 
    cv::imshow("SR_image", i_img);
    cv::waitKey(100);
    ++it; 
  }
  // test_camera_model(*it);
  

  return 0;
}


// dr, rotation + distortion 
void cal_Rotation(Eigen::Matrix3f& rot, double roll, double pitch, double yaw)
{
  rot = Eigen::Matrix3f::Zero(); 
  double c1 = cosf(roll); double s1 = sinf(roll); 
  double c2 = cosf(pitch); double s2 = sinf(pitch); 
  double c3 = cosf(yaw); double s3 = sinf(yaw); 
  
  rot(0,0) = c2*c3; //cosf(pitch)cosf(yaw); // rxx 
  rot(0,1) = -c2*s3; // sinf(roll)sinf(pitch)cosf(yaw); // rxy 
  rot(0,2) = s2; // rxz
  rot(1,0) = c1*s3 + c3*s1*s2; //-cosf(pitch)sinf(yaw); // ryx
  rot(1,1) = c1*c3 - s1*s2*s3; // ryy 
  rot(1,2) = -c2*s1; // rxz
  rot(2,0) = s1*s3 - c1*c3*s2; // rzx 
  rot(2,1) = c3*s1 + c1*s2*s3; 
  rot(3,3) = c1*c2; 
}


void test_camera_model(sr_data& d)
{
  double fx = 1./250.21; 
  double fy = 1./250.21; 
  double cx = 87.23; 
  double cy = 69.64; 
  double x,y,z;
  unsigned int i; 
  ofstream comp_xy("comp_xy.log"); 
  for(int v=0; v < SR_HEIGHT; v++)
  {
    for(int u= 0; u<SR_WIDTH; u++)
    {
      i = v*SR_WIDTH + u;
      z = d.z_[i]; 
      x = (cx-u)*fx*z; 
      y = (cy-v)*fy*z;
      // x = (u-cx)*fx*z;
      // y = (v-cy)*fy*z;
      comp_xy<<d.z_[i]<<"\t"<<d.x_[i]<<"\t"<<x<<"\t"<<d.y_[i]<<"\t"<<y<<endl;
    }
  }
}



