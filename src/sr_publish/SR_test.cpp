#include <ros/ros.h>
#include "SR_reader.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <Eigen/Eigen>

#ifndef SQ(x) 
#define SQ(x) ((x)*(x))
#endif


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
void test_camera_model_distortion(sr_data&);
void distortCorrection(double x_d, double y_d, double& x_u, double &y_u, vector<double>& k);

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

    // test_camera_model(*it);
    test_camera_model_distortion(*it);

    unsigned char* p =  (unsigned char*)(&d.intensity_[0]);
    // cv::Mat i_img(SR_HEIGHT, SR_WIDTH, CV_16UC1, p, SR_WIDTH*sizeof(SR_IMG_TYPE));
    cv::Mat i_img(SR_HEIGHT, SR_WIDTH, CV_16UC1, p);
    // cv::Mat mono_img; 
    // img2CV8UC1(i_img, mono_img); 
    cv::imshow("SR_image", i_img);
    cv::waitKey(60000); // wait for one minute 
    ++it; 
  }
  

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
  // test AMIR's camera model  
  double fx = 1./218.779; // 1./250.21; 
  double fy = 1./229.599; // 1./250.21; 
  double cx = 90.1334; // 87.23; 
  double cy = 69.1473; // 69.64; 
  double x,y,z, gx, gy, err;
  double sum_error = 0;
  int cnt = 0; 
  unsigned int i; 
  ofstream comp_xy("comp_xy.log"); 
  for(int v=0; v < SR_HEIGHT; v++)
  {
    for(int u= 0; u<SR_WIDTH; u++)
    {
      i = v*SR_WIDTH + u;
      gx = -d.x_[i]; 
      gy = -d.y_[i];

      z = d.z_[i]; 
      x = (cx-u)*fx*z; 
      y = (cy-v)*fy*z;
      // x = (u-cx)*fx*z;
      // y = (v-cy)*fy*z;
      err = SQ(x - gx) + SQ(y - gy);
      sum_error += err; 
      ++cnt;
      comp_xy<<d.z_[i]<<"\t"<<d.x_[i]<<"\t"<<x<<"\t"<<d.y_[i]<<"\t"<<y<<endl;
    }
  }
  printf("SR_test.cpp: this frame has %d points, sqr_err: %lf, MSE %lf\n", cnt, sum_error, sqrt(sum_error/(double)cnt));

}

void test_camera_model_distortion(sr_data& d)
{
  // test AMIR's camera model  
  // double f = 250.5773, Cx = 90, Cy = 70, k1 = -0.8466, k2 = 0.5370
  double fx = 1./250.5773; 
  double fy = 1./250.5773;  
  double cx =  90; 
  double cy =  70; 
  vector<double> k;
  k.push_back(-0.8466); // k1 
  k.push_back(0.5370);  // k2 

  double x_d, y_d;  // distorted point on the normalized image plane 
  double x_u, y_u;  // undistorted point on the normalized image plane
  double x, y, z;   // coordinate of the point in the camera's reference frame
  double gx, gy; // readout x, y, 

  double sum_error = 0; 
  double err = 0; 

  int cnt = 0; 
  int i;  // index of the point 
  ofstream comp_xy("comp_xy.log"); 
  for(int v=0; v < SR_HEIGHT; v++)
  {
    for(int u= 0; u<SR_WIDTH; u++)
    {
      i = v*SR_WIDTH + u;
      gx = -d.x_[i]; 
      gy = -d.y_[i];
      z = d.z_[i];   // depth 
      if(z <= 0 || z > 5)
        continue;
      z += 0.01; 
      cnt++;
      x_d = (u-cx)*fx; 
      y_d = (v-cy)*fy;
      
      distortCorrection(x_d, y_d, x_u, y_u, k); 

      x = x_u * z; 
      y = y_u * z; 
      err = SQ(x - gx) + SQ(y - gy);
      sum_error += err; 
      comp_xy<<d.z_[i]<<"\t"<<-d.x_[i]<<"\t"<<x<<"\t"<<-d.y_[i]<<"\t"<<y<<endl;
    }
  }
  printf("SR_test.cpp: this frame has %d points, sqr_err: %lf, MSE %lf\n", cnt, sum_error, sqrt(sum_error/(double)cnt));
}


void distortCorrection(double x_d, double y_d, double& x_u, double &y_u, vector<double>& k)
{
    if(k.size() == 0) // no distortion correction needed 
    {
      x_u = x_d; 
      y_u = y_d;
    }else if(k.size() == 1)// only k1 is given 
    {
      // TODO :  not  quite understand what's the magic 
      double k1 = k[0]; 
      double rd2 = SQ(x_d) + SQ(y_d); 
      double r_distoration = 1 + k1*rd2; 
      double ru2 = rd2 / r_distoration; 
      r_distoration = 1 + k1*ru2; 
      x_u = x_d / r_distoration; 
      y_u = y_d / r_distoration; 
      
    }else
    {
        // using iterative fixed_point root finding algorithm 
        double k1 = k[0]; 
        double k2 = k.size() >= 2? k[1]:0; 
        double p1 = k.size() >= 3? k[2]:0; 
        double p2 = k.size() >= 4? k[3]:0; 
        double k3 = k.size() >= 5? k[4]:0; 
        
        x_u = x_d; 
        y_u = y_d; 

        double rd2, rd4, rd6, k_radial, dx, dy; 
        for(int i=0; i<20; i++)
        {
          rd2 = SQ(x_u) + SQ(y_u); 
          rd4 = SQ(rd2); 
          rd6 = rd4 * rd2; 
          k_radial = 1 + k1 * rd2 + k2 * rd4 + k3 * rd6; 
          dx = 2 * p1* x_u * y_u + p2 * (rd2 + 2 * SQ(x_u)); 
          dy = p1 * (rd2 + 2 * SQ(y_u)) + 2 * p2 * x_u * y_u; 
          x_u = (x_d - dx)/k_radial; 
          y_u = (y_d - dy)/k_radial; 
        }
    }
    return ;
} 



