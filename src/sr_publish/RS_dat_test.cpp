#include <ros/ros.h>
#include "realsense_reader.h"
#include "realsense_writer.h"
#include <opencv2/opencv.hpp>

//
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
// #include "../sr_slam/vtk_viewer.h"

void test_pc();
void test_io();

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "RS_dat_test");
  ros::NodeHandle n; 
  // test_io(); 
  test_pc();

  return 0;
}

void test_pc()
{
  CRSReader r; 

  rs_data rs;
  // if(!r.readOneFrameDat("/home/davidz/work/data/SwissRanger4000/dat/dataset_10/d1_0010.dat", d))
  if(!r.readRSID(1, rs))
  {
    ROS_ERROR("SR_dat_test.cpp: failed to read data, return!");
    return ;
  }
  vector<float> x(&rs.x_[0], &rs.x_[rs_data::SIZE]);
  vector<float> y(&rs.y_[0], &rs.y_[rs_data::SIZE]);
  vector<float> z(&rs.z_[0], &rs.z_[rs_data::SIZE]);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  int N = rs_data::SIZE;
  cloud->points.resize(N); 
  for(int i=0; i<rs_data::SIZE; i++)
  {
    pcl::PointXYZ& pt = cloud->points[i]; 
    pt.x = x[i]; pt.y = y[i]; pt.z = z[i];
  }

  // 3, show it
  pcl::visualization::CloudViewer viewer("Realsense PCL Viewer"); 
  while(!viewer.wasStopped())
  {
    viewer.showCloud(cloud);
    usleep(30000); // sleep 30 ms 
  } 
  // 4, dump it into file 
  cloud->width = N; 
  cloud->height = 1;
  pcl::io::savePCDFile("tmp.pcd", *cloud);
  return ;
}

void test_io()
{
  CRSReader r; 
  CRSWriter w;
  // r.loadAllData(); 
  // CSReader::iterator it = r.begin();
 
  rs_data d;
  // if(!r.readOneFrameDat("/home/davidz/work/data/SwissRanger4000/dat/dataset_10/d1_0010.dat", d))
  if(!r.readRSID(1, d))
  {
    ROS_ERROR("SR_dat_test.cpp: failed to read data, return!");
    return ;
  }
  // if(w.writeSRDat("/home/davidz/work/data/SwissRanger4000/dat/dataset_1000.dat", d))
  if(w.writeRSFile("./dataset_1000.dat", d))
  {
    ROS_WARN("SR_dat_test.cpp: succeed to write data!");
  }

  // display it
  unsigned char* p =  (unsigned char*)(&d.intensity_[0]);
  // cv::Mat i_img(SR_HEIGHT, SR_WIDTH, CV_16UC1, p, SR_WIDTH*sizeof(SR_IMG_TYPE));
  cv::Mat i_img(rs_data::HEIGHT, rs_data::WIDTH, CV_8UC1, p);
  // cv::Mat mono_img; 
  // img2CV8UC1(i_img, mono_img); 
  cv::imshow("RS_image", i_img);
  cv::waitKey(100000);
  // test_camera_model(*it);

  return ;
}


