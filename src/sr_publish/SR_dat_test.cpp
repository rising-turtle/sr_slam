#include <ros/ros.h>
#include "SR_reader.h"
#include "SR_writer.h"
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "SR_dat_test");
  ros::NodeHandle n; 

  CSReader r; 
  CSRWriter w;
  // r.loadAllData(); 
  // CSReader::iterator it = r.begin();
 
  sr_data d;
  if(!r.readOneFrameDat("/home/davidz/work/data/SwissRanger4000/dat/dataset_10/d1_0010.dat", d))
  {
    ROS_ERROR("SR_dat_test.cpp: failed to read data, return!");
    return 0;
  }
  if(w.writeSRDat("/home/davidz/work/data/SwissRanger4000/dat/dataset_1000.dat", d))
  {
    ROS_WARN("SR_dat_test.cpp: succeed to write data!");
  }

  // display it
  unsigned char* p =  (unsigned char*)(&d.intensity_[0]);
  // cv::Mat i_img(SR_HEIGHT, SR_WIDTH, CV_16UC1, p, SR_WIDTH*sizeof(SR_IMG_TYPE));
  cv::Mat i_img(SR_HEIGHT, SR_WIDTH, CV_16UC1, p);
  // cv::Mat mono_img; 
  // img2CV8UC1(i_img, mono_img); 
  cv::imshow("SR_image", i_img);
  cv::waitKey(100000);
  // test_camera_model(*it);

  return 0;
}
