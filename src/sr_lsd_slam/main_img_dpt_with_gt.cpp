
#include "sr_lsd_slam.h"
#include "dpt_lsd_slam.h"
#include "dpt_lsd_slam_ori.h"
#include "util/settings.h"
#include "util/Undistorter.h"
#include "util/globalFuncs.h"
#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
#include "IOWrapper/ROS/rosReconfigure.h"
#include <ros/package.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "SR_reader.h"

using namespace lsd_slam; 

void convert16UC_8UC(cv::Mat& in_img, cv::Mat& out_img);
void srCallback(sr_data& , cv::Mat& , cv::Mat& );   // process each frame to obtain intensity and depth image

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "imgs_dpt_lsd"); 
  std::string calibFile;
  Undistorter* undistorter = 0;
  if(ros::param::get("~calib", calibFile))
  {
    ROS_INFO("main_sr_lsd.cpp: calibration file is %s", calibFile.c_str());
    undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
    ros::param::del("~calib");
  }

  if(undistorter == 0)
  {
    printf("need camera calibration file! (set using _calib:=FILE)\n");
    exit(0);
  }

  int w = undistorter->getOutputWidth();
  int h = undistorter->getOutputHeight();

  int w_inp = undistorter->getInputWidth();
  int h_inp = undistorter->getInputHeight();

  float fx = undistorter->getK().at<double>(0, 0);
  float fy = undistorter->getK().at<double>(1, 1);
  float cx = undistorter->getK().at<double>(2, 0);
  float cy = undistorter->getK().at<double>(2, 1);
  Sophus::Matrix3f K;
  K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

  printf("main_sr_lsd.cpp: undistorter has fx %f, fy %f cx %f cy %f\n", fx, fy, cx, cy);

  // make output wrapper. just set to zero if no output is required.
  Output3DWrapper* outputWrapper = new ROSOutput3DWrapper(w,h);

  // make slam system
  doSlam = true; 
  int kf_every_k = 10; 
  int look_back_kf = 1; 

  // NOTICE: if enable gt, needs to change the look_back_kf in contruction function, 
  CDLSDSlamSystem* system = new CDLSDSlamSystem(w, h, K, doSlam, kf_every_k , look_back_kf);  // doSlam
  // CDLSDSlamSystemOri* system = new CDLSDSlamSystemOri(w, h, K, doSlam); 
  system->setVisualization(outputWrapper);

  // load SR data 
  CSReader * sr_reader = new CSReader(); 
  bool ret = sr_reader->loadAllData(); 

  // set gt files 
  ros::NodeHandle nh("~"); 
  string gt_file = ""; 
  nh.param("gt_file", gt_file, gt_file);
  if(sr_reader->synFromGT(gt_file))
  {
    ROS_WARN("main_img_dpt_with_gt.cpp: succeed to read GT from file %s", gt_file.c_str()); 
  }else{
    ROS_ERROR("main_img_dpt_with_gt.cpp: though compute in/outlier gt, failed to set gt");
    return -1;
  }

  if(!ret)
  {
    ROS_ERROR("main_sr_lsd.cpp: failed to load SR data!"); 
    return ;
  }
  
  // process each frame 
  sr_data sr_frame; 
  bool finished = false; 
  sr_frame = sr_reader->get_current_frame(finished); 
  cv::Mat img, blur_img, dpt; 
  cv::Mat rect_img = cv::Mat(h, w, CV_8U); 
  cv::Mat rect_dpt = cv::Mat(h, w, CV_32F);

  int runningIDX = 0; 
  float fakeTimeStamp = 0;
  
  cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);

  while(!finished)
  {
    try{
      do{
        usleep(100); 
        if(!ros::ok())
        {
          return; 
        }
      }while(pauseForTracking);
      srCallback(sr_frame, img, dpt); 

      undistorter->undistort(img, rect_img);
      undistorter->undistort(dpt, rect_dpt);
      
      cv::Mat he_img; 
      cv::equalizeHist(rect_img, he_img); 
      fakeTimeStamp = sr_frame.timestamp_;
      
      if(sr_frame.b_gt_has_been_set_)
      {
        ROS_INFO("main_img_dpt_with_gt.cpp: frame %lf has gt %f %f %f %f %f %f %f", sr_frame.timestamp_, sr_frame.gt_pv_[0], 
            sr_frame.gt_pv_[1], sr_frame.gt_pv_[2], sr_frame.gt_pv_[3], sr_frame.gt_pv_[4], sr_frame.gt_pv_[5], sr_frame.gt_pv_[6]);
      }

      if(runningIDX == 0)
      {
        system->gtDepthInit(he_img.data, (float*)(rect_dpt.data), fakeTimeStamp, runningIDX);
        system->setCurrentKFGT(&sr_frame.gt_pv_[0]);
      }
      else 
      {
          system->trackFrameGT(he_img.data, (float*)(rect_dpt.data), runningIDX, 1, fakeTimeStamp, sr_frame.b_gt_has_been_set_, &(sr_frame.gt_pv_[0]));
      }
      runningIDX++; 
    
      ros::spinOnce();
      sr_frame = sr_reader->get_current_frame(finished); 

      usleep(20000);
      if(!ros::ok())
        break;

    }catch(...)
    {
      ROS_ERROR("main_sr_lsd.cpp: caught exception when processing sr_frame %d", sr_reader->curr_frame_); 
    }
  }
  // system->saveKFGraph("./lsd_slam/lsd_graph.log");
  return 1;
}

void srCallback(sr_data& sr, cv::Mat& img, cv::Mat& dpt)   // process each frame to obtain intensity and depth image 
{
  cv::Mat intensity_img = cv::Mat(SR_HEIGHT, SR_WIDTH, CV_16UC1, sr.intensity_);
  convert16UC_8UC(intensity_img, img); 
  
  ros::NodeHandle nh("~"); 
  bool b_new_version; 
  string data_suffix; 
  nh.param("sr_data_suffix", data_suffix, string("bdat")); 
  nh.param("sr_new_file_version", b_new_version, false);

   // vector<float> tmp_d(SR_SIZE); 
  if(data_suffix == string("dat") || !b_new_version)
  {
    // for(int i=0; i<SR_SIZE; i++)
    //  tmp_d[i] = sr.z_[i]; 
     dpt = cv::Mat(SR_HEIGHT, SR_WIDTH, CV_32FC1, &sr.z_[0]);
     // cv::Mat dpt_tmp = cv::Mat(SR_HEIGHT, SR_WIDTH, CV_32FC1, tmp_d.data());
     // cv::imshow("srCallWindow", dpt_tmp);
     // cv::waitKey(0);
  }else
  {
    // reconstruct the sr distance data
    static float dis[SR_SIZE] = {0};
    for(int i=0; i<SR_SIZE; i++)
    {
      dis[i] = (SR_IMG_TYPE)(sr.dis_[i]*0.001);
    }

    dpt = cv::Mat(SR_HEIGHT, SR_WIDTH, CV_32FC1, dis);
    // dpt = dpt_tmp.clone();
  }

  return ;
}

void convert16UC_8UC(cv::Mat& in_img, cv::Mat& out_img)
{
  static vector<unsigned char> convert_table;
  if(convert_table.size() == 0)
  {
    const static int N  = 65536;
    convert_table.resize(N, 0); 
    for(int i=0; i<N; i++)
    {
      convert_table[i] = (unsigned char)(sqrt((double)(i))); 
    }
  }
  out_img = cv::Mat(in_img.size(), CV_8UC1); 
  unsigned short * pIn = (unsigned short*)in_img.data; 
  unsigned char * pOut = out_img.data; 
  int total = in_img.rows*in_img.cols;
  const static unsigned short LIMIT_NUM = 65000;
  unsigned char max_c = 0;

  // delete pixels > LIMIT_NUM 
  // find max_char
  for(int i=0; i<total; ++i)
  {
    if(*pIn >= LIMIT_NUM) *pOut = 0; 
    else{
      *pOut = convert_table[*pIn];
    }
    if(*pOut > max_c) {max_c = (*pOut);}
    ++pIn;
    ++pOut;
  }

  if(max_c == 0) 
  {
    cout<<"OpenniWrapper.cpp: max_c = 0, something is wrong!"<<endl;
    return; 
  }
  // rescale to 0~255
  double scale_factor = 255./(double)max_c; 
  pOut = out_img.data; 
  for(int i=0; i<total; i++)
  {
    *pOut = (unsigned char)((*pOut)*scale_factor);
    ++pOut;
  }
  return ;
}
