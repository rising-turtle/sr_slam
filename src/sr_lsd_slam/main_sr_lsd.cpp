
#include "sr_lsd_slam.h"
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
cv::Mat threshold_label(cv::Mat& img, cv:: Mat& dpt, int img_thre, float dpt_thre, int dpt_thre_n);
void filter_img_dpt(cv::Mat& img, cv::Mat& dpt);  // threshold based filter
cv::Mat threshold_label2(cv::Mat& img, int img_thre, int neighbor_thre_n); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "SR_LSD_SLAM"); 
  dynamic_reconfigure::Server<lsd_slam_core::LSDParamsConfig> srv(ros::NodeHandle("~"));
  srv.setCallback(dynConfCb);

  // get camera calibration in form of an undistorter object.
  // if no undistortion is required, the undistorter will just pass images through.
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
  // SlamSystem* system = new SlamSystem(w, h, K, doSlam);
  CSRSlamSystem* system = new CSRSlamSystem(w, h, K, false); //
  system->setVisualization(outputWrapper);

  // load SR data 
  CSReader * sr_reader = new CSReader(); 
  bool ret = sr_reader->loadAllData(); 
  if(!ret)
  {
    ROS_ERROR("main_sr_lsd.cpp: failed to load SR data!"); 
    return ;
  }
  
  // process each frame 
  sr_data sr_frame; 
  bool finished = false; 
  sr_frame = sr_reader->get_current_frame(finished); 
  cv::Mat img, dpt; 
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
      threshold_label2(img, 25, 1);
      undistorter->undistort(img, rect_img); 
      undistorter->undistort(dpt, rect_dpt);
      
      cv::Mat he_img; 
      cv::equalizeHist(rect_img, he_img); 

      cv::imshow("Display window", he_img); 

      cv::waitKey(20); 
      
      // use specifig timestamp 
      fakeTimeStamp = sr_frame.timestamp_;

      if(runningIDX == 0)
          system->srInit(he_img.data, (float*)(rect_dpt.data), fakeTimeStamp, runningIDX); 
          // system->randomInit(img.data, fakeTimeStamp, runningIDX);
      else
          system->trackFrame(he_img.data, runningIDX, 1, fakeTimeStamp); 
      runningIDX++; 

      // fakeTimeStamp += 0.03;

      if(fullResetRequested)
      {
        printf("FULL RESET!\n");
        delete system;

        // system = new SlamSystem(w, h, K, doSlam);
        system = new CSRSlamSystem(w, h, K, doSlam);
        system->setVisualization(outputWrapper);

        fullResetRequested = false;
        runningIDX = 0;
      }

      ros::spinOnce();
      sr_frame = sr_reader->get_current_frame(finished); 

      if(!ros::ok())
        break;

    }catch(...)
    {
      ROS_ERROR("main_sr_lsd.cpp: caught exception when processing sr_frame %d", sr_reader->curr_frame_); 
    }
  }

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


void filter_img_dpt(cv::Mat& img, cv::Mat& dpt)
{
  int img_thre = 25; 
  float dpt_thre = 0.05; 
  int dpt_thre_n = 4; 
  cv::Mat bin_img =  threshold_label(img, dpt, img_thre, dpt_thre, dpt_thre_n); 
  
  cv::imshow("bin_img", bin_img);
  cv::waitKey(0);

  int rows = img.rows; 
  int cols = img.cols;
  for(int i=1; i<rows-1; i++)
    for(int j=1; j<cols-1; j++)
    {
      if(bin_img.at<unsigned char>(i, j) == 0)
        img.at<unsigned char>(i, j) = 0; 
    }
}

cv::Mat threshold_label2(cv::Mat& img, int img_thre, int neighbor_thre_n)
{
    int rows = img.rows; 
    int cols = img.cols; 
    unsigned char ini_v = 255;
    cv::Mat bin_img = cv::Mat(rows, cols, CV_8UC1, ini_v);//img.clone();
    int i_v ; 
    for(int i=1; i<rows-1; i++)
      for(int j=1; j<cols-1; j++)
      {
        i_v = img.at<unsigned char>(i-1, j) + img.at<unsigned char>(i, j) + img.at<unsigned char>(i+1, j) + 
          img.at<unsigned char>(i-1, j-1) + img.at<unsigned char>(i, j-1) + img.at<unsigned char>(i+1, j-1) + 
          img.at<unsigned char>(i-1, j+1) + img.at<unsigned char>(i, j+1) + img.at<unsigned char>(i+1, j+1);
        i_v = (int)(i_v/9);

        bin_img.at<unsigned char>(i,j) = i_v>img_thre? 255: 0;  
      }
    
    cv::Mat t_img = bin_img.clone();

    // depth threshold 
     for(int i=1; i<rows-1; i++)
      for(int j=1; j<cols-1; j++)
      {
        if(t_img.at<unsigned char>(i, j) == 0) continue; 
        int bad_neighbor = 0; 
        if(t_img.at<unsigned char>(i-1, j-1) == 0) ++ bad_neighbor; 
        if(bad_neighbor > neighbor_thre_n) { bin_img.at<unsigned char>(i, j) = 0;  continue;}
        
        if(t_img.at<unsigned char>(i, j-1) == 0) ++ bad_neighbor; 
        if(bad_neighbor > neighbor_thre_n) { bin_img.at<unsigned char>(i, j) = 0;  continue;}

        if(t_img.at<unsigned char>(i+1, j-1) == 0) ++ bad_neighbor; 
        if(bad_neighbor > neighbor_thre_n) { bin_img.at<unsigned char>(i, j) = 0;  continue;}

        if(t_img.at<unsigned char>(i-1, j) == 0) ++ bad_neighbor; 
        if(bad_neighbor > neighbor_thre_n) { bin_img.at<unsigned char>(i, j) = 0;  continue;}

        if(t_img.at<unsigned char>(i+1, j) == 0) ++ bad_neighbor; 
        if(bad_neighbor > neighbor_thre_n) { bin_img.at<unsigned char>(i, j) = 0;  continue;}

        if(t_img.at<unsigned char>(i-1, j+1) == 0) ++ bad_neighbor; 
        if(bad_neighbor > neighbor_thre_n) { bin_img.at<unsigned char>(i, j) = 0;  continue;}

        if(t_img.at<unsigned char>(i, j+1) == 0) ++ bad_neighbor; 
        if(bad_neighbor > neighbor_thre_n) { bin_img.at<unsigned char>(i, j) = 0;  continue;}

        if(t_img.at<unsigned char>(i+1, j+1) == 0) ++ bad_neighbor; 
        if(bad_neighbor > neighbor_thre_n) { bin_img.at<unsigned char>(i, j) = 0;  continue;}

        if(i-2 > 0 && t_img.at<unsigned char>(i-2, j) == 0) ++ bad_neighbor; 
        if(bad_neighbor > neighbor_thre_n) { bin_img.at<unsigned char>(i, j) = 0;  continue;}

        if(i+2 < rows && t_img.at<unsigned char>(i+2, j) == 0) ++ bad_neighbor; 
        if(bad_neighbor > neighbor_thre_n) { bin_img.at<unsigned char>(i, j) = 0;  continue;}

        if(j-2 > 0 && t_img.at<unsigned char>(i, j-2) == 0) ++ bad_neighbor;
        if(bad_neighbor > neighbor_thre_n) { bin_img.at<unsigned char>(i, j) = 0;  continue;}

        if(j+2 < cols && t_img.at<unsigned char>(i, j+2) == 0) ++ bad_neighbor; 
        if(bad_neighbor > neighbor_thre_n) { bin_img.at<unsigned char>(i, j) = 0;  continue;}
        // bin_img.at<unsigned char>(i, j) = 0; 
      } 

    for(int i=1; i<rows; i++)
      for(int j=1; j<cols; j++)
      {
        if(bin_img.at<unsigned char>(i, j) == 0)  img.at<unsigned char>(i, j) = 0;
      }

    return bin_img;

}

cv::Mat threshold_label(cv::Mat& img, cv:: Mat& dpt, int img_thre, float dpt_thre, int dpt_thre_n)
{
    int rows = img.rows; 
    int cols = img.cols; 
    unsigned char ini_v = 255;
    cv::Mat bin_img = cv::Mat(rows, cols, CV_8UC1, ini_v);//img.clone();
    int i_v ; 
    for(int i=1; i<rows-1; i++)
      for(int j=1; j<cols-1; j++)
      {
        i_v = img.at<unsigned char>(i-1, j) + img.at<unsigned char>(i, j) + img.at<unsigned char>(i+1, j) + 
          img.at<unsigned char>(i-1, j-1) + img.at<unsigned char>(i, j-1) + img.at<unsigned char>(i+1, j-1) + 
          img.at<unsigned char>(i-1, j+1) + img.at<unsigned char>(i, j+1) + img.at<unsigned char>(i+1, j+1);
        i_v = (int)(i_v/9);

        bin_img.at<unsigned char>(i,j) = i_v>img_thre? 255: 0;  
      }

    // depth threshold 
    float g_v, n_v ; // neighbor depth value 
     for(int i=1; i<rows-1; i++)
      for(int j=1; j<cols-1; j++)
      {
        int good_neighbor = 0; 

        if(bin_img.at<unsigned char>(i, j) == 0) continue; 
        
        g_v = dpt.at<float>(i, j); 
        n_v = dpt.at<float>(i-1, j-1); 
        if(fabs(g_v - n_v) < dpt_thre && bin_img.at<unsigned char>(i-1, j-1) == 255) ++ good_neighbor; 
        if(good_neighbor > dpt_thre_n) continue; 
        
        n_v = dpt.at<float>(i, j-1); 
        if(fabs(g_v - n_v) < dpt_thre && bin_img.at<unsigned char>(i, j-1) == 255) ++ good_neighbor; 
        if(good_neighbor > dpt_thre_n) continue; 

        n_v = dpt.at<float>(i+1, j-1); 
        if(fabs(g_v - n_v) < dpt_thre && bin_img.at<unsigned char>(i+1, j-1) == 255) ++ good_neighbor; 
        if(good_neighbor > dpt_thre_n) continue; 

        n_v = dpt.at<float>(i-1, j); 
        if(fabs(g_v - n_v) < dpt_thre && bin_img.at<unsigned char>(i-1, j) == 255) ++ good_neighbor; 
        if(good_neighbor > dpt_thre_n) continue; 

        n_v = dpt.at<float>(i+1, j); 
        if(fabs(g_v - n_v) < dpt_thre && bin_img.at<unsigned char>(i+1, j) == 255) ++ good_neighbor; 
        if(good_neighbor > dpt_thre_n) continue; 

        n_v = dpt.at<float>(i-1, j+1); 
        if(fabs(g_v - n_v) < dpt_thre && bin_img.at<unsigned char>(i-1, j+1) == 255) ++ good_neighbor; 
        if(good_neighbor > dpt_thre_n) continue; 

        n_v = dpt.at<float>(i, j+1); 
        if(fabs(g_v - n_v) < dpt_thre && bin_img.at<unsigned char>(i, j+1) == 255) ++ good_neighbor; 
        if(good_neighbor > dpt_thre_n) continue; 

        n_v = dpt.at<float>(i+1, j+1); 
        if(fabs(g_v - n_v) < dpt_thre && bin_img.at<unsigned char>(i+1, j+1) == 255) ++ good_neighbor; 
        if(good_neighbor > dpt_thre_n) continue; 

        bin_img.at<unsigned char>(i, j) = 0; 
      } 

    return bin_img;
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
