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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include "SR_reader.h"
#include <sstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include "SR_interface.h"

using namespace std;
using namespace cv;

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

cv::Mat hist_image(cv::Mat& img); 
cv::Mat threshold_label(cv::Mat& img, cv:: Mat& dpt, int img_thre, float dpt_thre, int dpt_thre_n);
cv::Mat threshold_label2(cv::Mat& img, int img_thre, int neighbor_thre_n);

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
 
  // publish image
  ros::Publisher intensity_img_pub = n.advertise<sensor_msgs::Image>("/sr_img_raw", 10);
  ros::Publisher depth_img_pub = n.advertise<sensor_msgs::Image>("/sr_dpt_raw", 10);

  // publish sw_array 
  ros::Publisher sr_array_pub = n.advertise<std_msgs::UInt8MultiArray>("/sr_array", 10); 
  
  ros::NodeHandle nh_p("~");
  // nh_p.setParam("sr_source", "SR_FILE"); 
  // nh_p.setParam("sr_source", "SR_CAM");
  // nh_p.setParam("sr_end_frame", 10); 
  // nh_p.setParam("sr_new_file_version", false); // test old file version  
  string sr_source;
  int sr_publish_rate; 
  int N; // total send 100 frames 
  nh_p.param("sr_source", sr_source, std::string("SR_FILE"));
  nh_p.param("sr_publish_rate", sr_publish_rate, 10);
  nh_p.param("sr_publish_max", N, 500);
  
  // whether to publish intensity img or depth img 
  bool b_new_sr_version; 
  bool b_pub_intensity_img; 
  bool b_pub_depth_img;
  bool b_need_syn_from_receiver;
  bool b_display_histogram; 
  bool b_threshold_display; 
  cv_bridge::CvImage out_msg; 
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; // image type
  cv_bridge::CvImage out_msg_dpt;
  out_msg_dpt.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  // out_msg.encoding = sensor_msgs::image_encodings::BGR8; // image type
  ros::Time msg_timestamp; // ros time, used in the msg.header

  nh_p.param("sr_new_file_version", b_new_sr_version, false); 
  nh_p.param("sr_publish_intensity", b_pub_intensity_img, false); 
  nh_p.param("sr_publish_depth", b_pub_depth_img, false); 
  nh_p.param("sr_syn_from_receiver", b_need_syn_from_receiver, true);
  nh_p.param("sr_display_histogram", b_display_histogram, false);
  nh_p.param("sr_threshold_display", b_threshold_display, false);

  if(b_pub_intensity_img)
  {
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
  }
  if(b_pub_depth_img)
  {
    cv::namedWindow( "Display window dpt", cv::WINDOW_AUTOSIZE );// Create a window for display.
  }

  // 1, generate sr data; 
  CSRInterface sr_instance; 
  std_msgs::UInt8MultiArray sr_array; 

  int count = 0; 
  if(!sr_instance.open())
  {
    ROS_ERROR("sr_publisher.cpp: open sr_instance fail!");
    return ;
  }else
  {
    ROS_INFO("sr_publisher.cpp: open sr_instance succeed!");
  }
  
  // 2, syn at first step
  if(b_need_syn_from_receiver)
  {
    ROS_WARN("sr_publisher.cpp: need syn from receiver, wait for syn!");
    first_syn(syn_pub_);
  }
  ros::Rate r(sr_publish_rate); // broadcast rate 10hz
  
  ROS_INFO("sr_publisher.cpp: after syn!");

  unsigned char * pImg = 0;
  unsigned char * pDpt = 0; 
  int wait_time =  20;
  while(ros::ok())
  {
    sr_array.data.clear();
    if(sr_instance.get(sr_array))
    {
      sr_array_pub.publish(sr_array);

      // new version: dpt , img 
      if(b_new_sr_version) 
      {
        pDpt = sr_array.data.data(); 
        pImg = sr_array.data.data() +   SR_SIZE*sizeof(unsigned short);
      }else // old version: z, x, y, img,
      {
        // TODO set dpt 
        pImg = sr_array.data.data(); // + 3*SR_SIZE*sizeof(float);
        pDpt = sr_array.data.data() + SR_SIZE*sizeof(unsigned char) + 2*SR_SIZE*sizeof(float);
      }

      if(b_pub_intensity_img)
      {
        cv::Mat i_img(SR_HEIGHT, SR_WIDTH, CV_8UC1, pImg, SR_WIDTH*sizeof(unsigned char)); 
        cv::Mat cv_img;
        cv_img = i_img.clone(); // CV_8UC1
        
        if(b_display_histogram)
        {
          cv::Mat hist = hist_image(cv_img); 
          cv::imshow("hist ", hist); 
          wait_time = 5000;
        }

        if(b_threshold_display)
        {
          int threshold;
          // cv::Mat d_img(SR_HEIGHT, SR_WIDTH, CV_32FC1, (float*)pDpt, SR_WIDTH*sizeof(float));
          nh_p.param("sr_threshold_value", threshold, 25);
          // cv::Mat bin_img = threshold_label(cv_img, d_img, threshold, 0.1, 3); 
          cv::Mat bin_img = threshold_label2(cv_img, threshold, 1); 
          cv::imshow("threshold label", bin_img); 
          wait_time = 50;
        }

        cv::imshow( "Display window", cv_img );                   // Show our image inside it.
        // cv::imshow( "Display window", i_img );                   // Show our image inside it.
        msg_timestamp = ros::Time();
        out_msg.header.stamp = msg_timestamp;
        out_msg.header.seq = count+1;
        out_msg.image = cv_img;
        intensity_img_pub.publish(out_msg);
      }
      if(b_pub_depth_img)
      {
         cv::Mat d_img(SR_HEIGHT, SR_WIDTH, CV_32FC1, (float*)pDpt, SR_WIDTH*sizeof(float));
         cv::Mat cv_dpt(SR_HEIGHT, SR_WIDTH, CV_16UC1);
         for(int i=0; i< SR_HEIGHT; i++)
           for(int j=0; j<SR_WIDTH; j++)
          {
            cv_dpt.at<unsigned short>(i, j) = (unsigned short)(d_img.at<float>(i, j) * 1000);
          }
        cv::imshow("Display window dpt", cv_dpt);
        out_msg_dpt.header.stamp = msg_timestamp;
        out_msg_dpt.header.seq = count+1;
        out_msg_dpt.image = cv_dpt; //cv_img;
        // intensity_img_pub.publish(out_msg);
        depth_img_pub.publish(out_msg_dpt);
      }
       
      cv::waitKey(wait_time);

      ros::spinOnce();
      if( ++count > N && N > 0) break; 
      ROS_INFO("sr_publisher.cpp: publish %d frame", count);
      if(sr_source == "SR_FILE")
        r.sleep(); 
    }
    else if(sr_source == "SR_FILE")
    {
      ROS_WARN("sr_publisher.cpp: read files have been traversed!"); 
      break;
    }
    else
    {
      // ROS_WARN("sr_publisher.cpp: failed to get next frame, exit!");
      // break; 
    }
  }
  if(b_pub_intensity_img)
  {
    cv::destroyWindow("Display window");
  }

  ROS_INFO("sr_publisher.cpp: exit after task!");
  // exit_pub_.publish(b_syn_ok);
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
        if(bin_img.at<unsigned char>(i, j) == 0) continue; 
        g_v = dpt.at<float>(i, j); 
        int good_neighbor = 0; 
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

cv::Mat hist_image(cv::Mat& gray)
{
    int histSize = 256;    // bin size
    float range[] = { 0, 255 };
    const float *ranges[] = { range };

    cv::MatND hist;
    cv::calcHist( &gray, 1, 0, Mat(), hist, 1, &histSize, ranges, true, false );
      double total;
    total = gray.rows * gray.cols;
    for( int h = 0; h < histSize; h++ )
         {
            float binVal = hist.at<float>(h);
            if(binVal > 0)
            {
              cout<<h<<" : "<<binVal<<" ";
            }
         }
    cout<<endl;
    // Plot the histogram
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
 
    cv::Mat histImage( hist_h, hist_w, CV_8UC1, Scalar( 0,0,0) );
    cv::normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
     
    for( int i = 1; i < histSize; i++ )
    {
       cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
                       cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
    }
    return histImage; 
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



