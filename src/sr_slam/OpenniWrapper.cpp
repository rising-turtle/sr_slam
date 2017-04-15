/*
 * David Z, Apr 20, 2015
 * wrapper for the openni_listenner in rgbdslam, 
 * to handle the swiss ranger msg, and construct the wrapped node type
 *
 * */

#include "OpenniWrapper.h"
//Documentation see header file
#include "pcl/ros/conversions.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
//#include "pcl/common/transform.h"
#include "pcl_ros/transforms.h"
#include "openni_listener.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cv.h>
//#include <ctime>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "NodeWrapper.h"
#include "plane_node.h"
#include "graph_wrapper.h"
#include "misc.h"
//#include <image_geometry/pinhole_camera_model.h>
//#include "pcl/ros/for_each_type.h"

//For rosbag reading
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "parameter_server.h"
#include "scoped_timer.h"
#include "paramSrvMi.h"
//for comparison with ground truth from mocap and movable cameras on robots
#include <tf/transform_listener.h>
#include <vector>
// #include "GraphWrapper.h"
// #include "DuoGraph.h"
#include "imu_reader.h"

using namespace std;
#define D2R(d) (((d)*M_PI)/(180.))

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;      

// swiss ranger msg type 
typedef message_filters::Subscriber<std_msgs::UInt8MultiArray> sr_array_sub_type; 

COpenniWrapper::COpenniWrapper(GraphManager* g_mgr):
    OpenNIListener(g_mgr)
    // p_imu_(new CIMUReader)
    // p_imu_(0)  // do not need IMU yet
// COpenniWrapper::COpenniWrapper(CDuoGraph* g_mgr) : 
//   OpenNIListener(g_mgr->getFrontGraph()),
//   m_duo_mgr(g_mgr)
{
    ParameterServer* ps = ParameterServer::instance();
    int q = ps->get<int>("subscriber_queue_size");
    std::string bagfile_name = ps->get<std::string>("bagfile_name");
    std::string visua_tpc = ps->get<std::string>("topic_image_mono");
    std::string depth_tpc = ps->get<std::string>("topic_image_depth");
    std::string cinfo_tpc = ps->get<std::string>("camera_info_topic");

    // sr_array_topic 
    ParamSrvMi* ps_mi = ParamSrvMi::instanceMi();
    std::string sr_array_tpc = ps_mi->get<std::string>("topic_swiss_ranger");

    ros::NodeHandle nh;
    tflistener_ = new tf::TransformListener(nh);

    //my publishers
    slamLost_pub = nh.advertise<std_msgs::String>("SlamLost", 1000);

     if(no_cloud_sync_ != NULL)
    {
	if(bagfile_name.empty())
	{
	    delete no_cloud_sync_;
	    delete visua_sub_;
	    delete depth_sub_;
	    delete cinfo_sub_;
	    ROS_INFO("no_cloud_sync succeed, override it!");

	    visua_sub_ = new image_sub_type(nh, visua_tpc, q);
	    depth_sub_ = new image_sub_type(nh, depth_tpc, q);
	    cinfo_sub_ = new cinfo_sub_type(nh, cinfo_tpc, q);
	    no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q),  *visua_sub_, *depth_sub_, *cinfo_sub_);
	}
	else
	{ 
	    delete no_cloud_sync_;
	    delete rgb_img_sub_;
	    delete depth_img_sub_;
	    delete cam_info_sub_;
	    ROS_INFO("no_cloud_sync succeed, override it!");

	    depth_img_sub_ = new BagSubscriber<sensor_msgs::Image>();
	    rgb_img_sub_ = new BagSubscriber<sensor_msgs::Image>();
	    cam_info_sub_ = new BagSubscriber<sensor_msgs::CameraInfo>();
	    no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q),  *rgb_img_sub_, *depth_img_sub_, *cam_info_sub_);
	}
	no_cloud_sync_->registerCallback(boost::bind(&COpenniWrapper::noCloudCallback, this, _1, _2, _3));
    }

    // create the swiss ranger subscriber 
    sr_array_sub_ = new sr_array_sub_type(nh, sr_array_tpc, q);
    sr_array_sub_->registerCallback(boost::bind(&COpenniWrapper::srCallback, this, _1));

    // create the swiss ranger syn interfaces
    sr_syn_sub_ = new message_filters::Subscriber<std_msgs::Bool>(nh, "/syn", 1);
    sr_syn_sub_->registerCallback(boost::bind(&COpenniWrapper::srSynCb, this, _1));
    sr_ack_pub_ = nh.advertise<std_msgs::Bool>("/ack",1);


    // create the 2D UI interfaces 
    // r_pos_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/robot_pos", 1); 

}

COpenniWrapper::~COpenniWrapper(){}

// callback for syn with the sr_publisher 
void COpenniWrapper::srSynCb(const std_msgs::Bool::ConstPtr& syn)
{
  ROS_INFO("OpenniWrapper.cpp: receive syn from sr_publisher, response ack!");
  std_msgs::BoolPtr ack(new std_msgs::Bool);
  ack->data = true; 
  sr_ack_pub_.publish(ack);
}

void COpenniWrapper::convert16UC_8UC(cv::Mat& in_img, cv::Mat& out_img)
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

void COpenniWrapper::preprocessImg(cv::Mat& in_img, cv::Mat& out_img, bool use_filter)
{
  if(in_img.type() == CV_16UC1)
  {
    // 
    convert16UC_8UC(in_img, out_img);
  }else if(in_img.type() == CV_8UC1)
  {
    // nothing to worry 
    out_img = in_img.clone();
  }else
  {
    cout<<"OpenniWrapper.cpp: something must has error!"<<endl;
    return ;
  }

  if(use_filter)
  {
    // TODO: filter? Histogram Equalization and Gaussian filter
  }
  return ;
}

// callback for receiving the sr_array_msg
void COpenniWrapper::srCallback(const std_msgs::UInt8MultiArray::ConstPtr& sr_array_ptr)
{
  ParameterServer* ps = ParameterServer::instance();

  // 1, construct a node, and 2 display the UI, 3 call nodeprocess 
  const unsigned char* pD = sr_array_ptr->data.data(); 

  // 2, distinguish either it contains (u,v,d) or (X,Y,Z) 
   
  // firstly, construct a grey image and a depth image 
  static std::vector<unsigned char> buf_img(sr_n, 0); 
  static std::vector<unsigned short> buf_depth(sr_n, 0); 
  static std::vector<float> buf_x(sr_n, 0); 
  static std::vector<float> buf_y(sr_n, 0); 
  static std::vector<float> buf_z(sr_n, 0);
  const static unsigned int distance_size = sr_n*(sizeof(unsigned short) + sizeof(unsigned char));
  const static unsigned int coordinate_size = sr_n*(sizeof(float)*3 + sizeof(unsigned char));
  if(sr_array_ptr->data.size() == distance_size)
  {
    memcpy(buf_img.data(), pD, sr_n*sizeof(unsigned char)); 
    memcpy(buf_depth.data(), pD + sr_n*sizeof(unsigned char), sr_n*sizeof(unsigned short)); 
    // memcpy(buf_depth.data(), pD,  sr_n*sizeof(unsigned short)); 
    // memcpy(buf_img.data(), pD + sr_n*sizeof(unsigned short), sr_n*sizeof(unsigned char));
  }
  else if(sr_array_ptr->data.size() == coordinate_size)
  {
    memcpy(buf_img.data(), pD, sr_n*sizeof(unsigned char)); 
    memcpy(buf_x.data(), pD + sr_n*sizeof(unsigned char), sr_n*sizeof(float)); 
    memcpy(buf_y.data(), pD + sr_n*(sizeof(unsigned char) + sizeof(float)), sr_n*sizeof(float)); 
    memcpy(buf_z.data(), pD + sr_n*(sizeof(unsigned char) + sizeof(float)*2), sr_n*sizeof(float));
    for(int i=0; i<sr_n; i++)
      buf_depth[i] = (unsigned short)(buf_z[i]*1000);
  }else
  {
    ROS_ERROR("OpenniWrapper.cpp: something horrible happens!");
    return ;
  }

  // Apr.22 2015,  
  // cv::Mat grey_img_tmp(sr_rows, sr_cols, CV_8UC1, buf_img.data()); 
  // cv::Mat depth_img(sr_rows, sr_cols, CV_16UC1, buf_depth.data());

  cv::Mat grey_img_(sr_rows, sr_cols, CV_8UC1, buf_img.data()); 
  cv::Mat depth_img_(sr_rows, sr_cols, CV_16UC1, buf_depth.data());

  // cv::Mat grey_img_; 
  // cv::Mat depth_img_; 
  // cv::transpose(grey_img_tmp, grey_img_);
  // cv::transpose(depth_img_tmp, depth_img_); 

  cv::Mat grey_mono8_img_;
  // cv::Mat depth_mono8_img_; // this is actually a class member in OpenniListener
  preprocessImg(grey_img_, grey_mono8_img_); 
  // preprocessImg(depth_img_, depth_mono8_img_);

  ParamSrvMi * ps_MI = ParamSrvMi::instanceMi(); 
  if(ps_MI->get<bool>("use_gaussian_filter"))
  {
    static int number_successful_added = 0; 
    static std::ofstream time_vro_go("time_gaussian_filter.log"); 
    // 3*3 gaussian filter 
    cv::Mat grey_gaussian = grey_mono8_img_.clone(); 
    double begin_time_ms = ros::Time::now().toSec()*1000.; 
    cv::GaussianBlur(grey_gaussian, grey_mono8_img_, cv::Size(3,3), 0, 0);
    double end_time_ms = ros::Time::now().toSec()*1000.; 
    time_vro_go << ++ number_successful_added <<" "<< end_time_ms - begin_time_ms<<std::endl;
  }

  cv::Mat depth_mono8_mask_; // this one will be used in Node Construction as a mask 
  cv::Mat depth_clone_img_ = depth_img_.clone(); // because depthToCV8UC1 would change the type of input depth image, so make a copy
  depthToCV8UC1(depth_clone_img_, depth_mono8_mask_); //float can't be visualized or used as mask in float format 
  depth_mono8_img_ = depth_mono8_mask_.clone(); 

  /* TODO:
     if (save_bag_file && bagfile_mutex.tryLock() ){
  // todo: make the names dynamic
  bag.write("/camera/rgb/image_mono", ros::Time::now(), visual_img_msg);
  bag.write("/camera/depth/image", ros::Time::now(), depth_img_msg);
  ROS_INFO_STREAM("Wrote to bagfile " << bag.getFileName());
  bagfile_mutex.unlock();
  }*/ 

  if(++data_id_ < ps->get<int>("skip_first_n_frames") 
      || data_id_ % ps->get<int>("data_skip_step") != 0)
  { 
    // If only a subset of frames are used, skip computations but visualize if gui is running
    //     ROS_INFO_THROTTLE(1, "Skipping Frame %i because of data_skip_step setting (this msg is only shown once a sec)", data_id_);
    if(ps->get<bool>("use_gui"))
    {
      //Show the image, even if not using it
      // cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
      // cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
      Q_EMIT newVisualImage(cvMat2QImage(grey_mono8_img_, 0)); //visual_idx=0
      Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_img_,1));//overwrites last cvMat2QImage
      // Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_mask_,1));//overwrites last cvMat2QImage
    }
    return;
  }

  if(pause_ && !getOneFrame_)
  { 
    if(ps->get<bool>("use_gui")){
      Q_EMIT newVisualImage(cvMat2QImage(grey_mono8_img_, 0)); //visual_idx=0
      Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_img_,1));//overwrites last cvMat2QImage
      // Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_mask_,1));//overwrites last cvMat2QImage
    }
    return ;
  }

  // gui logic
  if(getOneFrame_) { //if getOneFrame_ is set, unset it and skip check for  pause
    getOneFrame_ = false;
  } else if(pause_ && !save_bag_file) { //Visualization and nothing else
    return; 
  }

  // secondly, construct a node
  std_msgs::Header depth_header; 
  static unsigned int seq_id_ = 0; 
  depth_header.seq = seq_id_ ++ ;
  depth_header.stamp = ros::Time::now();
  depth_header.frame_id = "sr_depth"; // this should be sent from the publisher 
  // Node* node_ptr = new CNodeWrapper(grey_img_, depth_img_, depth_mono8_img_, depth_header, detector_, extractor_);
  
  Node* node_ptr = 0;
  if(sr_array_ptr->data.size() == distance_size)
  {
    // node_ptr = new CNodeWrapper(grey_mono8_img_, depth_img_, depth_mono8_mask_, depth_header, detector_, extractor_);
    node_ptr = new CPlaneNode(grey_mono8_img_, depth_img_, depth_mono8_mask_, depth_header, detector_, extractor_);
  }else if(sr_array_ptr->data.size() == coordinate_size)
  {
    // node_ptr = new CNodeWrapper(grey_mono8_img_, depth_img_, depth_mono8_mask_, depth_header, buf_x, buf_y, buf_z, detector_, extractor_);
    node_ptr = new CPlaneNode(grey_mono8_img_, depth_img_, depth_mono8_mask_, depth_header, buf_x, buf_y, buf_z, detector_, extractor_);
  }

  // TODO: this function must be carefully considered
  // 
  // In the future, when we use the odometry, imu, ground truth etc. or broadcast the 
  // base2point( relative SE3 from the center of the robot to camera), we will use this function, 
  // now we just use the camera position as the robot center and set the first position as the world reference coordinate
  //
  // retrieveTransformations(depth_header, node_ptr); 
  // 
  
  // for the first node, set the init_base_pose in the first_node() function 
  // if(((CGraphWrapper*)(graph_mgr_))->graph_size() <= 1)
  { 
    // TODO: retrieve IMU roll, pitch data, add it into transform 
    // setInit2Base(node_ptr);
  }

  // callProcessing(grey_img, node_ptr);
  callProcessing(grey_mono8_img_, node_ptr);
  return ;
}

void COpenniWrapper::noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                      const sensor_msgs::ImageConstPtr& depth_img_msg,
                                      const sensor_msgs::CameraInfoConstPtr& cam_info_msg) 
{
  ScopedTimer s(__FUNCTION__);
  ParameterServer* ps = ParameterServer::instance();

  if(++data_id_ < ps->get<int>("skip_first_n_frames") 
     || data_id_ % ps->get<int>("data_skip_step") != 0)
  { 
  // If only a subset of frames are used, skip computations but visualize if gui is running
    ROS_INFO_THROTTLE(1, "Skipping Frame %i because of data_skip_step setting (this msg is only shown once a sec)", data_id_);
    if(ps->get<bool>("use_gui")){//Show the image, even if not using it
      //sensor_msgs::CvBridge bridge;
      cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
      //const cv::Mat& depth_float_img_big = cv_bridge::toCvShare(depth_img_msg)->image;
      cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
      //const cv::Mat& visual_img_big =  cv_bridge::toCvShare(visual_img_msg)->image;
      //cv::Mat visual_img, depth_float_img;
      //cv::resize(visual_img_big, visual_img, cv::Size(), 0.25, 0.25);
      //cv::resize(depth_float_img_big, depth_float_img, cv::Size(), 0.25, 0.25);
      if(visual_img.rows != depth_float_img.rows || 
         visual_img.cols != depth_float_img.cols){
        ROS_ERROR("depth and visual image differ in size! Ignoring Data");
        return;
      }
      depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask
      image_encoding_ = visual_img_msg->encoding;
      Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
      Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_img_,1));//overwrites last cvMat2QImage
    }
    return;
  }
//Convert images to OpenCV format
  //sensor_msgs::CvBridge bridge;
  //cv::Mat depth_float_img = bridge.imgMsgToCv(depth_img_msg);
  //cv::Mat visual_img =  bridge.imgMsgToCv(visual_img_msg);
  cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
  //const cv::Mat& depth_float_img_big = cv_bridge::toCvShare(depth_img_msg)->image;
  cv::Mat visual_img;
  if(image_encoding_ == "bayer_grbg8"){
    cv_bridge::toCvShare(visual_img_msg);
    ROS_INFO("Converting from Bayer to RGB");
    cv::cvtColor(cv_bridge::toCvCopy(visual_img_msg)->image, visual_img, CV_BayerGR2RGB, 3);
  } else{
    ROS_DEBUG_STREAM("Encoding: " << visual_img_msg->encoding);
    visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
  }
  //const cv::Mat& visual_img_big =  cv_bridge::toCvShare(visual_img_msg)->image;
  //cv::Size newsize(320, 240);
  //cv::Mat visual_img(newsize, visual_img_big.type()), depth_float_img(newsize, depth_float_img_big.type());
  //cv::resize(visual_img_big, visual_img, newsize);
  //cv::resize(depth_float_img_big, depth_float_img, newsize);
  if(visual_img.rows != depth_float_img.rows || 
     visual_img.cols != depth_float_img.cols){
     ROS_ERROR("depth and visual image differ in size! Ignoring Data");
     return;
  }
  image_encoding_ = visual_img_msg->encoding;

  depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask

  if(asyncFrameDrop(depth_img_msg->header.stamp, visual_img_msg->header.stamp)) 
    return;

  if (bagfile_mutex.tryLock() && save_bag_file){
     // todo: make the names dynamic
     bag.write("/camera/rgb/image_mono", ros::Time::now(), visual_img_msg);
     bag.write("/camera/depth/image", ros::Time::now(), depth_img_msg);
     ROS_INFO_STREAM("Wrote to bagfile " << bag.getFileName());
     bagfile_mutex.unlock();
  }

  if(ps->get<bool>("use_gui")){
    Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
    Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_img_,1));//overwrites last cvMat2QImage
  }
  if(pause_ && !getOneFrame_) return;

  noCloudCameraCallback(visual_img, depth_float_img, depth_mono8_img_, depth_img_msg->header, cam_info_msg);
}
void COpenniWrapper::noCloudCameraCallback(cv::Mat visual_img, 
                                           cv::Mat depth, 
                                           cv::Mat depth_mono8_img,
                                           std_msgs::Header depth_header,
                                           const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  if(getOneFrame_) { //if getOneFrame_ is set, unset it and skip check for  pause
      getOneFrame_ = false;
  } else if(pause_ && !save_bag_file) { //Visualization and nothing else
    return; 
  }
  ScopedTimer s(__FUNCTION__);
  //######### Main Work: create new node ##############################################################
  Q_EMIT setGUIStatus("Computing Keypoints and Features");
  // ROS_INFO("Yes!!!!!!!!! Succeed to override the callback function!!!!!");
  // static int ncout = 0;
  // mean shift filter depth
  // cv::Mat filtered_depth;
  // cvSmooth(depth, filtered_depth, CV_BLUR, 3, 3);

  // Node* node_ptr = new Node(visual_img, depth, depth_mono8_img, cam_info, depth_header, detector_, extractor_);
  ROS_ERROR("That' it, the OpenNIWrapper works!");
  Node* node_ptr = new CNodeWrapper(visual_img, depth, depth_mono8_img, cam_info, depth_header, detector_, extractor_);

  // ROS_INFO("After Construct %d node!", ncout);
  retrieveTransformations(depth_header, node_ptr);//Retrieve the transform between the lens and the base-link at capturing time;

  callProcessing(visual_img, node_ptr);
}

// only set the pitch angle
bool COpenniWrapper::setInit2Base(Node* node_ptr)
{
  // static tf::StampedTransform init2base_; 
  static bool b_once_ = false; 
  static float roll = 0 ; 
  static float pitch = 0;

  if(!b_once_) // only calculate the initial pose one time, after moving, imu can not give accurate ax, ay, az
  {
    // init2base_ = node_ptr->getGroundTruthTransform(); 
    double droll, dpitch;
    int n = 30;
  
    // imu data already be given 
    // ParameterServer* ps = ParameterServer::instance();
    ParamSrvMi *ps = ParamSrvMi::instanceMi();
    droll = ps->get<double>("imu_roll");
    dpitch = ps->get<double>("imu_pitch");
    roll = droll; 
    pitch = dpitch;

    if( roll >= 0 && pitch >=0)
    {
      // Nothing todo here
      ROS_ERROR("OpenniWrapper.cpp: what, why here? roll= %f, pitch = %f", roll, pitch);
    }
    else if(!p_imu_ || !p_imu_->getRollPitch(roll, pitch, n))
    {
      ROS_ERROR("OpenniWrapper.cpp: fail to connect to IMU!"); 
      b_once_ = true;
      return false;
    }else
    {
      // transform to the navigational frame 
      //        Yb Xb 
      //        | / 
      //        |/
      //        O---> Zb
      static float roll_align = D2R(0.35);
      static float pitch_align = D2R(174.5); 

      roll -= roll_align; 
      pitch -= pitch_align ;

      // if record the new exp data 
      ofstream imu_v("imu_value.log"); 
      imu_v<<roll<<" "<<pitch<<endl;
      imu_v.close();

      ROS_ERROR("OpenniWrapper.cpp: succeedly connect to IMU, get roll = %f, pitch = %f!", roll, pitch); 
      
      // weather to use imu later? 
      p_imu_.reset(); // reset imu 
    }
    // only do it once 
    b_once_ = true;
  }
  
  CNodeWrapper* pn = dynamic_cast<CNodeWrapper*>(node_ptr);
  pn->setPitch(pitch);
}

/*
// set the Init2Base 
bool COpenniWrapper::setInit2Base(Node* node_ptr)
{
  static tf::StampedTransform init2base_; 
  static bool b_once_ = false; 

  if(!b_once_) // only calculate the initial pose one time, after moving, imu can not give accurate ax, ay, az
  {
    init2base_ = node_ptr->getGroundTruthTransform(); 
    double droll, dpitch;
    float roll, pitch;
    int n = 30;
  
    // imu data already be given 
    // ParameterServer* ps = ParameterServer::instance();
    ParamSrvMi *ps = ParamSrvMi::instanceMi();
    droll = ps->get<double>("imu_roll");
    dpitch = ps->get<double>("imu_pitch");
    roll = droll; 
    pitch = dpitch;

    if( roll >= 0 && pitch >=0)
    {
      // Nothing todo here
      ROS_ERROR("OpenniWrapper.cpp: what, why here? roll= %f, pitch = %f", roll, pitch);
    }
    else if(!p_imu_->getRollPitch(roll, pitch, n))
    {
      ROS_ERROR("OpenniWrapper.cpp: fail to connect to IMU!"); 
      b_once_ = true;
      return false;
    }else
    {
      ROS_ERROR("OpenniWrapper.cpp: succeedly connect to IMU, get roll = %f, pitch = %f!", roll, pitch); 
      
      // if record the new exp data 
      ofstream imu_v("imu_value.log"); 
      imu_v<<roll<<" "<<pitch<<endl;
      imu_v.close();
    }

    // transform to the navigational frame 
    //        Yb Xb 
    //        | / 
    //        |/
    //        O---> Zb
    static float roll_align = D2R(0.35);
    static float pitch_align = D2R(174.5); 

    roll -= roll_align; 
    pitch -= pitch_align;

    // construct the Rotation matrix, roll -> Zb, pitch -> Xb
    float cr = cos(roll); float sr = sin(roll); 
    float cp = cos(pitch); float sp = sin(pitch);

    tf::Matrix3x3 R_roll = tf::Matrix3x3::getIdentity(); 
    tf::Matrix3x3 R_pitch = tf::Matrix3x3::getIdentity(); 

    // roll along Zb, in the navigational coordinate reference
    R_roll[0].setX(cr);  R_roll[0].setY(sr); 
    R_roll[1].setX(-sr); R_roll[1].setY(cr);  

    // pitch along Xb, in the navigational coordinate reference 
    R_pitch[1].setY(cp);  R_pitch[1].setZ(sp);
    R_pitch[2].setY(-sp); R_pitch[2].setZ(cp);

    // get the rotation matrix
    tf::Matrix3x3 R = R_pitch*R_roll;

    init2base_.setBasis(R);

    // only do it once 
    b_once_ = true;
  }

  // succeed to retrieve roll and pitch, construct tf::transform
  node_ptr->setGroundTruthTransform(init2base_);
  return true;
}
*/


void COpenniWrapper::callProcessing(cv::Mat visual_img, Node* node_ptr)
{
  OpenNIListener::callProcessing(visual_img, node_ptr);
  /*
    if(!future_.isFinished())
    {
        ROS_INFO("wait for graph adding node!");
        future_.waitForFinished();
    }
    visualization_img_ = visual_img; //No copy
    visualization_depth_mono8_img_ = depth_mono8_img_;//No copy
    if(ParameterServer::instance()->get<bool>("use_gui"))
    {
        //visual_img points to the data received in the callback - which might be deallocated after the callback returns. 
        //This will happen before visualization if processNode is running in background, therefore the data needs to be cloned
        visualization_img_ = visual_img.clone();
        visualization_depth_mono8_img_ = depth_mono8_img_.clone();
    }     
    // future_ = QtConcurrent::run(this, &COpenniWrapper::processNode, node_ptr);      
    future_ = QtConcurrent::run(this, &OpenNIListener::processNode, node_ptr);
    */
}

/*
void COpenniWrapper::processNode(Node* new_node)
{
    Q_EMIT setGUIStatus("Adding Node to Graph");
    bool has_been_added = m_duo_mgr->addNode(new_node);

    if(m_duo_mgr->m_bhas_switched)
    {
        graph_mgr_ = m_duo_mgr->getFrontGraph();
        m_duo_mgr->m_bhas_switched = false;
    }
    //######### Visualization code  #############################################
    if(ParameterServer::instance()->get<bool>("use_gui"))
    {
    	std_msgs::String msg;
    	std::stringstream slamLost;

    	if(has_been_added)
    	{
        */
    		//save pcd and traj of every frames for linyuan
    		/*
    		static int node_id = 0;
    		CGraphWrapper* pg = m_duo_mgr->getFrontGraph();
            g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(pg->getOptimizer()->vertex(new_node->vertex_id_));
            if(!v)
            {
                ROS_ERROR("Nullpointer in graph at position!");
            }
            else
            {
            	node_id++;
            	//save organized pcd and its pose
        		char posename[255];
        		sprintf(posename, "./submaps/pose_%.06d.txt",node_id);
        		ofstream ofs_pose(posename);

        		char pcdname[255];
        		sprintf(pcdname, "./submaps/pcd_%.06d.pcd",node_id);
        		ofstream ofs_pcd(pcdname);

        		QMatrix4x4 qmat = g2o2QMatrix(v->estimateAsSE3Quat());
        		ofs_pose<<qmat(0,0)<<" "<<qmat(0,1)<<" "<<qmat(0,2)<<" "<<qmat(0,3)<<" ";
        		ofs_pose<<qmat(1,0)<<" "<<qmat(1,1)<<" "<<qmat(1,2)<<" "<<qmat(1,3)<<" ";
        		ofs_pose<<qmat(2,0)<<" "<<qmat(2,1)<<" "<<qmat(2,2)<<" "<<qmat(2,3)<<" ";
        		ofs_pose<<qmat(3,0)<<" "<<qmat(3,1)<<" "<<qmat(3,2)<<" "<<qmat(3,3)<<" ";
            	ofs_pose.close();
            	pcl::io::savePCDFile(pcdname, *(new_node->pc_col), false);
            	ROS_ERROR("Saved PCD File %s and %s", posename, pcdname);
            }
            */



    		/*
    		ROS_ERROR("ADDED");
    		  ofstream ofs("featuresForSH.txt");
    		  for(int i=0;i<new_node->feature_locations_2d_.size();i++)
    		  {
    			  ofs<<new_node->feature_locations_2d_[i].pt.x<<" "<<new_node->feature_locations_2d_[i].pt.y<<" ";
    			  ofs<<new_node->feature_locations_3d_[i].x()<<" "<<new_node->feature_locations_3d_[i].y()<<" "<<new_node->feature_locations_3d_[i].z()<<" ";
    			  for(int j=0;j<new_node->feature_descriptors_.cols;j++)
    			  {
    				  ofs<<new_node->feature_descriptors_.at<float>(i,j)<<" ";
    			  }
    			  ofs<<endl;
    		  }
    		  */
/*

    		//publish slamlost info, if working it is "No"
    		slamLost << "No";
    		msg.data = slamLost.str();

    		if(ParameterServer::instance()->get<bool>("visualize_mono_depth_overlay"))
    		{
    			cv::Mat feature_img = cv::Mat::zeros( visualization_img_.rows, visualization_img_.cols, CV_8UC1);
    			graph_mgr_->drawFeatureFlow(feature_img);
    			Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_,visualization_depth_mono8_img_, feature_img, 2)); //show registration
    		} else
    		{
    			graph_mgr_->drawFeatureFlow(visualization_img_, cv::Scalar(0,0,255), cv::Scalar(0,128,0) );
                Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_, 2)); //show registration
            }
        } else 
        {
        	//publish lost info
    		slamLost << "Yes";
    		msg.data = slamLost.str();

            if(ParameterServer::instance()->get<bool>("visualize_mono_depth_overlay")){
                cv::Mat feature_img = cv::Mat( visualization_img_.rows, visualization_img_.cols, CV_8UC1); 
                cv::drawKeypoints(feature_img, new_node->feature_locations_2d_, feature_img, cv::Scalar(155), 5);
                Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_,visualization_depth_mono8_img_, feature_img, 2)); //show registration
            } else 
            {
                cv::drawKeypoints(visualization_img_, new_node->feature_locations_2d_, visualization_img_, cv::Scalar(0, 100,0), 5);
                Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_, 2)); //show registration
            }
        }
    	slamLost_pub.publish(msg);
    }
    if(!has_been_added) delete new_node;
    
    //######### Submap Swap code #############################################
    // static CGraphWrapper* pg = dynamic_cast<CGraphWrapper*>(graph_mgr_);
    // if(pg != 0) pg->submapSwap();
}
*/


