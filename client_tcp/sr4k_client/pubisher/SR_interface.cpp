/*
 * David Z, Apr 14, 2015 
 * 
 * provide interface to access the Swiss Ranger data, 
 * either from : 
 *      1) local file 
 *      2) TCP-IP client 
 *      3) directly connect to the SwissRanger 4000
 * */

#include "SR_interface.h"
#include "ros/ros.h"
#include <cmath>

CSRInterface::CSRInterface()
  :
b_device_ready_(false), 
pSReader_(0)
{
  ros_init();
}

CSRInterface::~CSRInterface()
{
  close();
}

void CSRInterface::ros_init()
{
  // 
  ros::NodeHandle nh_p("~"); 
  nh_p.param("sr_new_file_version", b_new_file_version, true);
  nh_p.param("sr_source", sr_src_, string("SR_CAM")); 
  nh_p.param("sr_cam_ip", sr_cam_ip_, string("192.168.1.42"));
  nh_p.param("sr_cam_it", sr_cam_it_time_, 30);

  ROS_INFO("sr_source %s, sr_cam_ip: %s, sr_file_version: %s", sr_src_.c_str(), sr_cam_ip_.c_str(), b_new_file_version?"new":"old");

  /*if(!nh_p.getParam("sr_source", sr_src_))
  { 
    ROS_WARN("sr_src_ cannot be found!");
    sr_src_ = "SR_FILE";
  }*/

  // sqrt mapping 
  int N = 65536; 
  sqrt_map_.resize(N, 0); 
  for(int i=0; i<N; i++)
  {
    sqrt_map_[i] = (unsigned char)(sqrt(double(i)));
  }

  // SR intensity buffer initialization 
  pRaw_img_.resize(SR_SIZE, 0); 
  pGrey_img_.resize(SR_SIZE, 0);

  // SR distance buffer initialization 
  pDis_.resize(SR_SIZE, 0);

  // SR publisher buffer initialization
  if(b_new_file_version)
  {
    // pPub_.resize(sizeof(SRCAM)+SR_SIZE*(sizeof(unsigned char) + sizeof(unsigned short)), 0);
    pPub_.resize(SR_SIZE*(sizeof(unsigned char) + sizeof(unsigned short)), 0);
  }else
  {
    //old version 
    pPub_.resize(SR_SIZE*(sizeof(unsigned char) + sizeof(float)*3), 0); 
  }
}

bool CSRInterface::open()
{
    if(sr_src_ == "SR_CAM")
    {
      b_device_ready_ = openCam(sr_cam_);
    }else if(sr_src_ == "SR_FILE")
    {
      pSReader_ = new CSReader();
      b_device_ready_ = pSReader_->loadAllData();
    }else if(sr_src_ == "SR_TCP")
    {
      // TODO: SR_TCP 
      pSRTcp_ = new CSRTcpSocket; 
      b_device_ready_ = pSRTcp_->open();
    }
    return b_device_ready_;
}

void CSRInterface::close()
{
  if(sr_src_ == "SR_CAM")
  {
    SR_Close(sr_cam_);
  }else if(sr_src_ == "SR_FILE")
  {
    if(pSReader_ != 0) 
    {
      delete pSReader_;
      pSReader_ = 0;
    }
  }else if(sr_src_ == "SR_TCP")
  {
      if(pSRTcp_ !=0)
      {
        pSRTcp_->close(); 
        delete pSRTcp_; 
        pSRTcp_ = 0;
      }
  }
}

bool CSRInterface::get(std_msgs::UInt8MultiArray& sr_array)
{
    if(!b_device_ready_)
    {
      ROS_ERROR("SR_interface.cpp: device not ready!");
      return false;
    }
    
    // allocate memory 
    if(sr_array.data.size() != pPub_.size())
    {
      sr_array.data.resize(pPub_.size());
    }

    if(sr_src_ == "SR_CAM")
    {
      return getFromCam(sr_array);
    }else if(sr_src_ == "SR_FILE")
    {
      return getFromFile(sr_array);  
    }else if(sr_src_ == "SR_TCP")
    {
       // TODO: SR_FILE SR_TCP 
       return getFromTcp(sr_array);
    }
    return true;
}

bool CSRInterface::getFromTcp(std_msgs::UInt8MultiArray& sr_array)
{
  // TCP_data data; 
  if(!pSRTcp_->get(tcp_data))
  {
    cout<<"SR_interface.cpp: failed to get data from tcp socket!"<<endl;
    return false;
  }
  
  char* pbuf = tcp_data.data.data();
  
  // copy from camera to buffer 
  memcpy(&pRaw_img_[0], pbuf, SR_SIZE*sizeof(unsigned short)); 

  // convert it from unsigned short to unsigned char 
  map_raw_img_to_grey(&pRaw_img_[0], &pGrey_img_[0], SR_SIZE);

  // distance image 
  memcpy(&pDis_[0], pbuf + SR_SIZE*sizeof(unsigned short), SR_SIZE*sizeof(unsigned short));
  // memcpy(&pDis_[0], &d.intensity_[0], SR_SIZE*sizeof(unsigned short));

  // dump them into the publish buffer 
  unsigned char* pP = &pPub_[0]; 
  // first cam handler 
  // int cam_offset = sizeof(SRCAM); 
  // memcpy(pP, &d.cam_handle_[0], cam_offset); 
  // second distance 
  memcpy(pP , &pDis_[0], SR_SIZE*sizeof(unsigned short));
  // then intensity
  memcpy((pP + SR_SIZE*sizeof(unsigned short)), &pGrey_img_[0], SR_SIZE*sizeof(unsigned char)); 

  // copy from pub to msg 
  // unsigned int total_all = cam_offset + SR_SIZE*(sizeof(unsigned short) + sizeof(unsigned char));
  unsigned int total_all = SR_SIZE*(sizeof(unsigned short) + sizeof(unsigned char));
  if(sr_array.data.size() != total_all) 
    sr_array.data.resize(total_all);
  unsigned char* pDst = sr_array.data.data(); 
  memcpy(pDst, pP, total_all);

  return true; 
}

bool CSRInterface::getFromFile(std_msgs::UInt8MultiArray& sr_array)
{
  bool b_file_done = false;
  sr_data d = pSReader_->get_current_frame(b_file_done); 
  if(b_file_done)
  {
    return false;
  }
  // copy sr_data to sr_array 
  if(b_new_file_version)
  {
    // copy from camera to buffer 
    memcpy(&pRaw_img_[0], &d.intensity_[0], SR_SIZE*sizeof(unsigned short)); 

    // convert it from unsigned short to unsigned char 
    map_raw_img_to_grey(&pRaw_img_[0], &pGrey_img_[0], SR_SIZE);

    // distance image 
    memcpy(&pDis_[0], &d.dis_[0], SR_SIZE*sizeof(unsigned short));
    // memcpy(&pDis_[0], &d.intensity_[0], SR_SIZE*sizeof(unsigned short));

    // dump them into the publish buffer 
    unsigned char* pP = &pPub_[0]; 
    // first cam handler 
    // int cam_offset = sizeof(SRCAM); 
    // memcpy(pP, &d.cam_handle_[0], cam_offset); 
    // second distance 
    memcpy(pP , &pDis_[0], SR_SIZE*sizeof(unsigned short));
    // then intensity
    memcpy((pP + SR_SIZE*sizeof(unsigned short)), &pGrey_img_[0], SR_SIZE*sizeof(unsigned char)); 

    // copy from pub to msg 
    // unsigned int total_all = cam_offset + SR_SIZE*(sizeof(unsigned short) + sizeof(unsigned char));
    unsigned int total_all = SR_SIZE*(sizeof(unsigned short) + sizeof(unsigned char));
    if(sr_array.data.size() != total_all) 
      sr_array.data.resize(total_all);
    unsigned char* pDst = sr_array.data.data(); 
    memcpy(pDst, pP, total_all);
  }else
  {
    // old file version
    // copy from camera to buffer 
    memcpy(&pRaw_img_[0], &d.intensity_[0], SR_SIZE*sizeof(unsigned short)); 

    // convert it from unsigned short to unsigned char 
    map_raw_img_to_grey(&pRaw_img_[0], &pGrey_img_[0], SR_SIZE);

    // dump them into the publish buffer 
    unsigned char* pP = &pPub_[0]; 
    
    // 1, intensity img, 
    memcpy(pP, pGrey_img_.data(), SR_SIZE*sizeof(unsigned char)); 
   
    // 2, x, y, z 
    memcpy(pP + SR_SIZE*sizeof(unsigned char), &d.x_[0], SR_SIZE*sizeof(float)); 
    memcpy(pP + SR_SIZE*(sizeof(unsigned char) + sizeof(float)),   &d.y_[0], SR_SIZE*sizeof(float)); 
    memcpy(pP + SR_SIZE*(sizeof(unsigned char) + 2*sizeof(float)), &d.z_[0], SR_SIZE*sizeof(float)); 
    
    // finally, copy it to the msg buf
    unsigned int total_all = SR_SIZE*(sizeof(float)*3 + sizeof(unsigned char));
    if(sr_array.data.size() != total_all) 
      sr_array.data.resize(total_all);
    unsigned char* pDst = sr_array.data.data(); 
    memcpy(pDst, pP, total_all);
  }
  return true;
}

bool CSRInterface::getFromCam(std_msgs::UInt8MultiArray& sr_array)
{
  SR_Acquire(sr_cam_);

  // intensity image 
  ImgEntry* imgEntryArray;
  int nImg = SR_GetImageList(sr_cam_, &imgEntryArray);
  WORD* p = (WORD*)imgEntryArray[1].data;
    
  // copy from camera to buffer 
  memcpy(&pRaw_img_[0], p, SR_SIZE*sizeof(unsigned short)); 
  
  // convert it from unsigned short to unsigned char 
  map_raw_img_to_grey(&pRaw_img_[0], &pGrey_img_[0], SR_SIZE);

  // distance image 
  p = (WORD*)imgEntryArray[0].data;
  memcpy(&pDis_[0], p, SR_SIZE*sizeof(unsigned short));

  // dump them into the publish buffer 
  unsigned char* pP = &pPub_[0]; 
  // int sr_cam_offset = sizeof(SRCAM); 
  // first camera handle, camera handle way not work, because SRCAM is a pointer to the class instance
  // memcpy(pP, &sr_cam_, sr_cam_offset);

  // then distance
  memcpy(pP , &pDis_[0], SR_SIZE*sizeof(unsigned short));
  // finally intensity
  memcpy((pP + SR_SIZE*sizeof(unsigned short)), &pGrey_img_[0], SR_SIZE*sizeof(unsigned char)); 
 
  // check out the size is right 
  assert(pPub_.size() == SR_SIZE*(sizeof(unsigned short)+sizeof(unsigned char)));

  // copy data from local to msg 
  unsigned char* pDst = sr_array.data.data(); 
  memcpy(pDst, pP, pPub_.size());
  return true;
}

bool CSRInterface::openCam(SRCAM& srCam)
{
  const char * ip_addr = sr_cam_ip_.c_str(); // parameterized 
  int res = SR_OpenETH(&srCam, ip_addr); // open camera 
  if(res <=0)
  {
    ROS_ERROR("SR_interface.cpp: failed to open camera at IP: %s", ip_addr);
    return false;
  }
  SR_SetIntegrationTime(srCam, sr_cam_it_time_); 
  SR_SetMode(srCam, AM_SW_TRIGGER | AM_CONF_MAP | AM_MEDIAN | AM_COR_FIX_PTRN | AM_SW_ANF | AM_CONV_GRAY); 
  return true;
}

void CSRInterface::map_raw_img_to_grey(unsigned short * pRaw, unsigned char* pGrey, int N)
{
  unsigned short limit_s = 65000;
  unsigned short* p1 = pRaw; 
  unsigned char* p2 = pGrey; 
  
  unsigned char max_c = 0; 
  unsigned char tc = 0; 

  for(int i=0; i<N; i++)
  {
    if(*p1 >= limit_s) // delete the values larger than 65000
      tc = 0; 
    else 
      tc = sqrt_map_[*p1];
    if(tc > max_c) {max_c = tc; }
    *p2 = tc;
    ++p1; 
    ++p2; 
  }
  assert(max_c > 0);
  p2 = pGrey;
  float inv_max = (float)(255.0/max_c);
  for(int i=0; i<N; i++)
  {
    *p2 = (unsigned char)((*p2)*inv_max);
    ++p2;
  }
}

