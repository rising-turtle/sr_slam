/*
 * David Z, Apr 20, 2015 
 * wrapper for node in rgbdslam, to handle swiss ranger interface 
 * and extend the content in a node
 *
 * */

#include "NodeWrapper.h"
#include "global_def.h"
#include "pcl/registration/transformation_estimation.h"
#include <cmath>
// #include "scoped_timer.h"
#include <Eigen/Geometry>
#include <pcl/common/transformation_from_correspondences.h>
#include <fstream>
#include <sstream>
#include "misc.h"
#include <pcl/filters/voxel_grid.h>
#include <opencv/highgui.h>
#include "pcl/registration/icp.h"
#include <QMutex>
#include <vector>
#include "libMesaSR.h"
#include "cam_model.h"

#ifdef USE_SIFT_GPU
#include "sift_gpu_wrapper.h"
#endif

QMutex Node::gicp_mutex;
QMutex Node::siftgpu_mutex;

using namespace std;
namespace {
  inline int round(float d)
  {
    return static_cast<int>(floor(d+0.5));
  }

  
  ///Compute the RootSIFT from SIFT according to Arandjelovic and Zisserman
  /*void squareroot_descriptor_space(cv::Mat& descriptors)
  {
    // Compute sums for L1 Norm
    cv::Mat sums_vec;
    descriptors = cv::abs(descriptors); //otherwise we draw sqrt of negative vals
    cv::reduce(descriptors, sums_vec, 1 , CV_REDUCE_SUM, CV_32FC1);
    for(unsigned int row = 0; row < descriptors.rows; row++){
      if(sums_vec.at<float>(row) == 0) continue; //Do not normalize norm-zero vectors
      int offset = row*descriptors.cols;
      for(unsigned int col = 0; col < descriptors.cols; col++){
        descriptors.at<float>(offset + col) = 
          sqrt(descriptors.at<float>(offset + col) / sums_vec.at<float>(row) );
      }
    }
  }*/

  typedef union
  {
    struct /*anonymous*/
    {
      unsigned char Blue;
      unsigned char Green;
      unsigned char Red;
      unsigned char Alpha;
    };
    float float_value;
    long long_value;
  } RGBValue;

  pointcloud_type* createXYZRGBPointCloud(const cv::Mat& rgb, const cv::Mat& dpt, const CamModel& C)
  {
    int pixel_data_size = 3;
    char red_idx = 0, green_idx = 1, blue_idx = 2;
    if(rgb.type() == CV_8UC1) // mono image 
    {
      pixel_data_size = 1; 
    }
    // bgr2?
    else if(rgb.type() == CV_8UC3)
    {
      red_idx = 2;  blue_idx = 1; 
    }
    pointcloud_type* cloud (new pointcloud_type() );
    cloud->width = rgb.cols; 
    cloud->height = rgb.rows; 
    cloud->is_dense = false; 
    cloud->resize(cloud->width * cloud->height); 
    unsigned int color_row_step, color_pix_step, depth_pix_step, depth_row_step;
    color_pix_step = pixel_data_size * (rgb.cols / cloud->width);
    color_row_step = pixel_data_size * (rgb.rows / cloud->height -1 ) * rgb.cols;
    depth_pix_step = (dpt.cols / cloud->width);
    depth_row_step = (dpt.rows / cloud->height -1 ) * dpt.cols;

    cloud->points.resize (cloud->height * cloud->width);
    // const uint8_t* rgb_buffer = &rgb.data[0];
    int color_idx = 0, depth_idx = 0;
    double depth_scaling = 0.001; // mm to m ParameterServer::instance()->get<double>("depth_scaling_factor");
    float max_depth = -1; //ParameterServer::instance()->get<double>("maximum_depth");
    if(max_depth < 0.0) max_depth = std::numeric_limits<float>::infinity();
    // const unsigned short * depth_buffer = (const unsigned short *)(&dpt.data[0]); 
    pointcloud_type::iterator pt_iter = cloud->begin ();
    int data_skip_step = 1;
    double dx, dy, dz; 
    ROS_INFO("createXYZRGBPointCloud rgb.rows = %d rgb.cols = %d, cloud.size = %d", rgb.rows, rgb.cols, cloud->points.size());
    for (int v = 0; v < (int)rgb.rows; v += data_skip_step, color_idx += color_row_step, depth_idx += depth_row_step)
    {
      for (int u = 0; u < (int)rgb.cols; u += data_skip_step, color_idx += color_pix_step, depth_idx += depth_pix_step, ++pt_iter)
      {
        point_type& pt = *pt_iter;
        // float Z = float(depth_buffer[depth_idx]) * depth_scaling;
        float Z = float(dpt.at<unsigned short>(depth_idx))*depth_scaling;
        // float Z = dpt.at<float>(depth_idx);
        if (!(Z <= max_depth)) //Should also be trigger on NaN//std::isnan (Z))
        {
          // pt.x = (u - cx) * 1.0 * fx; //FIXME: better solution as to act as at 1meter?
          // pt.y = (v - cy) * 1.0 * fy;
          pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
        }
        else // Fill in XYZ
        {
          // pt.x = (u - cx) * Z * fx;
          // pt.y = (v - cy) * Z * fy;
          // pt.z = Z;
          C.convertUVZ2XYZ(u, v, Z, dx, dy, dz); 
          pt.x = dx; pt.y = dy; pt.z = dz; 
        }
	// ROS_INFO("v = %d u = %d pt = %f %f %f", v, u, pt.x, pt.y, pt.z);
        // Fill in color
        RGBValue color;
        if(color_idx > 0 && color_idx < rgb.total()*color_pix_step){ //Only necessary because of the color_idx offset 
          if(pixel_data_size == 3){
            color.Red   = rgb.at<uint8_t>(color_idx + red_idx);
            color.Green = rgb.at<uint8_t>(color_idx + green_idx);
            color.Blue  = rgb.at<uint8_t>(color_idx + blue_idx);
          } else {
            color.Red   = color.Green = color.Blue  = rgb.at<uint8_t>(color_idx);
          }
          color.Alpha = 0;
#ifndef RGB_IS_4TH_DIM
          pt.rgb = color.float_value;
#else
          pt.data[3] = color.float_value;
#endif
        }
    }
  }
    return cloud;
  }

}


CNodeWrapper::CNodeWrapper(const cv::Mat& visual, 
           const cv::Mat& depth,
           const cv::Mat& detection_mask,
           const sensor_msgs::CameraInfoConstPtr& cam_info, 
           std_msgs::Header depth_header,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor)
    : Node(visual, depth, detection_mask, cam_info, depth_header, detector, extractor),
    gt_has_been_set_(false), mpCamModel(NULL)
{
    // Init();
}

CNodeWrapper::CNodeWrapper(const cv::Mat visual,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor,
           pointcloud_type::Ptr point_cloud,
           const cv::Mat detection_mask)
    : Node(visual, detector, extractor, point_cloud, detection_mask),
    gt_has_been_set_(false), mpCamModel(NULL)
{
    // Init();
}


// CNodeWrapper::CNodeWrapper(unsigned char* pStream)
// {
//  init();
// }

CNodeWrapper::CNodeWrapper(const cv::Mat& visual, 
           const cv::Mat& depth,
           const cv::Mat& detection_mask,
           const CamModel& cam_model,
           std_msgs::Header depth_header,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor) 
{
    Init(depth_header);
    ParameterServer* ps = ParameterServer::instance();
  
    if(mpCamModel == NULL)
    {
      mpCamModel = new CamModel;
      *mpCamModel = cam_model;
    }
    d_rows_ = cam_model.height; 
    d_cols_ = cam_model.width;
    d_N_ = d_rows_*d_cols_;

    pc_col = pointcloud_type::Ptr(createXYZRGBPointCloud(visual, depth, cam_model));
	ROS_INFO("pc_col has %d points", pc_col->points.size());
    sr_cloudPtr tmp_cloud(new sr_cloud_type);  
    tmp_cloud->resize(d_N_); 
    tmp_cloud->width = d_cols_; 
    tmp_cloud->height = d_rows_; 
    for(int i = 0; i<d_N_; i++)
    {
      sr_point_type& pt = tmp_cloud->points[i]; 
      point_type& pct = pc_col->points[i]; 
      pt.x = pct.x; pt.y = pct.y; pt.z = pct.z; 
    }
    featureInit(visual, depth, detection_mask, depth_header, tmp_cloud, detector, extractor);
}

bool CNodeWrapper::regainPointCloud()
{
  if(mpCamModel == NULL) return false; 

  // 1. read images 
  cv::Mat rgb = cv::imread(m_frgb.c_str(), -1); 
  cv::Mat dpt = cv::imread(m_fdpt.c_str(), -1); 
  if(!rgb.data || !dpt.data)
    return false; 
  
  // 2. 
  pc_col = pointcloud_type::Ptr(createXYZRGBPointCloud(rgb, dpt, *mpCamModel ));
  return true;
}

CNodeWrapper::CNodeWrapper(const cv::Mat& visual, 
           const cv::Mat& depth,
           const cv::Mat& detection_mask,
           std_msgs::Header depth_header,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor)

 /* : 
  id_(-1), seq_id_(-1), vertex_id_(-1), valid_tf_estimate_(true), matchable_(true),
  pc_col(new pointcloud_type()),
  flannIndex(NULL),
  header_(depth_header),
  base2points_(tf::Transform::getIdentity(), depth_header.stamp, ParameterServer::instance()->get<std::string>("base_frame_name"), depth_header.frame_id),
  ground_truth_transform_(tf::Transform::getIdentity(), depth_header.stamp, ParameterServer::instance()->get<std::string>("ground_truth_frame_name"), ParameterServer::instance()->get<std::string>("base_frame_name")),
  odom_transform_(tf::Transform::getIdentity(), depth_header.stamp, "missing_odometry", depth_header.frame_id),
  initial_node_matches_(0)*/
{
    // ScopedTimer s("Node Constructor");
    Init(depth_header);
    ParameterServer* ps = ParameterServer::instance();
    pc_col.reset(new pointcloud_type());

    // now only the sr4k has the camera model 
    d_N_ = sr_n ; 
    d_rows_ = sr_rows; 
    d_cols_ = sr_cols;

    // compute time elapse 
    double begin_time_ms, end_time_ms; 
    begin_time_ms = ros::Time::now().toSec()*1000.; 

    // compute the corresponding 3D point cloud 
    sr_cloudPtr tmp_cloud(new sr_cloud_type);  
    sr_calSRCloud(tmp_cloud, depth); // actually, here depth is radial distance from pixel(P') to object(P), not only the Z 
    // assert(tmp_cloud->points.size() == sr_n);
    assert(tmp_cloud->points.size() == d_N_);

    // initialize the features and point cloud 
    featureInit(visual, depth, detection_mask, depth_header, tmp_cloud, detector, extractor);

    end_time_ms = ros::Time::now().toSec()*1000.;
}

CNodeWrapper::CNodeWrapper(const cv::Mat& visual, 
           const cv::Mat& depth,
           const cv::Mat& detection_mask,
           std_msgs::Header depth_header,
           std::vector<float>& px, std::vector<float>& py, 
           std::vector<float>& pz,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor)
{
     // ScopedTimer s("Node Constructor");
    Init(depth_header);

    // compute time elapse 
    // double begin_time_ms, end_time_ms; 
    // begin_time_ms = ros::Time::now().toSec()*1000.; 
    
    // decide which data it is 
    if(pz.size() == sr_n)
    {
      d_N_ = sr_n; 
      d_rows_ = sr_rows; 
      d_cols_ = sr_cols; 
    }else if(pz.size() == rs_n)
    {
      d_N_ = rs_n; 
      d_rows_ = rs_rows; 
      d_cols_ = rs_cols;
    }else
    {
      ROS_ERROR("NodeWrapper.cpp: error, pz.size() = %u, sr_n= %d, rs_n = %d, d_N_ cannot be decided!", pz.size(), sr_n, rs_n);
      return ;
    }

    // compute the corresponding 3D point cloud 
    sr_cloudPtr tmp_cloud(new sr_cloud_type);  
    pc_col.reset(new pointcloud_type());
    // sr_calSRCloud(tmp_cloud, depth); // actually, here depth is radial distance from pixel(P') to object(P), not only the Z 
    tmp_cloud->points.resize(d_N_); //  tmp_cloud->points.resize(sr_n);
    tmp_cloud->width =  d_cols_; // sr_cols; 
    tmp_cloud->height = d_rows_; // sr_rows;

     // sr_cloudPtr dbg_cloud(new sr_cloud_type); // for debug 
     // dbg_cloud->points.resize(d_N_); 
     // sr_calSRCloud(dbg_cloud, depth);
     // ofstream tlog("comp_xy.log");
  
    // ROS_INFO("%s at %d before tmp point cloud assignment", __FILE__, __LINE__);

    for(int i=0; i<d_N_; i++)
    {
      sr_point_type& pt = tmp_cloud->points[i]; 
      // pt.x = px[i]; pt.y = py[i]; pt.z = pz[i];
      // pt.x = -ox;  pt.y = -oy; pt.z = oz - 0.01; // 1cm shift 
      // Swap XYZ order, so:
      // SwissRange -> Smart Cane
      //  Z            X
      //  X            Y
      //  Y            Z
      // pt.x = pz[i];
      // pt.y = px[i];
      // pt.z = py[i];
      // SwissRange -> standard camera coordinate
      if(d_N_ == sr_n)  // sr4k
      {
        pt.x = -px[i]; 
        pt.y = -py[i];
        pt.z = pz[i] + 0.01;
        // sr_point_type& dpt = dbg_cloud->points[i];
        // tlog<<dpt.z<<" "<<pt.z<<" "<<dpt.x<<" "<<pt.x<<" "<<dpt.y<<" "<<pt.y<<endl;
      }else if(d_N_ == rs_n) // realsense
      {
        pt.x = px[i]; 
        pt.y = py[i];
        if(pz[i] <= 0.0)
          pt.z = NAN; 
          // pt.z = pz[i];
        else
          pt.z = pz[i];
      }
      // pt.intensity = visual.at<unsigned char>(i)/255.;
    }
    // assert(tmp_cloud->points.size() == sr_n);
    assert(tmp_cloud->points.size() == d_N_);

    // ROS_INFO("before feature Init"); 
    // initialize the features and point cloud 
    featureInit(visual, depth, detection_mask, depth_header, tmp_cloud, detector, extractor);

    // end_time_ms = ros::Time::now().toSec()*1000.;
}
CNodeWrapper::CNodeWrapper()
{
  std_msgs::Header h; 
  Init(h);
}

CNodeWrapper::CNodeWrapper(string path, int id)
{
  std_msgs::Header h; 
  Init(h);

  if(!read(path, id))
  {
    ROS_ERROR("NodeWrapper.cpp: failed to read node from disk!");
  }
}

CNodeWrapper::~CNodeWrapper()
{
  // ROS_WARN("NodeWrapper.cpp: in ~CNodeWrapper()!");
#ifdef USE_ICP_CODE
    if(gicp_point_set_ != NULL)
    {
        delete gicp_point_set_;
        gicp_point_set_ = NULL;
    }
#endif
    if(mpCamModel != NULL)
    {
      delete mpCamModel; 
      mpCamModel = NULL;
    }
  // ROS_WARN("NodeWrapper.cpp: after ~CNodeWrapper()!");
}

void CNodeWrapper::storeSRPointCloud(sr_cloudPtr& tmpSR, const cv::Mat& intensity)
{
    // a bug here, pc_col could be NULL 
    // if(pc_col->size() == tmpSR->size())
    if(pc_col != NULL && pc_col->size() == tmpSR->size()) 
    {
      // already been set 
      return; 
    }
    // pc_col.reset(new pointcloud_type());
    //   pc_col = pointcloud_type::Ptr(new pointcloud_type());
    // pc_col->points.resize(sr_n); 
    pc_col->points.resize(d_N_); 
    int sr_n = tmpSR->points.size(); 
    pc_col->width = tmpSR->width; 
    pc_col->height = tmpSR->height;
    
    for(int k=0; k<d_N_; k++)
    {
      point_type& pt = pc_col->points[k]; 
      sr_point_type& pt2 = tmpSR->points[k]; 
      pt.x = pt2.x; pt.y = pt2.y; pt.z = pt2.z; 
      // TODO: add some color for the points
      unsigned char grey = intensity.at<unsigned char>(k);
      pt.r = grey; pt.g = grey; pt.b = grey;
    }
}

void CNodeWrapper::featureInit(const cv::Mat& visual, 
           const cv::Mat& depth,
           const cv::Mat& detection_mask,
           std_msgs::Header depth_header,
           sr_cloudPtr& tmp_cloud,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor)
{
    ParameterServer* ps = ParameterServer::instance();
    static int save_first_pc = 0;
   
    //Create point cloud inf necessary
    if(ps->get<bool>("store_pointclouds") || 
        ps->get<int>("emm__skip_step") > 0 ||
        ps->get<bool>("use_icp") ||
        (ps->get<bool>("use_glwidget") && ps->get<bool>("use_gui") && ! ps->get<bool>("glwidget_without_clouds")))
    {
      // pc_col = pointcloud_type::Ptr(createXYZRGBPointCloud(depth, visual, cam_info));
      // pc_col = pointcloud_type::Ptr(createSRPointCloud(depth, visual));
      
      /*
      pc_col = pointcloud_type::Ptr(new pointcloud_type());
      pc_col->points.resize(sr_n); 
      for(int k=0; k<sr_n; k++)
      {
        point_type& pt = pc_col->points[k]; 
        sr_point_type& pt2 = tmp_cloud->points[k]; 
        pt.x = pt2.x; pt.y = pt2.y; pt.z = pt2.z; 
        // TODO: add some color for the points
      }*/

      // ROS_INFO("before storeSRPointCloud");
      storeSRPointCloud(tmp_cloud, visual);
    }else // empty pc 
    {
        // ROS_INFO("pc_col = empty!");
        if(save_first_pc++ > 0)
	        pc_col = pointcloud_type::Ptr(new pointcloud_type());
    }
    pc_col->header = header_;
    
    // actually the SR intensity image should be gray_img
    cv::Mat gray_img; 
    if(visual.type() == CV_8UC3){
      cvtColor(visual, gray_img, CV_RGB2GRAY);
    } else 
    {
      gray_img = visual;
    }

    // try histogram equalization 
    cv::Mat img; 
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(gray_img, img);
    gray_img = img;

#ifdef USE_SIFT_GPU
    std::vector<float> descriptors;
    if(ps->get<std::string>("feature_detector_type") == "SIFTGPU"){
      // ScopedTimer s("Feature Detection and Descriptor Extraction");
      SiftGPUWrapper* siftgpu = SiftGPUWrapper::getInstance();
      siftgpu->detect(gray_img, feature_locations_2d_, descriptors);
      ROS_WARN_COND(descriptors.size()==0, "No keypoints for current image!");
    } else 
#endif
    {
      // ScopedTimer s("Feature Detection");
      ROS_FATAL_COND(detector.empty(), "No valid detector!");
      detector->detect( gray_img, feature_locations_2d_, detection_mask);// fill 2d locations
      // ROS_ERROR("NodeWrapper.cpp: detected %u 2D features", feature_locations_2d_.size());
    }

    // project pixels to 3dPositions and create search structures for the gicp
#ifdef USE_SIFT_GPU
    if(ps->get<std::string>("feature_extractor_type") == "SIFTGPU"){

      if(ps->get<std::string>("feature_detector_type") != "SIFTGPU"){
        //not already extracted descriptors in detection step
        //clean keypoints from those without 3d FIXME: can be made more performant
        
        // projectTo3D(feature_locations_2d_, feature_locations_3d_, depth, cam_info);
        sr_projectTo3D(feature_locations_2d_, feature_locations_3d_, tmp_cloud);
        SiftGPUWrapper* siftgpu = SiftGPUWrapper::getInstance();
        siftgpu->detect(gray_img, feature_locations_2d_, descriptors);
      }
      if(descriptors.size() > 0){
        // projectTo3DSiftGPU(feature_locations_2d_, feature_locations_3d_, depth, cam_info, descriptors, feature_descriptors_); 
        sr_projectTo3DSiftGPU(feature_locations_2d_, feature_locations_3d_, tmp_cloud, descriptors, feature_descriptors_);
        ROS_INFO("Siftgpu Feature Descriptors size: %d x %d", feature_descriptors_.rows, feature_descriptors_.cols);
      } else {
        ROS_WARN("No descriptors for current image!");
      }
    }
    else
#endif
    {
        sr_projectTo3D(feature_locations_2d_, feature_locations_3d_, tmp_cloud); //takes less than 0.01 sec
        // projectTo3d need a dense cloud to use the points.at(px.x,px.y)-Call
        // ScopedTimer s("Feature Extraction");
        extractor->compute(gray_img, feature_locations_2d_, feature_descriptors_); //fill feature_descriptors_ with information 
    }
    assert(feature_locations_2d_.size() == feature_locations_3d_.size());
    assert(feature_locations_3d_.size() == (unsigned int)feature_descriptors_.rows); 
    feature_matching_stats_.resize(feature_locations_2d_.size(), 0);
    ROS_INFO_NAMED("statistics", "Feature Count of Node:\t%d", (int)feature_locations_2d_.size());

#ifdef USE_ICP_CODE
    gicp_initialized = false;
    gicp_point_set_ = NULL;
    if(ps->get<int>("emm__skip_step") <= 0 && !ps->get<bool>("store_pointclouds") && ps->get<bool>("use_icp")) 
    {//if clearing out point clouds, the icp structure needs to be built before
      gicp_mutex.lock();
      gicp_point_set_ = this->getGICPStructure();
      gicp_mutex.unlock();
    }
#endif
    if(ps->get<bool>("use_root_sift") &&
        (ps->get<std::string>("feature_extractor_type") == "SIFTGPU" ||
         ps->get<std::string>("feature_extractor_type") == "SURF" ||
         ps->get<std::string>("feature_extractor_type") == "GFTT" ||
         ps->get<std::string>("feature_extractor_type") == "SIFT")){
      squareroot_descriptor_space(feature_descriptors_);
    }

}

bool CNodeWrapper::sr_calSRCloud(sr_cloudPtr& pc, const cv::Mat& d )
{
  // static CamModel cam_model(223.9758, 226.7442, 89.361, 75.8112); 
  static CamModel cam_model(250.5773, 250.5773, 90, 70, -0.8466, 0.5370);
  if(d.type() != CV_16UC1) 
  {
    ROS_ERROR("NodeWrapper.cpp: distance mat is not CV_16UC1!");
    return false; 
  }

  if(pc->points.size() != d_N_)
  {
    pc->points.resize(d_N_); 
  }

  pc->width = d_cols_; // sr_cols; 
  pc->height = d_rows_; // sr_rows;

  unsigned short* pD = (unsigned short*)d.data; 
  int k;
  float z;  
  double ox,oy,oz;
  for(int i=0; i < d_rows_; i++)
  {
    for(int j=0; j <d_cols_; j++)
    {
      k = j + i*d_cols_;
      z = *pD*0.001; // convert mm to m
      // cam_model.convertUVZ2XYZ(j+0.5, i+0.5, z, ox, oy, oz); 
      if(z <= 0)
      {
        ox = oy = oz = 0; 
      }else
      {
        z += 0.015; 
        cam_model.convertUVZ2XYZ(j, i, z, ox, oy, oz); 
      }
      sr_point_type & pt = pc->points[k];

      pt.x = ox;  pt.y = oy; pt.z = oz; // 1cm shift 
      
      // do not do this, just transform the coordinate reference into global, from camera to global 
      //            z                            z   x
      //           /                             |  /
      //          /                              | /
      //         /----- x                 y ---- |/
      //         |                             Global     
      //         |                                     
      //         | y
      //       Camera 
      //
      // Swap XYZ order, so:
      // SwissRange -> Smart Cane
      //  Z            X
      //  X            Y
      //  Y            Z
      // pt.x = oz - 0.01; pt.y = -ox; pt.z = -oy;

      ++pD; 
    }
  }
}

/*****
 *  The SRCAM handler does not work, because SRCAM is a pointer to an enclosed class 
// construct a point cloud using SR distance info
bool CNodeWrapper::sr_calSRCloud(sr_cloudPtr& pc, const cv::Mat& d)
{
    static SRCAM cam; 
    unsigned short * values = (unsigned short*)SR_GetImage(cam, 0); // distance entrance 

    // copy d to values 
    cv::Mat tmp = d.clone();
    unsigned char * pSrc = (unsigned char*)tmp.data;
    unsigned char * pDst = (unsigned char*)values;
    memcpy(pDst, pSrc, sr_n*sizeof(unsigned short));

    // index mapping 
    static vector<unsigned char> ix(sr_n, 0); 
    static vector<unsigned char> iy(sr_n, 0); 
    static vector<unsigned short> iDst(sr_n, 0); 
    unsigned short* pD = values; 

    int k=0; 
    for(int j=0; j<sr_rows; j++)
      for(int i=0; i<sr_cols; i++)
      {
        iy[k] = j ;
        ix[k] = i ;
        iDst[k] = pD[j*sr_cols + i]; 
        ++k;
      }

    // compute XYZ from (u,v,D)
    static vector<float> lx(sr_n, 0); 
    static vector<float> ly(sr_n, 0); 
    static vector<float> lz(sr_n, 0); 
    SR_CoordTrfPntFlt(cam, &ix[0], &iy[0], &iDst[0], &lx[0], &ly[0], &lz[0], sr_n); 

    // assign to the point cloud 
    pc->points.resize(sr_n); 
    for(k=0; k<sr_n; k++)
    {
      sr_point_type& pt = pc->points[k]; 
      pt.x = lx[k]; pt.y = ly[k]; pt.z = lz[k];
    }
    return true;
}
*/

bool CNodeWrapper::sr_projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d, 
    sr_cloudPtr& pc)
{
  bool use_feature_min_depth = ParameterServer::instance()->get<bool>("use_feature_min_depth"); //TODO
  size_t max_keyp = ParameterServer::instance()->get<int>("max_keypoints");
 
  cv::Point2f p2d;
  if(feature_locations_3d.size())
  {
    feature_locations_3d.clear();
  }
  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/)
  {
    p2d = feature_locations_2d[i].pt;
    if (p2d.x >= d_cols_ || p2d.x < 0 ||
        p2d.y >= d_rows_ || p2d.y < 0 ||
        std::isnan(p2d.x) || std::isnan(p2d.y))
    { //TODO: Unclear why points should be outside the image or be NaN
      ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    float  X,Y,Z; 
    // int u = round(p2d.x); int v = round(p2d.y); 
    int u = (int)p2d.x; int v = (int)p2d.y;
    int index = v * sr_cols + u; 
    // sr_point_type& pt = pc->points[index]; 
    sr_point_type& pt = pc->at(u,v);
    X = pt.x ; Y = pt.y; Z = pt.z; 
    if(std::isnan (Z) || std::isnan(X) || std::isnan(Y) || Z < 0.01)
    {
      // ROS_DEBUG("Feature %d has been extracted at NaN depth. Omitting", i);
      //FIXME Use parameter here to choose whether to use
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    /*
    if(i==259)
    {
      ROS_WARN("feature 259 u = %d v = %d X Y Z = %f %f %f", u, v, X, Y, Z); 
    }*/
    feature_locations_3d.push_back(Eigen::Vector4f(X, Y, Z, 1.0));
    i++; //Only increment if no element is removed from vector
    if(feature_locations_3d.size() >= max_keyp) break;
  }

  feature_locations_2d.resize(feature_locations_3d.size());
  // ROS_INFO("Feature 2d size: %zu, 3D: %zu", feature_locations_2d.size(), feature_locations_3d.size());
  // ROS_ERROR("Feature 2d size: %zu, 3D: %zu", feature_locations_2d.size(), feature_locations_3d.size());

}

void CNodeWrapper::sr_projectTo3DSiftGPU(std::vector<cv::KeyPoint>& feature_locations_2d,
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
    sr_cloudPtr& point_cloud, 
    std::vector<float>& descriptors_in, cv::Mat& descriptors_out)
{
  size_t max_keyp = ParameterServer::instance()->get<int>("max_keypoints");
  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    // ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  std::list<int> featuresUsed;

  int index = -1;
  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    ++index;

    p2d = feature_locations_2d[i].pt;
    sr_point_type& p3d = point_cloud->at((int) p2d.x,(int) p2d.y);
    if (std::isnan(p3d.z) || p3d.z < 0.01)
    {
      // ROS_INFO("Feature %d has been extracted at NaN depth. Using pixel coordinates", i);
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
#ifdef HEMACLOUDS
    int target_label = ParameterServer::instance()->get<int>("segment_to_optimize");
    if(target_label >= 0 && searchLabelInNeighborhood(point_cloud, p2d, 1, target_label)){ // Optimize transformation estimation specifically for features in segment
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
#endif
    feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1));
    // if(i<10)
      // ROS_INFO("feature point at %d has %f %f %f %f %f %f", i, p2d.x, p2d.y, p3d.z, p3d.x, p3d.y, p3d.z);
    i++; //Only increment if no element is removed from vector
    featuresUsed.push_back(index);  //save id for constructing the descriptor matrix
    if(feature_locations_3d.size() >= max_keyp) break;
  }
  
  // create descriptor matrix
  int size = feature_locations_3d.size();
  descriptors_out = cv::Mat(size, 128, CV_32F);
  siftgpu_descriptors.resize(size * 128);
  for (int y = 0; y < size && featuresUsed.size() > 0; ++y) {
    int id = featuresUsed.front();
    featuresUsed.pop_front();

    for (int x = 0; x < 128; ++x) {
      descriptors_out.at<float>(y, x) = descriptors_in[id * 128 + x];
      siftgpu_descriptors[y * 128 + x] = descriptors_in[id * 128 + x];
    }
  }
  feature_locations_2d.resize(feature_locations_3d.size());
}

CNodeWrapper::CNodeWrapper(const CNodeWrapper& rhs)
{
    // 1 ids
    id_ = rhs.id_;
    seq_id_ = rhs.seq_id_;
    vertex_id_ = rhs.vertex_id_;
    matchable_ = rhs.matchable_;
    initial_node_matches_ = rhs.initial_node_matches_;

    // 2 point cloud
    pc_col = rhs.pc_col->makeShared();
    
    // 3 flann, this will be created when matched
    flannIndex = NULL;
    
    // 4 transform
    valid_tf_estimate_ = rhs.valid_tf_estimate_;
    base2points_ = rhs.base2points_;
    ground_truth_transform_ = rhs.ground_truth_transform_;
    odom_transform_ = rhs.odom_transform_;
    
    // 5 features in 2D
    feature_locations_2d_.resize(rhs.feature_locations_2d_.size());
    for(int i=0;i<feature_locations_2d_.size();i++)
    {
        feature_locations_2d_[i] = rhs.feature_locations_2d_[i];
    }
    unsigned int N = rhs.feature_depth_stats_.size();
    for(int i=0; i<N; i++)
    {
        const pair<float, float>& item = rhs.feature_depth_stats_[i];
        feature_depth_stats_.push_back(make_pair(item.first, item.second));
    }
    feature_matching_stats_.insert(feature_matching_stats_.begin(),
        rhs.feature_matching_stats_.begin(),rhs.feature_matching_stats_.end());
    
    // 6 feature in 3D
    feature_locations_3d_.insert(feature_locations_3d_.begin(), 
        rhs.feature_locations_3d_.begin(), rhs.feature_locations_3d_.end());
    feature_descriptors_ = rhs.feature_descriptors_.clone();

#ifdef  DO_FEATURE_OPTIMIZATION
    // 7 landmark 
    // std::map<int, int> kpt_to_landmark;
    kpt_to_landmark.clear();
    map<int, int>::const_iterator it_rhs = rhs.kpt_to_landmark.begin();
    while(it_rhs!=rhs.kpt_to_landmark.end())
    {
        kpt_to_landmark[it_rhs->first] = it_rhs->second;
        ++it_rhs;
    }
#endif
    
// TODO:
#ifdef USE_PCL_ICP
     // filtered_pc_col(new pointcloud_type()),
#endif
#ifdef USE_ICP_CODE
    gicp_point_set_ = NULL;
#endif
#ifdef USE_SIFT_GPU
#endif
    
    // reset flannindex 
    flannIndex = new cv::flann::Index(feature_descriptors_, cv::flann::KDTreeIndexParams(4));
}

CNodeWrapper& CNodeWrapper::operator=(const CNodeWrapper& rhs)
{   
    if(this == &rhs)
    {
        return (*this);
    }
        // 1 ids
    id_ = rhs.id_;
    seq_id_ = rhs.seq_id_;
    vertex_id_ = rhs.vertex_id_;
    matchable_ = rhs.matchable_;
    initial_node_matches_ = rhs.initial_node_matches_;

    // 2 point cloud
    pc_col = rhs.pc_col->makeShared();
    
    // 3 flann, this will be created when matched
    flannIndex = NULL;
    
    // 4 transform
    valid_tf_estimate_ = rhs.valid_tf_estimate_;
    base2points_ = rhs.base2points_;
    ground_truth_transform_ = rhs.ground_truth_transform_;
    odom_transform_ = rhs.odom_transform_;
    
    // 5 features in 2D
    feature_locations_2d_.resize(rhs.feature_locations_2d_.size());
    for(int i=0;i<feature_locations_2d_.size();i++)
    {
        feature_locations_2d_[i] = rhs.feature_locations_2d_[i];
    }
    unsigned int N = rhs.feature_depth_stats_.size();
    for(int i=0; i<N; i++)
    {
        const pair<float, float>& item = rhs.feature_depth_stats_[i];
        feature_depth_stats_.push_back(make_pair(item.first, item.second));
    }
    feature_matching_stats_.insert(feature_matching_stats_.begin(),
        rhs.feature_matching_stats_.begin(),rhs.feature_matching_stats_.end());
    
    // 6 feature in 3D
    feature_locations_3d_.insert(feature_locations_3d_.begin(), 
        rhs.feature_locations_3d_.begin(), rhs.feature_locations_3d_.end());
    feature_descriptors_ = rhs.feature_descriptors_.clone();

#ifdef  DO_FEATURE_OPTIMIZATION
    // 7 landmark 
    // std::map<int, int> kpt_to_landmark;
    kpt_to_landmark.clear();
    map<int, int>::const_iterator it_rhs = rhs.kpt_to_landmark.begin();
    while(it_rhs!=rhs.kpt_to_landmark.end())
    {
        kpt_to_landmark[it_rhs->first] = it_rhs->second;
        ++it_rhs;
    }
#endif
    
// TODO:
#ifdef USE_PCL_ICP
     // filtered_pc_col(new pointcloud_type()),
#endif
#ifdef USE_ICP_CODE
    gicp_point_set_ = NULL;
#endif
#ifdef USE_SIFT_GPU
#endif
    return (*this);
}

void CNodeWrapper::setGTPose(float *p)
{
  gt_T_ = tf::Transform(tf::Quaternion(p[3], p[4], p[5], p[6]), tf::Vector3(p[0], p[1], p[2]));
  gt_has_been_set_ = true;
}

bool CNodeWrapper::read(string path, int vertex_id)
{
  // ROS_WARN("NodeWrapper.cpp: start to read node!");
  stringstream ss; 
  ss<<path<<"/nodes/node_"<<vertex_id<<".log";
  ifstream inf(ss.str().c_str()); 

  // now only access swiss ranger data from nodes on disk
  d_N_ = sr_n; 
  d_rows_ = sr_rows; 
  d_cols_ = sr_cols; 

  if(!inf.is_open())
  {
    cerr<<"NodeWrapper.cpp: failed to open file: "<<ss.str()<<endl;
    return false;
  }
  
  // 1 ids 
  int match_i;
  inf>> id_>>seq_id_>>vertex_id_>>match_i>>initial_node_matches_; 
  // assert(vertex_id_ == vertex_id); 
  matchable_ = (match_i==1); 
  
  // 2 point cloud 
  {
    // ROS_WARN("NodeWrapper.cpp: start to read pcd!");
    stringstream ss; 
    ss<<path<<"/pcds/pc_"<<vertex_id<<".pcd"; 
    pc_col = pointcloud_type::Ptr(new pointcloud_type());
    pcl::io::loadPCDFile(ss.str().c_str(), *pc_col); //, true); 
  }

  // 3 flann
  // 4 TODO: transform , 
  
  // 5 features in 2D 
  unsigned int N; 
  inf>>N; 
  if(N <= 0)
  {
    cout<<"NodeWrapper.cpp: what? node: "<<id_<<" has no feature ? "<<endl;
    inf.close();
    return true;
  }
  
  // 5.1
  feature_locations_2d_.resize(N); 
  for(int i=0; i<N; i++)
  {
    cv::KeyPoint& p = feature_locations_2d_[i]; 
    inf>>p.pt.x>>p.pt.y>>p.size>>p.angle>>p.response>>p.octave>>p.class_id;
    // if(i <= 10)
      // ROS_WARN("NodeWrapper.cpp: the last feature 2D %f %f %f %f %f %f %f", p.pt.x, p.pt.y, p.size, p.angle, p.response, p.octave, p.class_id);
  }
  // ROS_WARN("NodeWrapper.cpp: finish reading feature_2d!");

  // 5.2 
  /*
  feature_depth_stats_.resize(N); 
  for(int i=0; i<N; i++)
  {
    pair<float, float>& item = feature_depth_stats_[i]; 
    inf>>item.first>>item.second; 
  }*/
  
  // 6 features in 3D 
  // 6.1 locations 
  feature_locations_3d_.resize(N);
  for(int i=0; i<N; i++)
  {
    Eigen::Vector4f& p = feature_locations_3d_[i]; 
    inf>>p(0)>>p(1)>>p(2)>>p(3); 
    // if(i == N-1)
      // ROS_WARN("NodeWrapper.cpp: the last position is %f %f %f %f", p(0), p(1), p(2), p(3));
  }

  // 6.2 descriptors
  unsigned int M; 
  inf>>M; 
  // ROS_WARN("NodeWrapper.cpp: read feature size %d x %d", N, M);
  feature_descriptors_ = cv::Mat(N, M, CV_32FC1);
  for(int i=0; i<N; i++)
    for(int j=0; j<M; j++)
    {
      inf>>feature_descriptors_.at<float>(i, j);
    }
#ifdef USE_SIFT_GPU
  inf>>M;
  // ROS_WARN("NodeWrapper.cpp: read SIFT GPU size %d ", M);
  siftgpu_descriptors.resize(M); 
  for(int i=0; i<M; i++)
  {
    inf>>siftgpu_descriptors[i];
  }
#endif

#ifdef DO_FEATURE_OPTIMIZATION
  cerr<<"NodeWrapper.cpp: not handle kpt_to_landmark yet!"<<endl;
#endif

#ifdef USE_PCL_CODE
  cerr<<"NodeWrapper.cpp: not handle gicp data yet!"<<endl;
#endif
  inf.close();
  
  // ROS_WARN("NodeWrapper.cpp: finish read node!");

  return true;
}

bool CNodeWrapper::write(string path)
{
  stringstream ss; 
  ss<<path<<"/nodes/node_"<<vertex_id_<<".log";
  ofstream ouf(ss.str().c_str()); 
  
  // 1 ids 
  ouf << id_<<" "<<seq_id_<<" "<<vertex_id_<<" "<<(matchable_?1:0)<<" "<<initial_node_matches_<<endl; 
  
  // 2 point cloud
  {
    stringstream ss;
    ss<<path<<"/pcds/pc_"<<vertex_id_<<".pcd";
    pcl::io::savePCDFile(ss.str().c_str(), *pc_col, true);
  }
  
  // 3 flann 
  // flannIndex will be created when matched 
  
  // 4 TODO: transform , 
  // base2points 
  // ground_truth_transform_ 
  // odom_transform_
  
  // 5 features in 2D 
  unsigned int N = feature_locations_2d_.size(); 
  ouf<< N<<endl;
  if(N <= 0)
  {
    cout<<"NodeWrapper.cpp: what? node: "<<id_<<" has no feature? "<<endl;
    ouf<<0;
    ouf.close(); 
    return true; 
  }

  // 5.1
  for(int i=0; i<N; i++)
  {
    cv::KeyPoint& p = feature_locations_2d_[i]; 
    ouf<<p.pt.x<<" "<<p.pt.y<<" "<<p.size<<" "<<p.angle<<" "<<p.response<<" "<<p.octave<<" "<<p.class_id<<endl;
  }
  
  /*
  N = feature_depth_stats_.size();
  // 5.2 
  for(int i=0; i<N; i++)
  {
    pair<float, float>& item = feature_depth_stats_[i]; 
    ouf<<item.first<<" "<<item.second<<" "<<endl;
  }*/

  // 6 features in 3D 
  // 6.1 locations
  N = feature_locations_3d_.size(); 
  for(int i=0; i<N; i++)
  {
    Eigen::Vector4f& p = feature_locations_3d_[i]; 
    ouf<<p(0)<<" "<<p(1)<<" "<<p(2)<<" "<<p(3)<<endl;
  }
  
  // 6.2 descriptors 
  unsigned int M = feature_descriptors_.cols;
  ouf<<M<<endl;
  for(int i=0; i<N; i++)
  {
    for(int j=0; j<M; j++)
    {
      ouf<<feature_descriptors_.at<float>(i,j)<<" ";
    }
    ouf<<endl;
  }

#ifdef USE_SIFT_GPU
  M = siftgpu_descriptors.size(); 
  ouf<<M<<endl;
  for(int i=0; i<siftgpu_descriptors.size(); i++)
  {
    ouf<<siftgpu_descriptors[i]<<" ";
  }
#endif

#ifdef DO_FEATURE_OPTIMIZATION
  cerr<<"NodeWrapper.cpp: not handle kpt_to_landmark yet!"<<endl;
#endif

#ifdef USE_PCL_CODE
  cerr<<"NodeWrapper.cpp: not handle gicp data yet!"<<endl;
#endif
  ouf.close();
  return true;
}

/*
MatchingResult CNodeWrapper::matchNodePair(const Node* older_node)
{
  MatchingResult mr;
  ///First check if this node has the information required
  if(older_node->pc_col->size() == 0 || older_node->feature_locations_2d_.size() == 0){
    // ROS_WARN("Tried to match against a cleared node (%d). Skipping.", older_node->id_); 
    return mr;
  }
  ParameterServer* ps = ParameterServer::instance();
  if(ps->get<int>("max_connections") > 0 && initial_node_matches_ > ps->get<int>("max_connections")) {
    return mr; //enough is enough
  }

  try{
    ///FEATURE MATCHING+RANSAC
    bool found_transformation = false;

    this->featureMatching(older_node, &mr.all_matches); 
    double ransac_quality = 0;
    if (mr.all_matches.size() < (unsigned int) ps->get<int>("min_matches")){
        // ROS_INFO("Too few inliers between %i and %i for RANSAC method. Only %i correspondences to begin with.",
         //        older_node->id_,this->id_,(int)mr.all_matches.size());
    } 
    else {//All good for feature based transformation estimation
        if(getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches))
        {
          pairwiseObservationLikelihood(this, older_node, mr);
          bool valid_tf = observation_criterion_met(mr.inlier_points, mr.outlier_points, mr.occluded_points + mr.inlier_points + mr.outlier_points, ransac_quality);
          if(valid_tf){
            edgeFromMatchingResult(this, older_node, mr.ransac_trafo, mr);
            // printNNRatioInfo("valid", mr.inlier_matches);
            found_transformation = true;
          }
        } 
        else {//Informational output only
          // printNNRatioInfo("invalid", mr.all_matches);
          ROS_INFO("RANSAC match fails!");
        }
        if(!found_transformation) mr.inlier_matches.clear();
    } 
    
    // ros::Time before_icp = ros::Time::now();
    ///ICP - This sets the icp transformation in "mr", if the icp alignment is better than the ransac_quality
    // found_transformation = edge_from_icp_alignment(found_transformation, this, older_node, mr, ransac_quality) || found_transformation;
    // ros::Time after_icp = ros::Time::now();
    // long icp_duration = abs(static_cast<long>(after_icp.nsec) - static_cast<long>(before_icp.nsec));
    // long icp_duration = ms_rostimeDiff(before_icp, after_icp);
    // if(icp_duration > 0)
       //  ROS_INFO("!!~ ICP cost time: %ld ",icp_duration);

    if(found_transformation) {
        // ROS_INFO("Returning Valid Edge");
        ++initial_node_matches_; //trafo is accepted
    } else {
        mr.edge.id1 = mr.edge.id2 = -1;
    }
  }
  catch (std::exception e){//Catch exceptions: Unexpected problems shouldn't crash the application
    ROS_ERROR("Caught Exception in comparison of Nodes %i and %i: %s", this->id_, older_node->id_, e.what());
  }
  return mr;
}*/

void CNodeWrapper::setPitch(float p)
{
  pitch_ = p;
}

float CNodeWrapper::getPitch()
{
  return pitch_;
}

void CNodeWrapper::Init(std_msgs::Header& depth_header)
{
    ParameterServer* ps = ParameterServer::instance();
  
    // initialization 
    id_ = -1; seq_id_ = -1; vertex_id_ = -1; valid_tf_estimate_ = true; matchable_ = true; 
    flannIndex = NULL; header_ = depth_header; 
    base2points_ = tf::StampedTransform(tf::Transform::getIdentity(), depth_header.stamp, ps->get<std::string>("base_frame_name"), depth_header.frame_id); 
    ground_truth_transform_ = tf::StampedTransform(tf::Transform::getIdentity(), depth_header.stamp, ps->get<std::string>("ground_truth_frame_name"), ps->get<std::string>("base_frame_name")); 
    odom_transform_ = tf::StampedTransform(tf::Transform::getIdentity(), depth_header.stamp, "missing_odometry", depth_header.frame_id);
    initial_node_matches_ = 0; 
    gt_has_been_set_ = false;
    mpCamModel = NULL; 
}

/*
void CNodeWrapper::Init()
{
    bool resetPC = false;
    bool resetGICP = false;
    ParameterServer* ps = ParameterServer::instance();
*/
    /*
#ifndef USE_PCL_ICP
    resetPC = true;
#else
    if(!ps->get<bool>("use_icp"))
        resetPC = true;
#endif    

#ifndef USE_ICP_CODE
    resetGICP = true;
#else
    if(!(!ps->get<bool>("store_pointclouds") && ps->get<bool>("use_icp")))
        resetGICP = true;
#endif

    if(resetPC)
    {
        filterCloud(*pc_col, *filtered_pc_col, ps->get<int>("gicp_max_cloud_size")); 
    }
    
    if(resetGICP)
    {
        gicp_initialized = false;
        gicp_point_set_ = NULL;
        {//if clearing out point clouds, the icp structure needs to be built before
            gicp_mutex.lock();
            gicp_point_set_ = this->getGICPStructure();
            gicp_mutex.unlock();
        } 
    }
    ROS_INFO("before set gicp debug!");
     gicp_mutex.lock();
        dgc::gicp::GICPPointSet* gicp_point_set = this->getGICPStructure();
        gicp_point_set->SetDebug(false);
     gicp_mutex.unlock();
     */
//}



