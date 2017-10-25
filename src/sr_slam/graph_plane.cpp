#include "graph_plane.h"
#include "../sr_plane/glob_def.h"
#include "../sr_publish/SR_reader.h"
#include "../sr_publish/realsense_reader.h"
#include "../sr_publish/SR_writer.h"
#include "OpenniWrapper.h"
#include "NodeWrapper.h"
#include "plane_node.h"
#include "paramSrvMi.h"
// #include "misc.h"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "feature_detector.h"
#include <sstream>
#include "ros/time.h"
#include <QThread>
#include <qtconcurrentrun.h>
#include <QtConcurrentMap> 

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"

#include "std_msgs/Float32MultiArray.h"
#include "cam_model.h"
// #include "misc.h"

extern cv::FeatureDetector* createDetector( const std::string& detectorType );
extern cv::DescriptorExtractor* createDescriptorExtractor( const std::string& descriptorType );
extern void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img);
extern bool isBigTrafo(const Eigen::Isometry3d& t);
extern bool isBigTrafo(const g2o::SE3Quat& t);
//! Computes whether the motion per time is bigger than the parameters max_translation_meter and max_rotation_degree define
extern bool isSmallTrafo(const g2o::SE3Quat& t, double seconds );
extern bool isSmallTrafo(const Eigen::Isometry3d& t, double seconds );
extern g2o::SE3Quat tf2G2O(const tf::Transform t) ;

extern void mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw);

namespace{
void updateInlierFeatures(const MatchingResult& mr, Node* new_node, Node* old_node)
{
  BOOST_FOREACH(const cv::DMatch& match, mr.inlier_matches){
    assert(new_node->feature_matching_stats_.size() > match.queryIdx);
    assert(old_node->feature_matching_stats_.size() > match.trainIdx);
    unsigned char& new_flag = new_node->feature_matching_stats_[match.queryIdx];
    if(new_flag < 255) ++new_flag;
    unsigned char& old_flag = old_node->feature_matching_stats_[match.trainIdx];
    if(old_flag < 255) ++old_flag;
  }
}

template <typename T >
QMatrix4x4 eigenTF2QMatrix(const T& transf) 
{
  Eigen::Matrix<qreal, 4, 4, Eigen::RowMajor> m = transf.matrix();
  QMatrix4x4 qmat( static_cast<qreal*>( m.data() )  );
  // printQMatrix4x4("From Eigen::Transform", qmat); 
  return qmat;
}

void print_tf(ostream& out, tf::Transform tT)
{
  tfScalar r, p, y, tx, ty, tz;
  tT.getBasis().getEulerYPR(y, p, r); 
  tf::Vector3 t = tT.getOrigin(); 
  tx = t.getX(); ty = t.getY(); tz = t.getZ();
  // out<<"test_vro: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<" tx: "<<tx<<" ty: "<<ty<<" tz: "<<tz<<endl;
  out<<R2D(r)<<" "<<R2D(p)<<" "<<R2D(y)<<" "<<tx<<" "<<ty<<" "<<tz<<endl;
}

}

CGraphPlane::CGraphPlane(): 
sr_reader_(new CSReader),
rs_reader_(new CRSReader),
image_encoding_("rgb8"),
sequence_frame_id_(0),
  pause_(true)
{
  ParameterServer* ps = ParameterServer::instance();
  // detector_ = createDetector(ps->get<std::string>("feature_detector_type"));
  detector_ = myCreateDetector(ps->get<std::string>("feature_detector_type"));
  extractor_ = createDescriptorExtractor(ps->get<std::string>("feature_extractor_type"));

  // some functions of openniListenner are used 
  // ni_wrapper_.reset(new COpenniWrapper((CGraphWrapper*)this));
  ros::NodeHandle n;
  ros::NodeHandle nh_p("~");

  ParamSrvMi * psMi = ParamSrvMi::instanceMi();
  path_ = psMi->get<std::string>("save_node_path"); 

  b_write_img_ = false; 

  if(psMi->get<std::string>("process_node_method") == string("write"))
  {
    // in order to write nodes into disk, set up directory first
    // path_ = psMi->get<std::string>("save_node_path"); 
    ((CSRWriter*)0)->notExistThenCreate(path_);
    string tmp = path_ + string("/nodes"); 
    ((CSRWriter*)0)->notExistThenCreate(tmp);
    tmp = path_ + string("/pcds"); 
    ((CSRWriter*)0)->notExistThenCreate(tmp);
    if(psMi->get<bool>("save_node_image"))
    {
      b_write_img_ = true;
    }
  }
  
  // parameters for imu data 
  b_use_gyro_imu_ = false; 
  pGyroReader_ = NULL; 
  nh_p.param("imu_camera_syn_id", imu_camera_syn_id_, 0); 
  if(psMi->get<bool>("load_imu_data"))
  {
    ROS_WARN("graph_plane.cpp: try to load imu data");
    string gyro_file = psMi->get<string>("imu_file_dir");           // gyro file 
    pGyroReader_ = new CGyroEuler; 
    if(pGyroReader_->readGyro(gyro_file))
    {
      b_use_gyro_imu_ = true; 
      pGyroReader_->computeBias(); 
      pGyroReader_->computeAllEulers(); 
      imu_pub_ = n.advertise<std_msgs::Float32MultiArray>("/euler_msg",10);
      
      string syn_camera_file = psMi->get<string>("camera_syn_data");  // camera syn data 
      if(load_syn_camera_time(syn_camera_file))
      {
        ROS_WARN("graph_plane.cpp: succeed to load %d time stamp for camera", time_step_.size());
      }

      ROS_WARN("graph_plane.cpp: succeed to load %d imu data", pGyroReader_->gyro_rpy_.size());

    }else
    {
      delete pGyroReader_; 
      pGyroReader_ = NULL;
    }
  }else
  {
    ROS_WARN("graph_plane.cpp: do not use imu data");
  }

  // parameters for storing the nodes
  nh_p.param("sr_start_frame", write_data_id_, 1);
  read_data_id_ = write_data_id_;
  // nh_p.param("sr_data_file_dir", path_, string("./"));
  
  nh_p.param("publish_pose_3d", b_publish_pose_3d_, false);
  pose_publisher_ =  n.advertise<geometry_msgs::PoseStamped>("/offline/pose", 1); 
}
CGraphPlane::~CGraphPlane()
{
  if(pGyroReader_!=NULL)
  {
    delete pGyroReader_; 
    pGyroReader_ = NULL;
  }
}

bool CGraphPlane::load_syn_camera_time(string fname)
{
  ifstream inf(fname.c_str()); 
  if(!inf.is_open())
  {
    ROS_ERROR("graph_plane.cpp: failed to load syn timestamp %s", fname.c_str()); 
    return false; 
  }
  int t; 
  while(!inf.eof())
  {
    inf>>t; 
    time_step_.push_back(t); 
  }
  ROS_WARN("graph_plane.cpp: succeed to read time stamp %d", time_step_.size());
}

void CGraphPlane::loadSkData()
{
  ROS_INFO("graph_plane.cpp: played in loadSkData()");
  if(ParamSrvMi::instanceMi()->get<std::string>("run_data") == std::string("sr4k"))
  {
    QtConcurrent::run(this, &CGraphPlane::loadSkDataAsync);
  }else if(ParamSrvMi::instanceMi()->get<std::string>("run_data") == std::string("realsense"))
  {
     QtConcurrent::run(this, &CGraphPlane::loadRSDataAsync);
  }else if(ParamSrvMi::instanceMi()->get<std::string>("run_data") == std::string("RGBD_R200"))
  {
    QtConcurrent::run(this, &CGraphPlane::loadR200RGBDAsync);
  }
  return ; 
}

void CGraphPlane::loadR200RGBDAsync()
{
  int rs_start_id; 
  int rs_end_id; 
  int rs_data_strip; 
  string rs_data_dir; 
  ros::NodeHandle nh_p("~"); 
  nh_p.param<int>("r200_strip", rs_data_strip, 1);
  nh_p.param<int>("r200_start", rs_start_id, 1); 
  nh_p.param<int>("r200_end", rs_end_id, 100); 
  nh_p.param<std::string>("r200_dir", rs_data_dir, string("")); 
  
  // generate the camera models 
  // CamModel cam_info(608.167/2, 605.7/2, 323.72/2, 228.84/2, 0.1816, -0.6086); // R200 
  // cam_info.width = 320; 
  // cam_info.height = 240; 

  // CamModel cam_info(606.508, 607.075, 316.00, 244.682, 0.11064, -0.55174); // F200 

  CamModel cam_info(581.902, 581.902, 319.5, 239.5);
  
  cam_info.width = 640; 
  cam_info.height = 480; 

  for(int i=rs_start_id; i<rs_end_id; i++)
  {
    stringstream ss_rgb, ss_dpt; 
    ss_rgb << rs_data_dir<<"/color/"<<setfill('0')<<setw(6)<<i<<".png"; 
    ss_dpt << rs_data_dir<<"/depth/"<<setfill('0')<<setw(6)<<i<<".pgm";  //"png" 
    
    cv::Mat rgb = cv::imread(ss_rgb.str().c_str(), -1); 
    cv::Mat dpt = cv::imread(ss_dpt.str().c_str(), -1); 
    if(!rgb.data || !dpt.data)
    {
      ROS_ERROR("%s failed to load rgb or dpt data img: %s", __FILE__, ss_rgb.str().c_str()); 
      break; 
    }

    // handle it 
    Q_EMIT progress(3, "Handling RealSense file", i);
    try{
      do{
        usleep(150); 
        if(!ros::ok())
        {
          ROS_WARN("graph_plane.cpp: ros is not ok, return!");
          return ;
        }
      }while(pause_);
      // rsCallback(*rs_frame);
      // skCallback(sr_frame);
      Node* pnew_node = fromRGBD(rgb, dpt, cam_info); 
      processNode(pnew_node); 
      // sr_frame = sr_reader_->get_current_frame(finished); 
    }catch(...)
    {
      ROS_ERROR("graph_plane.cpp: caught exception when processing rs_frame %d", i);
    }
  }

}

void CGraphPlane::loadRSDataAsync()
{
  int rs_start_id = rs_reader_->start_frame_; 
  int rs_end_id = rs_reader_->end_frame_; 
  int rs_data_strip ; 
  ros::NodeHandle nh_p("~"); 
  nh_p.param<int>("rs_data_strip", rs_data_strip, 1);
  rs_data* rs_frame = new rs_data; 
  for(int i=rs_start_id; i<rs_end_id; i+= rs_data_strip)
  {
    if(!rs_reader_->readRSID(i, *rs_frame))
    {
      ROS_ERROR("graph_plane.cpp: failed to read rs_data %d", i); 
      break;
    }
 
    Q_EMIT progress(3, "Handling RealSense file", i);
    try{
      do{
        usleep(150); 
        if(!ros::ok())
        {
          ROS_WARN("graph_plane.cpp: ros is not ok, return!");
          return ;
        }
      }while(pause_);
      rsCallback(*rs_frame);
      // skCallback(sr_frame);
      // sr_frame = sr_reader_->get_current_frame(finished); 
    }catch(...)
    {
      ROS_ERROR("graph_plane.cpp: caught exception when processing rs_frame %d", i);
    }
  }
  delete rs_frame;
}

void CGraphPlane::loadSkDataAsync()
{
  bool ret = sr_reader_->loadAllData(); 
  int i = 0;
  int sr_data_strip ; 
  ros::NodeHandle nh_p("~"); 
  nh_p.param<int>("sr_data_strip", sr_data_strip, 1);
  int ncount = 0;
  
  if(ParamSrvMi::instanceMi()->get<bool>("compute_inlier_outlier_gt"))
  {
    string file = ParamSrvMi::instanceMi()->get<string>("gt_file"); 
    if(sr_reader_->synFromGT(file))
    {
      ROS_WARN("graph_plane.cpp: succeed to read GT from file %s", file.c_str()); 
    }else{
      ROS_WARN("graph_plane.cpp: though compute in/outlier gt, failed to set gt ");
    }
  }

  if(ret)
  {
    sr_data sr_frame;
    Q_EMIT progress(3, "Handling SR4k file", i++);
    bool finished = true; 
    sr_frame = sr_reader_->get_current_frame(finished); 
    while(!finished)
    {
      try{
        do{
          usleep(150); 
          if(!ros::ok())
          {
            ROS_WARN("graph_plane.cpp: ros is not ok, return!");
            return ;
          }
        }while(pause_);
        skCallback(sr_frame);
        sequence_frame_id_++;
        // sr_frame = sr_reader_->get_current_frame(finished); 

        while(ncount++ < sr_data_strip)
        {
          sr_frame = sr_reader_->get_current_frame(finished); 
          if(finished) break;
        }
        ncount = 0;

        if(ParamSrvMi::instanceMi()->get<bool>("compute_inlier_outlier_gt"))
        { 
          while(!sr_frame.b_gt_has_been_set_) // filter those frames do not has ground truth transformation
          {
            sr_frame = sr_reader_->get_current_frame(finished); 
            if(finished) break;
          }
        }

      }catch(...)
      {
        ROS_ERROR("graph_plane.cpp: caught exception when processing sk_frame %d", sr_reader_->curr_frame_);
      }
    }
    ROS_WARN("graph_plane.cpp: what? exit publishing data!");
  }
  return ;
}

void CGraphPlane::rsCallback(rs_data& rs_frame)
{
  Node* node = fromRS(rs_frame);
  processNode(node);
  return ;
}

void CGraphPlane::imuCallback(Node* node)
{
      float rpy[3];
      static int index = imu_camera_syn_id_;
      static unsigned int time_elapsed = 0;
      static int camera_id = 1;
      pGyroReader_->readEulerAt(index, rpy);  // get rpy data at index 
      std_msgs::Float32MultiArray msg; 
      msg.data.resize(3);
      msg.data[0] = rpy[0]; msg.data[1] = rpy[1]; msg.data[2] = rpy[2]; 
      imu_pub_.publish(msg);
      ROS_WARN("graph_plane.cpp: publish rpy %f %f %f", rpy[0], rpy[1], rpy[2]);
      
      ((CPlaneNode*)(node))->setRPY(rpy);     // save rpy data in the node 

      time_elapsed += camera_id >= time_step_.size()?34:time_step_[camera_id++]; 
      index = imu_camera_syn_id_ + (time_elapsed/10);  // every 3 imu data relates to 1 camera data 
    
    // here, by comparing the result of VRO and the gyro readings, we figure out the relationship between the number of features 
    static map<int, Node*> g; 
    double roll, pitch, yaw, ir, ip, iy;
    if(ParamSrvMi::instanceMi()->get<bool>("b_test_imu_vro"))
    {
      static const int comp_step = 4; 
      static int id = 0;
      static ofstream ouf("imu_vro_euler_comparison.log");
      if(id++ % comp_step == 0)
      {
        if(g.size() == 0) // first node 
        {
          g[g.size()] = node; 
        }else  //
        {
          Node* early_node = g[g.size()-1]; 
          Eigen::Isometry3d t; 
          MatchingResult m = ((CPlaneNode*)(node))->VRO(early_node); 
          int matched_features = m.inlier_matches.size();
          if(matched_features > 0)
          {
            t.matrix() = m.final_trafo.cast<double>(); 
            // if(!isSmallTrafo(t, 1))
            {
              // VRO result 
              mat2RPY(m.final_trafo, roll,pitch,yaw);
              // IMU result 
              ((CPlaneNode*)(node))->diffRPY(early_node, ir, ip, iy); 
              // record the result for comparison 
              ouf<<matched_features<<" "<<roll<<" "<<pitch<<" "<<yaw<<" "<<ir<<" "<<ip<<" "<<iy<<endl;
            }
          }
         g[g.size()] = node; 
        }
      } // id % step == 0; 
    }
  return ;
}

void CGraphPlane::skCallback(sr_data& sr_frame)
{
    // ROS_INFO("%s at %d before fromSR ", __FILE__, __LINE__);
    Node* node = fromSR(sr_frame);
    if(b_use_gyro_imu_)  // publish imu data 
    {
      imuCallback(node);
      return ;
    }
   
    string method = ParamSrvMi::instanceMi()->get<std::string>("process_node_method");
    // if(ParamSrvMi::instanceMi()->get<bool>("only_display_data"))
    if(method == string("display"))
    {
      processNodeForDisplay(node);
    }else if(method == string("write"))
    {
      processNodeForWrite(node);
    } 
    else if(method == string("slam"))  // default
    {
      processNode(node);
    }else{
      ROS_ERROR("graph_plane.cpp: unknonwn node process method : %s", method.c_str());
    }
    return ;
}


void CGraphPlane::processNodeForWrite(Node* new_node)
{
  // write this node into file 
  ROS_WARN("graph_plane.cpp: try to write node id: %d", write_data_id_);
  new_node->vertex_id_ = write_data_id_ ++;
  ((CNodeWrapper*)new_node)->write(path_);

  if(b_write_img_)
  {
    stringstream ss; 
    ss<<path_<<"/image_"<<write_data_id_-1<<".png";
    cv::imwrite(ss.str().c_str(), visualization_img_);
  }
  ROS_WARN("graph_plane.cpp: succeed to write node id: %d", write_data_id_-1);
}

bool CGraphPlane::VRO(string path_1, int tar_id, string path_2, int src_id, tf::Transform& trans)
{
  Node* n_tar = new CPlaneNode(path_1, tar_id); 
  Node* n_src = new CPlaneNode(path_2, src_id);

  // FOR DEBUG
  /*
  ofstream ouf("feat_descriptor.log"); 

  unsigned int M = n_tar->feature_descriptors_.cols;
  unsigned int N = n_tar->feature_descriptors_.rows;
  ROS_WARN("graph_plane.cpp: output feature_descriptors with %d x %d", N, M);
  for(unsigned int i=0; i<N; i++)
  {
    for(unsigned int j=0; j<M; j++)
    {
      ouf<<n_tar->feature_descriptors_.at<float>(i,j)<<" ";
    }
    ouf<<endl;
  }*/
  bool ret = VRO(n_tar, n_src, trans);
  delete n_tar;
  delete n_src;
  return ret;
}


bool CGraphPlane::VRO(string path, int tar_id, int src_id, tf::Transform& trans)
{
  Node* n_tar = new CPlaneNode(path, tar_id); 
  Node* n_src = new CPlaneNode(path, src_id); 

  bool ret = VRO(n_tar, n_src, trans);
  delete n_tar;
  delete n_src;
  return ret;
}

bool CGraphPlane::VRO(sr_data& tar, sr_data& src, tf::Transform& trans)
{
  Node* n_tar = fromSR(tar); 
  Node* n_src = fromSR(src);

  bool ret = VRO(n_tar, n_src, trans);
  delete n_tar;
  delete n_src;
  return ret;
}

bool CGraphPlane::VRO(Node* tar, Node* src, tf::Transform& trans)
{
  Eigen::Matrix4f T;
  bool ret;
  string vro_strategy = ParamSrvMi::instanceMi()->get<std::string>("vro_strategy");
  {
    if(vro_strategy == string("vro_ori"))
    {
      ROS_WARN("graph_plane.cpp: USE VRO_ORI");
      ret = ((CPlaneNode*)src)->VRO_ori(tar, T); 
    }else if(vro_strategy == string("vro_my"))
    {
      ROS_WARN("graph_plane.cpp: USE VRO_MY");
      ret = ((CPlaneNode*)src)->VRO(tar, T);
    }else if(vro_strategy == string("vro_plane"))
    {
      ROS_WARN("graph_plane.cpp: USE VRO_PLANE");
      // TODO:
      // ((CPlaneNode*)src)->intersect_to_plane(); 
      // ((CPlaneNode*)tar)->intersect_to_plane();
      // ((CPlaneNode*)src)->intersect_to_planes();
      //((CPlaneNode*)tar)->intersect_to_planes();
      
      // ((CPlaneNode*)src)->intersect_to_floor();
      // ((CPlaneNode*)tar)->intersect_to_floor();

      ret = ((CPlaneNode*)src)->VRO(tar, T);
    }else if(vro_strategy == string("vro_plane_em"))
    {
      // MatchingResult mr = ((CPlaneNode*)src)->VRO(tar); 
      // ((CPlaneNode*)src)->intersect_to_planes();
      // ((CPlaneNode*)tar)->intersect_to_planes();
      // double rmse;
      // ((CPlaneNode*)src)->EMRefineTrans(tar, mr.inlier_matches, mr.rmse, mr.final_trafo, rmse, T);
      ((CPlaneNode*)src)->VRO(tar, T);
    }
    else{
      ROS_ERROR("graph_plane.cpp: mistake being here!");
    }
  }
  // if(((CPlaneNode*)src)->VRO(tar, T))
  // if(((CPlaneNode*)src)->VRO_ori(tar, T))
  if(ret)
  {
    trans = eigenTransf2TF(T);
    return true;
  }

  return false;
}

Node* CGraphPlane::fromRGBD(cv::Mat& rgb, cv::Mat& dpt, CamModel& cam_info)
{
  cv::Mat grey_mono8_mask = cv::Mat(dpt.size(), CV_8UC1); 
  cv::Mat depth_clone_img = dpt.clone(); 
  depthToCV8UC1(depth_clone_img, grey_mono8_mask);

  // send to visualization 
  if(ParameterServer::instance()->get<bool>("use_gui"))
  {
    Q_EMIT newVisualImage(cvMat2QImage(rgb, 0)); 
    Q_EMIT newDepthImage(cvMat2QImage(grey_mono8_mask, 1));
  }
  
  std_msgs::Header depth_header; 
  static unsigned int seq_id_ = 0; 
  depth_header.seq = seq_id_ ++ ;
  depth_header.stamp = ros::Time::now();

  Node* node_ptr = new CPlaneNode(rgb, dpt/*depth_clone_img*/, grey_mono8_mask, cam_info, depth_header, detector_, extractor_); 
  // Node* node_ptr = new CPlaneNode(rgb, depth_clone_img, grey_mono8_mask, depth_header, x, y, z, detector_, extractor_);
  // Node* node_ptr = new CPlaneNode(grey_mono8_img_, depth_img_, cv::Mat(), depth_header, x, y, z, detector_, extractor_);

  visualization_img_ = rgb;

  return node_ptr; 
}
  /*
Node* CGraphPlane::fromImgs(cv::Mat& rgb, cv::Mat& dpt)
{
  cv::Mat grey_mono8_img; 
  if(rgb.type() == CV_8UC3)
  {
    cvtColor(rgb, grey_mono8_img_, CV_RGB2GRAY); 
  }else
    grey_mono8_img_ = rgb.clone(); 

  cv::Mat depth_clone_img_, depth_mono8_mask_; 
  depth_clone_img_ = dpt.clone(); 
  depthToCV8UC1(depth_clone_img_, depth_mono8_mask_); 

  // send to visualization 
  if(ParameterServer::instance()->get<bool>("use_gui"))
  {
    Q_EMIT newVisualImage(cvMat2QImage(grey_mono8_img_, 0)); 
    Q_EMIT newDepthImage(cvMat2QImage(depth_mono8_mask_, 1));
  }
  // prepare depth header
  std_msgs::Header depth_header; 
  static unsigned int seq_id_ = 0; 
  depth_header.seq = seq_id_ ++ ;
  depth_header.stamp = ros::Time::now();
 
  node_ptr = new CPlaneNode(grey_mono8_img_, dpt, depth_mono8_mask_, depth_header, detector_, extractor_);
  visualization_img_ = grey_mono8_img_;

  return node_ptr; 
}*/

Node* CGraphPlane::fromRS(rs_data& rs)
{
  cv::Mat grey_mono8_img_(rs_data::HEIGHT, rs_data::WIDTH, CV_8UC1, rs.intensity_); // intensity img 
  // depth img 
  static SR_IMG_TYPE dis[rs_data::SIZE] = {0};  
  for(int i=0; i<rs_data::SIZE; i++)
  {
    dis[i] = (SR_IMG_TYPE)(rs.z_[i]*1000);
  }
  cv::Mat depth_img_ = cv::Mat(rs_data::HEIGHT, rs_data::WIDTH, CV_16UC1, dis);
  cv::Mat depth_clone_img_, depth_mono8_mask_; 
  depth_clone_img_ = depth_img_.clone();
  depthToCV8UC1(depth_clone_img_, depth_mono8_mask_); //float can't be visualized or used as mask in float format 
 
  // send to visualization 
  if(ParameterServer::instance()->get<bool>("use_gui"))
  {
    Q_EMIT newVisualImage(cvMat2QImage(grey_mono8_img_, 0)); 
    Q_EMIT newDepthImage(cvMat2QImage(depth_mono8_mask_, 1));
  }

  // construct node 
  //
  // prepare depth header
  std_msgs::Header depth_header; 
  static unsigned int seq_id_ = 0; 
  depth_header.seq = seq_id_ ++ ;
  depth_header.stamp = ros::Time::now();
 
  vector<float> x(&rs.x_[0], &rs.x_[rs_data::SIZE]);
  vector<float> y(&rs.y_[0], &rs.y_[rs_data::SIZE]);
  vector<float> z(&rs.z_[0], &rs.z_[rs_data::SIZE]);

  Node* node_ptr = new CPlaneNode(grey_mono8_img_, depth_img_, depth_mono8_mask_, depth_header, x, y, z, detector_, extractor_);
  // Node* node_ptr = new CPlaneNode(grey_mono8_img_, depth_img_, cv::Mat(), depth_header, x, y, z, detector_, extractor_);

  visualization_img_ = grey_mono8_img_;

  return node_ptr; 
}

Node* CGraphPlane::fromSR(sr_data& sr)
{
  ros::NodeHandle nh("~"); 
  bool b_new_version; 
  string data_suffix; 
  nh.param("sr_data_suffix", data_suffix, string("bdat")); 
  nh.param("sr_new_file_version", b_new_version, true);

  // prepare intensity image 
  cv::Mat intensity_img_(SR_HEIGHT, SR_WIDTH, CV_16UC1, sr.intensity_);
  cv::Mat grey_ini, grey_mono8_img_;
  if(b_new_version)
  {
    grey_ini = cv::Mat(SR_HEIGHT, SR_WIDTH, CV_8UC1, sr.mono_intensity_);
  }else
    ((COpenniWrapper*)0)->convert16UC_8UC(intensity_img_, grey_ini); 
  // cv::equalizeHist(grey_ini, grey_mono8_img_);
  grey_mono8_img_ = grey_ini; 

  // prepare depth image
  cv::Mat depth_img_;
  cv::Mat depth_mono8_mask_; // this one will be used in Node Construction as a mask 
  cv::Mat depth_clone_img_;  // because depthToCV8UC1 would change the type of input depth image, so make a copy
  
  if(data_suffix == string("dat") || !b_new_version)
  {
    // reconstruct the sr distance data
    static SR_IMG_TYPE dis[SR_SIZE] = {0};
    for(int i=0; i<SR_SIZE; i++)
    {
      dis[i] = (SR_IMG_TYPE)(sr.z_[i]*1000);
    }
    depth_img_ = cv::Mat(SR_HEIGHT, SR_WIDTH, CV_16UC1, dis);
  }else
  {
    depth_img_ = cv::Mat(SR_HEIGHT, SR_WIDTH, CV_16UC1, sr.dis_);
  }
  depth_clone_img_ = depth_img_.clone();
  depthToCV8UC1(depth_clone_img_, depth_mono8_mask_); //float can't be visualized or used as mask in float format 
  
  // ROS_INFO("depth at (51, 6) = %d (112, 32) = %d", depth_img_.at<unsigned short>(51, 6), depth_img_.at<unsigned short>(112, 32));

  // prepare depth header
  std_msgs::Header depth_header; 
  static unsigned int seq_id_ = 0; 
  depth_header.seq = seq_id_ ++ ;

  // depth_header.stamp = ros::Time::now();
  depth_header.stamp.fromSec(sr.timestamp_);

  depth_header.frame_id = "sr_depth"; // this should be sent from the publisher 
 
  // send to visualization 
  if(ParameterServer::instance()->get<bool>("use_gui"))
  {
    Q_EMIT newVisualImage(cvMat2QImage(grey_mono8_img_, 0)); 
    Q_EMIT newDepthImage(cvMat2QImage(depth_mono8_mask_, 1));
  }
  
  // data.seq

  Node* node_ptr ; 
  if(!ParamSrvMi::instanceMi()->get<bool>("read_node_from_disk"))
  {
    if(data_suffix == string("dat") || !b_new_version)
    {
      vector<float> x(&sr.x_[0], &sr.x_[SR_SIZE]);
      vector<float> y(&sr.y_[0], &sr.y_[SR_SIZE]);
      vector<float> z(&sr.z_[0], &sr.z_[SR_SIZE]);

      // test camera model

      // ROS_ERROR("graph_plane.cpp: start to read old file ");
      // node_ptr = new CPlaneNode(grey_mono8_img_, depth_img_, depth_mono8_mask_, depth_header, detector_, extractor_);
      // ROS_INFO("%s at %d node construction with point cloud", __FILE__, __LINE__);
      node_ptr = new CPlaneNode(grey_mono8_img_, depth_img_, depth_mono8_mask_, depth_header, x, y, z, detector_, extractor_);
      // ROS_INFO("after node Construction");
    }else
    {
      // ROS_INFO("here without point cloud houhou!");
      node_ptr = new CPlaneNode(grey_mono8_img_, depth_img_, depth_mono8_mask_, depth_header, detector_, extractor_);
    }
  }else
  {
    // ROS_ERROR("graph_plane.cpp: start to read node %d", read_data_id_);
    node_ptr = new CPlaneNode(path_, read_data_id_++);
    ROS_WARN("graph_plane.cpp: read node %d ", read_data_id_-1);
  }
  visualization_img_ = grey_mono8_img_;
  
  // set ground truth pose 
  if(sr.b_gt_has_been_set_)
    ((CNodeWrapper*)node_ptr)->setGTPose(sr.gt_pv_);
  
  return node_ptr; 
}

void CGraphPlane::processNodeForDisplay(Node* new_node)
{
  // display 2D feature
  if(ParameterServer::instance()->get<bool>("use_gui"))
  {
    // ROS_WARN("graph_plane.cpp: before drawKeypoints !");
    cv::drawKeypoints(visualization_img_, new_node->feature_locations_2d_, visualization_img_, cv::Scalar(0, 100,0), 5);
    Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_, 2)); //show registration
    // ROS_WARN("graph_plane.cpp: after drawKeypoints !");
  } 

  // display point cloud and 3D feature
  if(!ParameterServer::instance()->get<bool>("glwidget_without_clouds")) 
  { 
    // clear first 
    // then send data 
    QMatrix4x4 latest_transform; 
    latest_transform.setToIdentity();
    ros::NodeHandle nh("~");
    bool b_display_pc_and_feature;
    nh.param("display_pc_and_feature", b_display_pc_and_feature, false);
    if(b_display_pc_and_feature)
    {
      Q_EMIT setPointCloud(new_node->pc_col.get(), latest_transform);
      Q_EMIT setFeatures(&(new_node->feature_locations_3d_));
    }
  }

  // ROS_WARN("graph_plane.cpp: before delete new_node!");
  delete new_node; 
  // ROS_WARN("graph_plane.cpp: after delete new_node !");

  new_node = NULL;
  return ;
}

bool CGraphPlane::nodeComparisons(Node* new_node, QMatrix4x4& curr_motion_estimate, bool& edge_to_keyframe)
{
    process_node_runs_ = true;
    ParameterServer* ps = ParameterServer::instance();
    if ((int)new_node->feature_locations_2d_.size() < ps->get<int>("min_matches") && 
        ! ps->get<bool>("keep_all_nodes"))
    {
        ROS_INFO("Found only %i features on image, node is not included",(int)new_node->feature_locations_2d_.size());
        process_node_runs_ = false;
        return false;
    }
    
    new_node->id_ = graph_.size();
    new_node->seq_id_ = next_seq_id++; // allways incremented, even if node is not added

    earliest_loop_closure_node_ = new_node->id_;
    unsigned int num_edges_before = cam_cam_edges_.size();
    edge_to_keyframe = false; //not yet found
    marker_id_ = 0; //overdraw old markers
  
    // Odometry Stuff 
    int sequentially_previous_id = graph_.rbegin()->second->id_; 
    tf::StampedTransform odom_tf_new = new_node->getOdomTransform();
    tf::StampedTransform odom_tf_old = graph_[sequentially_previous_id]->getOdomTransform();
    tf::Transform odom_delta_tf = odom_tf_new * odom_tf_old.inverse();
    // printTransform("Odometry Delta", odom_delta_tf);
    if(odom_tf_old.frame_id_ == "missing_odometry" || odom_tf_new.frame_id_ == "missing_odometry"){
      ROS_INFO("No Valid Odometry, using identity");
      odom_delta_tf = tf::Transform::getIdentity();
    }
    
    MatchingResult mr;
    int prev_best = mr.edge.id1;
    curr_best_result_ = mr;

    bool b_slam_offline ; 
    ros::NodeHandle nh("~");
    nh.param("run_slam_offline", b_slam_offline, true);
    
    // for debug 
    // static ofstream ouf_tf("./rs_slam/vo_result.log");
    // static ofstream ouf_vo("./rs_slam/inc_result.log");

    //Initial Comparison ######################################################################
    bool predecessor_matched = false;
    if(ps->get<double>("min_translation_meter") > 0.0 ||
        ps->get<double>("min_rotation_degree") > 0.0)
    {
      Node* prev_frame = graph_[graph_.size()-1];
      if(localization_only_ && curr_best_result_.edge.id1 > 0){ prev_frame =  graph_[curr_best_result_.edge.id1]; }
      mr = new_node->matchNodePair(prev_frame);
      // mr = ((const CPlaneNode*)new_node)->VRO(prev_frame);

      if(mr.edge.id1 >= 0 && mr.edge.id2 >= 0) 
      {//Found trafo
        ros::Time time1 = prev_frame->header_.stamp;
        ros::Time time2 = new_node->header_.stamp;
        ros::Duration delta_time =  time2 - time1;

        if( !b_slam_offline && (!isBigTrafo(mr.edge.transform) || !isSmallTrafo(mr.edge.transform, delta_time.toSec())))
        { //Found trafo, but bad trafo (too small to big)
          //Send the current pose via tf nevertheless
          tf::Transform incremental = eigenTransf2TF(mr.edge.transform);
          g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[prev_frame->id_]->vertex_id_));
          tf::Transform previous = eigenTransf2TF(v->estimate());
          tf::Transform combined = previous*incremental;
          
          latest_transform_cache_ = stampedTransformInWorldFrame(new_node, combined);
          // printTransform("Computed new transform", latest_transform_cache_);
          broadcastTransform(latest_transform_cache_);
          process_node_runs_ = false;
          curr_best_result_ = mr;
          // ROS_ERROR("graph_plane.cpp: failed to compute Trans with node_id : %d", new_node->id_);
          return false;
        } else { //Good Transformation

          // for debug 
          // ROS_ERROR("graph_plane.cpp: succeed to compute Tran with node_id : %d", new_node->id_);
          // static tf::Transform previous = tf::Transform::getIdentity();
          // tf::Transform incremental = eigenTransf2TF(mr.edge.transform);
          // tf::Transform combined = previous*incremental;
          // print_tf(ouf_tf, combined);
          // print_tf(ouf_vo, incremental);
          // previous = combined;

          if (addEdgeToG2O(mr.edge, prev_frame, new_node,  true, true, curr_motion_estimate)) 
          {
            graph_[new_node->id_] = new_node; //Needs to be added
            if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
#ifdef DO_FEATURE_OPTIMIZATION
            updateLandmarks(mr, prev_frame,new_node);
#endif
            updateInlierFeatures(mr, new_node, prev_frame);
            graph_[mr.edge.id1]->valid_tf_estimate_ = true;
            curr_best_result_ = mr;
            //addOutliers(Node* new_node, mr.inlier_matches);
          } else {
            ROS_INFO("Edge not added");
            process_node_runs_ = false;
            return false;
          }
        } 
        predecessor_matched = true;
      }else{
        ROS_WARN("graph_plane.cpp: Found no transformation to predecessor (edge ids are negative)");
      }
    } // end: Initial Comparison      

    // prepare the previous nodes to be comparied 
    QList<int> vertices_to_comp;
    int  seq_cand = localization_only_ ? 0 : ps->get<int>("predecessor_candidates") - 1; //minus one, the predecessor has been checked
    int geod_cand = ps->get<int>("neighbor_candidates");
    int samp_cand = ps->get<int>("min_sampled_candidates");
    if(predecessor_matched)
    {
      vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, curr_best_result_.edge.id1); 
    } else 
    {
      // vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, sequentially_previous_id, true); 
      vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, sequentially_previous_id, false); 

    }
    if(prev_best >= 0 && !vertices_to_comp.contains(prev_best))
    {
      vertices_to_comp.append(prev_best);//Test: definitely reuse best (currently: the oldest) matched node from last
    }

    //MAIN LOOP: Compare node pairs ######################################################################
    QList<const Node* > nodes_to_comp;//only necessary for parallel computation
    if (ps->get<bool>("concurrent_edge_construction")) 
    {
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) 
        {
            nodes_to_comp.push_front(graph_[vertices_to_comp[id_of_id]]); 
        }
        QThreadPool* qtp = QThreadPool::globalInstance();
        if (qtp->maxThreadCount() - qtp->activeThreadCount() == 1) 
        {
           qtp->setMaxThreadCount(qtp->maxThreadCount() + 1);
        }
        QList<MatchingResult> results = QtConcurrent::blockingMapped(nodes_to_comp, boost::bind(&Node::matchNodePair, new_node, _1));

        for (int i = 0; i < results.size(); i++) 
        {
            MatchingResult& mr = results[i];
            if (mr.edge.id1 >= 0 ) 
            {
              assert(graph_[mr.edge.id1]);
              ros::Duration delta_time = new_node->header_.stamp - graph_[mr.edge.id1]->header_.stamp;
              if ((b_slam_offline || isSmallTrafo(mr.edge.transform, delta_time.toSec())) && addEdgeToG2O(mr.edge,graph_[mr.edge.id1],new_node, isBigTrafo(mr.edge.transform), mr.inlier_matches.size() > curr_best_result_.inlier_matches.size(), curr_motion_estimate))
                { 
                  graph_[new_node->id_] = new_node; //Needs to be added
#ifdef DO_FEATURE_OPTIMIZATION
                  updateLandmarks(mr, graph_[mr.edge.id1],new_node);
#endif
                  updateInlierFeatures(mr, new_node, graph_[mr.edge.id1]);
                  graph_[mr.edge.id1]->valid_tf_estimate_ = true;
                  ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                  if (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size()) 
                  {
                          curr_best_result_ = mr;
                  }
                  if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
                }
                else
                {
                  ROS_WARN("Rejected edge from %d to %d", mr.edge.id1, mr.edge.id2);
                }
              }
        }
      } else { //Nonconcurrent
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) 
        {
            Node* node_to_compare = graph_[vertices_to_comp[id_of_id]];
            MatchingResult mr = new_node->matchNodePair(node_to_compare);
            // MatchingResult mr = ((const CPlaneNode*)new_node)->VRO(node_to_compare);

            if (mr.edge.id1 >= 0) 
            {
              ros::Duration delta_time = new_node->header_.stamp - graph_[mr.edge.id1]->header_.stamp;
              if ( (b_slam_offline || isSmallTrafo(mr.edge.transform, delta_time.toSec())) &&
                  addEdgeToG2O(mr.edge, node_to_compare, new_node, isBigTrafo(mr.edge.transform), mr.inlier_matches.size() > curr_best_result_.inlier_matches.size(), curr_motion_estimate))
              {
#ifdef DO_FEATURE_OPTIMIZATION
                updateLandmarks(mr, node_to_compare, new_node);
#endif
                graph_[new_node->id_] = new_node; //Needs to be added
                updateInlierFeatures(mr, new_node, node_to_compare);
                graph_[mr.edge.id1]->valid_tf_estimate_ = true;
                if (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size()) 
                {
                  curr_best_result_ = mr;
                }
                if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
              }
              else {
                ROS_INFO("Matching result rejected for being too big? Time Delta: %f", delta_time.toSec());
              }
            }
            else 
            {
              ROS_INFO("Matching result rejected for edge.id1");
            }
        }
    }
    //END OF MAIN LOOP: Compare node pairs ######################################################################
    
    bool found_trafo = (cam_cam_edges_.size() != num_edges_before);
    bool invalid_odometry = ps->get<std::string>("odom_frame_name").empty() || 
                            odom_tf_old.frame_id_ == "missing_odometry" || 
                            odom_tf_new.frame_id_ == "missing_odometry"; 

    bool keep_anyway = (ps->get<bool>("keep_all_nodes") || 
                        (((int)new_node->feature_locations_3d_.size() > ps->get<int>("min_matches")) 
                         && ps->get<bool>("keep_good_nodes")));
    if(!invalid_odometry)
    {
      // ROS_ERROR("graph_plane.cpp: Adding odometry motion edge for Node %i (if available, otherwise using identity)", (int)graph_.rbegin()->second->id_);
      LoadedEdge3D odom_edge;
      odom_edge.id1 = sequentially_previous_id;
      odom_edge.id2 = new_node->id_;
      odom_edge.transform = tf2G2O(odom_delta_tf);

      //Real odometry
      //FIXME get odometry information matrix and transform it to the optical frame
      odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity(); //0.1m /0.1rad error
      /*
      //Assumption: Motion in the plane
      odom_edge.informationMatrix(0,0) = 100; //0.1m accuracy in the floor plane
      odom_edge.informationMatrix(1,1) = 100; //
      odom_edge.informationMatrix(2,2) = 1000000;//0.01m information on rotation w.r. to floor
      odom_edge.informationMatrix(3,3) = 400000000;//0.02rad information on rotation w.r. to floor
      odom_edge.informationMatrix(4,4) = 400000000;//0.02rad information on rotation w.r. to floor
      odom_edge.informationMatrix(5,5) = 1600; //0.4rad (~20°) on rotation about vertical
      */
       addEdgeToG2O(odom_edge,graph_[sequentially_previous_id],new_node, true,true, curr_motion_estimate);
      graph_[new_node->id_] = new_node; //Needs to be added
    }
    else if (!found_trafo && keep_anyway) // Constant position assumption 
    {
      LoadedEdge3D odom_edge;
      odom_edge.id1 = sequentially_previous_id;
      odom_edge.id2 = new_node->id_;
      odom_edge.transform.setIdentity();
      curr_motion_estimate = eigenTF2QMatrix(odom_edge.transform);
      odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Zero(); 
      odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity() * 1;//e-9; 
      odom_edge.informationMatrix(3,3) = 1e-100;
      odom_edge.informationMatrix(4,4) = 1e-100;
      odom_edge.informationMatrix(5,5) = 1e-100;
      addEdgeToG2O(odom_edge,graph_[sequentially_previous_id],new_node, true,true, curr_motion_estimate);
      graph_[new_node->id_] = new_node; //Needs to be added
      new_node->valid_tf_estimate_ = false; //Don't use for postprocessing, rendering etc
      MatchingResult mr;
      mr.edge = odom_edge;
      curr_best_result_ = mr;
    }
   return cam_cam_edges_.size() > num_edges_before;
}

void CGraphPlane::publish_new_3D_pose()
{
  tf::Transform tf_p = CGraphWrapper::getLastNodePose(); 
  
  geometry_msgs::PoseStamped pMsg;

  pMsg.pose.position.x = tf_p.getOrigin()[0];
  pMsg.pose.position.y = tf_p.getOrigin()[1];
  pMsg.pose.position.z = tf_p.getOrigin()[2];
  pMsg.pose.orientation.x = tf_p.getRotation().x();
  pMsg.pose.orientation.y = tf_p.getRotation().y();
  pMsg.pose.orientation.z = tf_p.getRotation().z(); 
  pMsg.pose.orientation.w = tf_p.getRotation().getW();

  if (pMsg.pose.orientation.w < 0)
  {
    pMsg.pose.orientation.x *= -1;
    pMsg.pose.orientation.y *= -1;
    pMsg.pose.orientation.z *= -1;
    pMsg.pose.orientation.w *= -1;
  }

  pMsg.header.stamp = ros::Time::now();
  pMsg.header.frame_id = "world";
  ROS_INFO("graph_plane.cpp: frame %d publish pos_3d : %f %f %f %f %f %f %f", graph_.size()-1, pMsg.pose.position.x, pMsg.pose.position.y, pMsg.pose.position.z, pMsg.pose.orientation.x, pMsg.pose.orientation.y, pMsg.pose.orientation.z, pMsg.pose.orientation.w);
  pose_publisher_.publish(pMsg);
  static ofstream ouf("./lsd_slam/rgbd_vro_id.log"); 
  ouf<<sequence_frame_id_<<endl;
}

void CGraphPlane::processNode(Node* new_node)
{
   //  ScopedTimer s(__FUNCTION__);
    Q_EMIT setGUIStatus("Adding Node to Graph");

    // timing the addNode process 
    double begin_time_ms = ros::Time::now().toSec()*1000.; 
    // bool has_been_added = graph_mgr_->addNode(new_node);
    bool has_been_added = CGraphWrapper::addNode(new_node);
    double end_time_ms = ros::Time::now().toSec()*1000.; 
    if(has_been_added)
    { 
      static int number_successful_added = 0; 
      // ROS_ERROR("openni_listenner.cpp: add %d node cost time %.4f ms", ++number_successful_added, end_time_ms - begin_time_ms);
      if(b_publish_pose_3d_)
        publish_new_3D_pose(); 
    }
    static int num_processed_ = 0;
    ++num_processed_;
    Q_EMIT setGUIInfo2(QString("Frames processed: ")+QString::number(num_processed_));

    //######### Visualization code  #############################################
    if(ParameterServer::instance()->get<bool>("use_gui")){
      if(has_been_added)
      {
        ROS_WARN("graph_plane.cpp: succeed to add node, drawFeatureFlow");
        drawFeatureFlow(visualization_img_, cv::Scalar(0,0,255), cv::Scalar(0,128,0) );
        Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_, 5)); //show registration
      } else {
        ROS_WARN("graph_plane.cpp: failed to add node, drawKeypoints");
        cv::drawKeypoints(visualization_img_, new_node->feature_locations_2d_, visualization_img_, cv::Scalar(0, 100,0), 5);
        Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_, 2)); //show registration
      }
    }
    if(!has_been_added) {
      delete new_node; 
      new_node = NULL;
    }
}

namespace {   

pointcloud_type *  create_rectangle()
{
  pointcloud_type* rectangle = new pointcloud_type(); 
  float l, w , h;
  l = w = h = 1; 
  point_type p1(-l, -w, -h);   point_type p2(-l, -w, h);   point_type p3(-l, w, -h);   point_type p4(-l, w, h); 
  point_type p5( l, -w, -h);   point_type p6( l, -w, h);   point_type p7( l, w, -h);   point_type p8( l, w, h); 
  rectangle->points.push_back(p1);  rectangle->points.push_back(p2);   rectangle->points.push_back(p3);
  rectangle->points.push_back(p4);  rectangle->points.push_back(p5);   rectangle->points.push_back(p6);
  rectangle->points.push_back(p7);  rectangle->points.push_back(p8);
  

  // a matrix of points 
  for(float la = -1; la <= 1; la+=0.03)
    for(float wa = -1; wa <=1; wa+=0.03)
      for(float ha = -1; ha <=1; ha+= 0.03)
      { 
        point_type p(la, wa, ha); 
        p.r = 250;
        rectangle->points.push_back(p);
      }

  rectangle->height = 1; rectangle->width = rectangle->points.size();
  rectangle->is_dense = true;

  return rectangle;
}

}

// no idea, why this not work 
void CGraphPlane::imu_gui_display()
{
  usleep(1000);
  // while(ros::ok())
  {
    // if(pause_)
    {
      ROS_INFO("graph_plane.cpp: send a rectangle to be rendered!");
      // draw a rectangle to display the robot  
      static pointcloud_type * rectangle = create_rectangle();
      ROS_WARN("graph_plane.cpp: rectangle contains %d points", rectangle->points.size());
      Eigen::Matrix4d m = Eigen::Matrix4d::Identity(); 
      Eigen::Isometry3d im; 
      im.matrix() = m;
      QMatrix4x4 qm = eigenTF2QMatrix(im); 
      Q_EMIT GraphManager::setPointCloud(rectangle, qm); 
    }
    usleep(150); 
  }
}

QImage CGraphPlane::cvMat2QImage(const cv::Mat& image, unsigned int idx)
{
  if(rgba_buffers_.size() <= idx){
    rgba_buffers_.resize(idx+1);
  }
  if(image.rows != rgba_buffers_[idx].rows || image.cols != rgba_buffers_[idx].cols)
  {
    rgba_buffers_[idx] = cv::Mat( image.rows, image.cols, CV_8UC4); 
  }
  char red_idx = 0, green_idx = 1, blue_idx = 2;
  if(image_encoding_.compare("rgb8") == 0) { red_idx = 2; blue_idx = 0; }
  cv::Mat alpha( image.rows, image.cols, CV_8UC1, cv::Scalar(255)); //TODO this could be buffered for performance
  cv::Mat in[] = { image, alpha };
   if(image.type() == CV_8UC1){
    int from_to[] = { 0,0,  0,1,  0,2,  1,3 };
    mixChannels( in , 2, &rgba_buffers_[idx], 1, from_to, 4 );
  } else {
    int from_to[] = { red_idx,0,  green_idx,1,  blue_idx,2,  3,3 }; //BGR+A -> RGBA
    mixChannels( in , 2, &rgba_buffers_[idx], 1, from_to, 4 );
  }
  return QImage((unsigned char *)(rgba_buffers_[idx].data), 
      rgba_buffers_[idx].cols, rgba_buffers_[idx].rows, 
      rgba_buffers_[idx].step, QImage::Format_RGB32 );
}

