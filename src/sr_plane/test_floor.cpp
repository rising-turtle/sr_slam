/*
 *  Sep. 28, 2015 David Z
 *  extract floor from SR4k data, and draw sift features on the image, 
 *  make sure the plane is right and then use these features to do VO
 *
 * */

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <ros/ros.h>

#include <QApplication>
#include <QObject>

#include "vtk_viewer.h"
#include "../sr_publish/SR_reader.h"
#include "../sr_slam/NodeWrapper.h"
#include "../sr_slam/OpenniWrapper.h"
#include "../sr_slam/plane_node.h"
#include "../sr_slam/paramSrvMi.h"

#include "plane.h"
#include "mean_sigma.h"

#define SHOW
// #define STATISTIC

extern void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img);
extern cv::FeatureDetector* createDetector( const std::string& detectorType );
extern cv::DescriptorExtractor* createDescriptorExtractor( const std::string& descriptorType );

using namespace std;

void test_floor(int argc, char* argv[]);
void test_floor2(); 
CNodeWrapper* fromSR(sr_data& );  // construct a node from sr_data
Node* fromSR2(sr_data& sr);
void display_plane(CPlane& floor, VectorPF3& ori_pc);
void test_floor3(); // display the locations of the features

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_floor");
  ros::NodeHandle n;
  // test_floor(argc, argv);
  // test_floor2();
  test_floor3(); 
  return 0; 
}

void getPC(VectorPF3& pc, sr_data& sr_frame);

void test_floor3()
{
  // read sr data 
  CSReader sr_reader;
  if(!sr_reader.loadAllData())
  {
    ROS_ERROR("test_floor.cpp: failed to load data!"); 
    return ;
  }
  sr_data frame; 
  bool valid;
  frame = sr_reader.get_current_frame(valid);
  Node* pNode = fromSR2(frame);
  CPlaneNode* pNodePlane = dynamic_cast<CPlaneNode*>(pNode); 
  
  // pNodePlane->intersect_to_plane();
  CloudPtr pc_f2D_ori(new Cloud); 
  int M = pNode->feature_locations_3d_.size(); 
  pc_f2D_ori->points.resize(M);
  for(int i=0; i<M; i++)
  {
    Point& pt = pc_f2D_ori->points[i]; 
    pt.x = pNode->feature_locations_3d_[i](0); pt.y = pNode->feature_locations_3d_[i](1); pt.z = pNode->feature_locations_3d_[i](2);
    // pt.r = pt.g = pt.b = 1.;
  }

  if(!pNodePlane->intersect_to_planes())
  // if(!pNodePlane->intersect_to_plane2())
  // if(!pNodePlane->intersect_to_floor())
  {
    ROS_ERROR("test_floor.cpp: failed to compute the intersect points with multiple planes!");
    return ;
  }
  
  // display plane, features, and projected features 
  CloudPtr pc_floor(new Cloud);
  CloudPtr pc_f2D(new Cloud);
  CloudPtr pc_f2D_p(new Cloud);
  VectorPF3 ori_pc; 
  fromPC(ori_pc, pNode->pc_col);
  // ori_pc = copyPercent(ori_pc, 0.5);
  toPC(ori_pc, pc_floor); 
  
  // features 
  M = pNode->feature_locations_3d_.size(); 
  
  if(M == pc_f2D_ori->points.size())
  {
    ROS_ERROR("test_floor.cpp: no feature is deleted, M = %d", M);
  }

  pc_f2D->points.resize(M);
  for(int i=0; i<M; i++)
  {
    Point& pt = pc_f2D->points[i]; 
    pt.x = pNode->feature_locations_3d_[i](0); pt.y = pNode->feature_locations_3d_[i](1); pt.z = pNode->feature_locations_3d_[i](2);
    // pt.r = pt.g = pt.b = 1.;
  }

  // projected features 
  pc_f2D_p->points.resize(M);
  for(int i=0; i<M; i++)
  {
    Point & pt = pc_f2D_p->points[i]; 
    if(!pNodePlane->b_valid_feature_projec_[i]) continue;
    if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
      continue;
    pt.x = pNodePlane->feature_locations_3d_on_plane_[i](0); 
    pt.y = pNodePlane->feature_locations_3d_on_plane_[i](1);
    pt.z = pNodePlane->feature_locations_3d_on_plane_[i](2);
  }
  
  markColor(*pc_f2D_ori, YELLOW);
  markColor(*pc_floor, WHITE); 
  markColor(*pc_f2D, RED); 
  markColor(*pc_f2D_p, GREEN); 
  
  ROS_WARN("pc_feature_size: %u ", pc_f2D->points.size());

  // display it 
  CVTKViewer<Point> viewer;
  viewer.getViewer()->addCoordinateSystem(0.2, 0,0,0);
  // viewer.addPointCloud(pc_f2D, "features"); 
  viewer.addColorSpheres(pc_f2D_ori, 0.008, "features_oru", g_color[YELLOW][0]/255., g_color[YELLOW][1]/255., g_color[YELLOW][2]/255.);
  viewer.addColorSpheres(pc_f2D, 0.008, "features", g_color[RED][0]/255., g_color[RED][1]/255., g_color[RED][2]/255.);
  viewer.addPointCloud(pc_floor, "floor");
  // viewer.addPointCloud(pc_f2D_p, "features_on_plane");
  viewer.addColorSpheres(pc_f2D_p, 0.008, "features_on_plane", g_color[GREEN][0]/255., g_color[GREEN][1]/255., g_color[GREEN][2]/255.);

  ros::NodeHandle nh("~");
  bool b_display_ray; 
  nh.param("b_display_camera_ray", b_display_ray, true);

  if(b_display_ray)
  {
    Point zero;
    for(int i=0; i<M; i++)
    {
      stringstream ss;
      if(pNodePlane->b_valid_feature_projec_[i])
      {
        Point & pt = pc_f2D_p->points[i]; 
        if(!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z))
        {
          ss<<"arrow "<<i;
          viewer.getViewer()->addLine<Point>(pc_f2D_p->points[i], pc_f2D->points[i], g_color[YELLOW][0]/255., g_color[YELLOW][1]/255., g_color[YELLOW][2]/255., ss.str().c_str());
          // viewer.getViewer()->addLine<Point>(pc_f2D_p->points[i], pc_f2D->points[i], ss.str().c_str());
        }
      }
      ss<<" zero"<<i;
      viewer.getViewer()->addLine<Point>(pc_f2D->points[i], zero, ss.str().c_str());
    }
  }
  while(!viewer.stopped())
  {
    viewer.runOnce(); 
    usleep(100000);
  }
  return ;
}

void test_floor2()
{
  // read sr data 
  CSReader sr_reader;
  if(!sr_reader.loadAllData())
  {
    ROS_ERROR("test_floor.cpp: failed to load data!"); 
    return ;
  }

  ros::NodeHandle nh("~"); 
  bool b_data_new_version; 
  bool b_display_floor;
  nh.param("sr_new_file_version", b_data_new_version, true);
  nh.param("sr_display_floor", b_display_floor, true);

  //  
  sr_data frame; 
  bool b_show_extracted_floor ; 
  bool finish_flag;
  int N = sr_reader.get_number_frames();
  vector<float> etheta(N, 0);
  vector<float> edis(N, 0);

  for(int i=0; i<N; i++)
  {
    frame = sr_reader.get_current_frame(finish_flag);
    if(finish_flag == true)
    {
      ROS_WARN("test_floor.cpp: data has been traversed, break");
      break;
    }
    
    // obtain PC 
    VectorPF3 ori_pc; 
    getPC(ori_pc, frame);

    // compute the parameterso this plane
    CPlane floor; 
    floor.computeParameters(ori_pc); 
    floor.refineParameters();
    floor.print_m1(); 

    etheta[i] = R2D(floor.pitch()); 
    edis[i] = fabs(floor.d1_);

    // display
    if(b_display_floor)
    {
      display_plane(floor, ori_pc);
    }else{}
  }

  // statistic 
  float mean, sigma; 
  compute_mu_sigma(etheta.data(), N, mean, sigma);
  cout<<"test_floor.cpp: mean pitch : "<<mean<<" sigma: "<<sigma<<endl;

  compute_mu_sigma(edis.data(), N, mean, sigma); 
  cout<<"test_floor.cpp: mean dis: "<<mean<<" sigma: "<<sigma<<endl;
  
  return ;
}

void getPC(VectorPF3& ori_pc, sr_data& sr_frame)
{
  ros::NodeHandle nh("~"); 
  bool b_data_new_version; 
  nh.param("sr_new_file_version", b_data_new_version, true);
  // extract plane 
  if(b_data_new_version)
  {
    cv::Mat depth_img_(sr_rows, sr_cols, CV_16UC1, sr_frame.dis_);
    sr_cloudPtr pc_sr(new sr_cloud_type);
    ((CNodeWrapper*)0)->sr_calSRCloud(pc_sr, depth_img_);

    // extract floor from this
    // VectorPF3 ori_pc; 
    fromPC(ori_pc, pc_sr);
  }else
  {
    SR_TYPE * px = &sr_frame.x_[0];
    SR_TYPE * py = &sr_frame.y_[0]; 
    SR_TYPE * pz = &sr_frame.z_[0];
    ori_pc.resize(SR_SIZE);
    for(int i=0; i<SR_SIZE; i++)
    {
      // sr_point_type& pt = tmp_cloud->points[i]; 
      // pt.x = -ox;  pt.y = -oy; pt.z = oz - 0.01; // 1cm shift 
      // Swap XYZ order, so:
      // SwissRange -> standard camera coordinate
      ori_pc[i][0] = -px[i]; 
      ori_pc[i][1] = -py[i];
      ori_pc[i][2] = pz[i] + 0.01;
      // pt.intensity = visual.at<unsigned char>(i)/255.;
    }
  }
  return ;
}

void display_plane(CPlane& floor, VectorPF3& ori_pc)
{
  CloudPtr pc1(new Cloud); 
  CloudPtr pc2(new Cloud); 
  CloudPtr pc3(new Cloud);
  toPC(ori_pc, pc1); 
  // floor.keepInliers();

  // dumpPC2File(floor.pts_, "floor_inlier.log");

  VectorPF3 inlier_pc = copyPercent(floor.pts_, 0.5);
  // dumpPC2File(inlier_pc, "floor_inlier_cut.log");

  toPC(inlier_pc, pc2);
  toPC(ori_pc, pc3, floor.inliers_);
  VectorPF3 tmp;
  fromPC(tmp, pc3); 
  // dumpPC2File(tmp,"tmp.log");

  markColor(*pc1, WHITE); 
  markColor(*pc2, GREEN);
  markColor(*pc3, YELLOW);
  // display it 
  CVTKViewer<Point> viewer;
  viewer.getViewer()->addCoordinateSystem(0.2, 0,0,0);
  viewer.addPointCloud(pc1, "obs"); 
  viewer.addPointCloud(pc2, "floor");
  // viewer.addPointCloud(pc3, "inlier");
  while(!viewer.stopped())
  {
    viewer.runOnce(); 
    usleep(100000);
  }
  return ;
}

void test_floor(int argc, char* argv[])
{
  int N = 500;
 
  int data_id = 20; 
  if(argc >= 2)
  {
    data_id = atoi(argv[1]);
  }
  // read sr data 
  CSReader sr_reader;
  stringstream ss; 
  ss<<"/home/davidz/work/data/SwissRanger4000/with_gt/dataset_"<<data_id;
  sr_reader.setDataDir(ss.str().c_str()); 
  sr_reader.setPrefix("d1");
  sr_reader.setSuffix("bdat");
  sr_reader.setEndFrame(N); 
  sr_reader.setVersion(true); // true -> new version, false -> old version 
  sr_reader.loadAllData(); 

  // get one sr frame
  sr_data sr_frame;
  bool flag = false; // indicate weather all the data has been visited 

#ifdef STATISTIC
  vector<float> etheta(N, 0);
  vector<float> edis(N, 0);
  for(int i=0; i<N; i++)
#endif
  {
    sr_frame = sr_reader.get_current_frame(flag); 

    cv::Mat depth_img_(sr_rows, sr_cols, CV_16UC1, sr_frame.dis_);
    sr_cloudPtr pc_sr(new sr_cloud_type);
    ((CNodeWrapper*)0)->sr_calSRCloud(pc_sr, depth_img_);

    // extract floor from this
    VectorPF3 ori_pc; 
    fromPC(ori_pc, pc_sr);
    
    // dump original point cloud
    // dumpPC2File(ori_pc, "ori_pc.log");

    CPlane floor; 
    floor.computeParameters(ori_pc); 
    floor.refineParameters();
    floor.print_m1(); 

    // cout<<"test_floor.cpp: pitch angle: "<<R2D(theta)<<" dis: "<<floor.d1_<<endl;
#ifdef STATISTIC
    etheta[i] = R2D(theta); 
    edis[i] = fabs(floor.d1_);
#endif

 #ifdef SHOW
    { 
    }
#endif
  }
#ifdef STATISTIC
  float mean, sigma; 
  compute_mu_sigma(etheta.data(), N, mean, sigma);
  cout<<"test_floor.cpp: mean pitch : "<<mean<<" sigma: "<<sigma<<endl;

  compute_mu_sigma(edis.data(), N, mean, sigma); 
  cout<<"test_floor.cpp: mean dis: "<<mean<<" sigma: "<<sigma<<endl;
#endif
  return ;
}

CNodeWrapper* fromSR(sr_data& sr)
{
  // prepare intensity image 
  cv::Mat intensity_img_(sr_rows, sr_cols, CV_16UC1, sr.intensity_);
  cv::Mat grey_mono8_img_;
  ((COpenniWrapper*)0)->convert16UC_8UC(intensity_img_, grey_mono8_img_);
  
  // prepare depth image
  cv::Mat depth_img_(sr_rows, sr_cols, CV_16UC1, sr.dis_);
  cv::Mat depth_mono8_mask_; // this one will be used in Node Construction as a mask 
  cv::Mat depth_clone_img_ = depth_img_.clone(); // because depthToCV8UC1 would change the type of input depth image, so make a copy
  depthToCV8UC1(depth_clone_img_, depth_mono8_mask_); //float can't be visualized or used as mask in float format 
  
  // prepare depth header
  std_msgs::Header depth_header; 
  static unsigned int seq_id_ = 0; 
  depth_header.seq = seq_id_ ++ ;
  depth_header.stamp = ros::Time::now();
  depth_header.frame_id = "sr_depth"; // this should be sent from the publisher 

  // prepare detector and extractor 
  cv::Ptr<cv::FeatureDetector> detector_ = createDetector("SIFT"); 
  cv::Ptr<cv::DescriptorExtractor> extractor_ = createDescriptorExtractor("SIFT");
  
  CNodeWrapper* node_ptr = new CNodeWrapper(grey_mono8_img_, depth_img_, depth_mono8_mask_, depth_header, detector_, extractor_);

  return node_ptr; 
}

Node* fromSR2(sr_data& sr)
{
  // prepare intensity image 
  cv::Mat intensity_img_(SR_HEIGHT, SR_WIDTH, CV_16UC1, sr.intensity_);
  cv::Mat grey_mono8_img_;
  ((COpenniWrapper*)0)->convert16UC_8UC(intensity_img_, grey_mono8_img_);
  
  // prepare depth image
  cv::Mat depth_img_;
  cv::Mat depth_mono8_mask_; // this one will be used in Node Construction as a mask 
  cv::Mat depth_clone_img_;  // because depthToCV8UC1 would change the type of input depth image, so make a copy
  
  ros::NodeHandle nh("~"); 
  bool b_new_version; 
  string data_suffix; 
  string path_ = ParamSrvMi::instanceMi()->get<std::string>("save_node_path"); 
  int read_data_id_; 
  nh.param("sr_start_frame", read_data_id_, 1);
  nh.param("sr_data_suffix", data_suffix, string("bdat")); 
  nh.param("sr_new_file_version", b_new_version, true);
  // nh.param("sr_data_file_dir", path_, string(""));

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

  // prepare detector and extractor 
  cv::Ptr<cv::FeatureDetector> detector_ = createDetector("SIFT"); 
  cv::Ptr<cv::DescriptorExtractor> extractor_ = createDescriptorExtractor("SIFT");
 
  // prepare depth header
  std_msgs::Header depth_header; 
  static unsigned int seq_id_ = 0; 
  depth_header.seq = seq_id_ ++ ;
  depth_header.stamp = ros::Time::now();
  depth_header.frame_id = "sr_depth"; // this should be sent from the publisher 
 
  // data.seq

  Node* node_ptr ; 
  if(!ParamSrvMi::instanceMi()->get<bool>("read_node_from_disk"))
  {
    if(data_suffix == string("dat") || !b_new_version)
    {
      vector<float> x(&sr.x_[0], &sr.x_[SR_SIZE-1]);
      vector<float> y(&sr.y_[0], &sr.y_[SR_SIZE-1]);
      vector<float> z(&sr.z_[0], &sr.z_[SR_SIZE-1]);
      node_ptr = new CPlaneNode(grey_mono8_img_, depth_img_, depth_mono8_mask_, depth_header, x, y, z, detector_, extractor_);
    }else
    {
      node_ptr = new CPlaneNode(grey_mono8_img_, depth_img_, depth_mono8_mask_, depth_header, detector_, extractor_);
    }
  }else
  {
    ROS_WARN("graph_plane.cpp: read node %d  at path %s", read_data_id_-1, path_.c_str());
    node_ptr = new CPlaneNode(path_, read_data_id_++);
  }
  return node_ptr; 
}





