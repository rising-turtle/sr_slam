/* 
 * July. 19, 2016 David Z
 * 
 * display matched features from dense tracking between two images 
 *
 * */

#include <ros/ros.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "../sr_publish/SR_reader.h"
#include "../sr_slam/plane_node.h"
#include "../sr_slam/graph_plane.h"
#include "../sr_plane/vtk_viewer.h"
#include "../sr_plane/glob_def.h"
#include "pcl_ros/transforms.h"
#include <tf/tf.h>
#include "util/Undistorter.h"
#include <opencv2/opencv.hpp>

#include "Tracking/SE3Tracker.h"
#include "Tracking/TrackingReferenceSE3.h"
#include "GlobalMapping/g2oTypeSE3Sophus.h"
#include "DataStructures/FrameSE3.h"

using namespace std;
using namespace cv;
using namespace lsd_slam;

#ifndef R2D(r)
#define R2D(r) ((r)*180./M_PI)
#endif

void convert16UC_8UC(cv::Mat& in_img, cv::Mat& out_img);
void srCallback(sr_data& , cv::Mat& , cv::Mat& );   // process each frame to obtain intensity and depth image
bool loadKeyPoints(string f, vector<KeyPoint>& kp_pre, vector<KeyPoint>& kp_cur, vector<DMatch>& matches, int step=1);
void display_img(Mat img, string window_name="");
bool getTransform(string trans_log, tf::Transform& ini_T, tf::Transform& fin_T);
bool getSRAndImg(sr_data& , cv::Mat&, Mat&, sr_data&, Mat&, Mat& );
void getTF_xyzrpy(tf::Transform tT, tfScalar& tx, tfScalar& ty, tfScalar& tz, tfScalar& roll, tfScalar& pitch, tfScalar& yaw); 
vector<bool> computeInlierOutlier(Mat& eh_img_pre, Mat& dpt_pre, Mat& eh_img_cur, Mat& dpt_cur, tf::Transform fin_T, 
    vector<KeyPoint>&, vector<KeyPoint>&);
void print_tf(ostream& out, tf::Transform tT);
Undistorter* getCalibUndist(); 

void getGTTransform(vector<tf::Transform>& vgt); // these transforms are result from rgbd-SLAM 

void test_display_matched_points();
void generate_error_distribution();
void computeInlierAndOutlier();

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "show_match_points");

  test_display_matched_points();

  // generate_error_distribution(); 

  // computeInlierAndOutlier(); // pack style 

  return 0; 
}

void computeInlierAndOutlier()
{
  // 1. get ground truth transformation 
  vector<tf::Transform> vgt; 
  getGTTransform(vgt);
  cout<<"first stop vgt.size() = "<<vgt.size()<<endl;

  // 2. traverse all the recorded key frames 
  string log_dir = "/home/davidz/.ros/lsd_slam/compare";
  string data_dir = "/home/davidz/work/data/SwissRanger4000/exp/dataset_82";
  int start_id = 6;
  int end_id = 12; // 94 
  int base_id = 820;
  int kf_id = 412;
  vector<KeyPoint> v_kp1, v_kp2;
  vector<DMatch> v_m;
  ofstream ouf("/home/davidz/.ros/lsd_slam/compare/sub_inlier_outlier.log");
  for(int i=start_id, j=2; i<=end_id && j<vgt.size(); i+=2, j++)
  {
    // match log 
    stringstream ss; 
    ss<<log_dir<<"/"<<i-2<<"_"<<i<<".log";
    printf("main_show_match_points.cpp: work on %s \n", ss.str().c_str());
    if(!loadKeyPoints(ss.str(), v_kp1, v_kp2, v_m, 1)){return ;}
    
    // img data 
    ros::NodeHandle nh("~");
    stringstream pre_path, cur_path;
    pre_path<<data_dir<<"/d1_0"<<base_id+i-2<<".bdat";
    cur_path<<data_dir<<"/d1_0"<<base_id+i<<".bdat";
    nh.setParam("previous_frame_path", pre_path.str()); 
    nh.setParam("current_frame_path", cur_path.str());
    
    sr_data pre_sr, cur_sr; 
    Mat eh_img_pre, eh_img_cur; 
    Mat dpt_pre, dpt_cur;

    if(!getSRAndImg(pre_sr, eh_img_pre, dpt_pre, cur_sr, eh_img_cur, dpt_cur))
    {
      cout<<"main_show_match_points.cpp: failed to get image"<<endl;
      return ;
    }   
    
    tf::Transform fin_T = vgt[j-1].inverse()*vgt[j];
    vector<bool> inlier_ind = computeInlierOutlier(eh_img_pre, dpt_pre, eh_img_cur, dpt_cur, fin_T, v_kp1, v_kp2);   
    int inlier_cnt = 0; 
    for(int i=0; i<inlier_ind.size(); i++) 
    {
      if(inlier_ind[i]) ++inlier_cnt;
    }

    cout<<kf_id<<" "<<inlier_cnt<<" "<<v_kp1.size()-inlier_cnt<<" "<<endl;
    ouf<<kf_id++<<" "<<inlier_cnt<<" "<<v_kp1.size()-inlier_cnt<<" "<<endl;
  }
}

void test_display_matched_points()
{
  sr_data pre_sr, cur_sr; 
  Mat eh_img_pre, eh_img_cur; 
  Mat dpt_pre, dpt_cur;

  if(!getSRAndImg(pre_sr, eh_img_pre, dpt_pre, cur_sr, eh_img_cur, dpt_cur))
  {
    return ;
  }

  // load matched points 
  string match_ini_log("");
  string match_fin_log("");
  ros::NodeHandle nh("~");
  nh.param("initial_match_log", match_ini_log, match_ini_log); 
  nh.param("final_match_log", match_fin_log, match_fin_log);
  
  int step = 1; 
  nh.param("show_points_step", step, step);

  vector<KeyPoint> v_kp1, v_kp2;
  vector<DMatch> v_m;
  if(!loadKeyPoints(match_ini_log, v_kp1, v_kp2, v_m, step)){return ;}
  
  cv::Mat ini_match_img, fin_match_img;
  // display initial img
  ROS_INFO("main_show_match_points.cpp: display initial matched points"); 
  // cv::drawMatches(eh_img_cur, v_kp2, eh_img_pre, v_kp1, v_m, ini_match_img); 
  // display_img(ini_match_img); 

  if(!loadKeyPoints(match_fin_log, v_kp1, v_kp2, v_m, step)){return ;}
   // display final img
  ROS_INFO("main_show_match_points.cpp: display final matched points"); 
  // cv::drawMatches(eh_img_cur, v_kp2, eh_img_pre, v_kp1, v_m, fin_match_img); 
  // display_img(fin_match_img); 
  
  // display transformation 
  string trans_log("");
  nh.param("transform_log", trans_log, trans_log); 
  tf::Transform ini_T, fin_T;
  if(!getTransform(trans_log, ini_T, fin_T))
  {
    ROS_WARN("main_show_match_points.cpp: no transform available in file %s", trans_log.c_str());
    return ;
  }

  // fin_T = ini_T.inverse()*fin_T;
  print_tf(std::cout, fin_T); 

  float qx, qy, qz, qw, x, y, z; 
  qx = -0.002; qy = 0.0306; qz = 0.0145; qw = sqrt(1-qx*qx - qy*qy - qz*qz); 
  x = -0.001;  y = -0.0033; z = 0.0223;
  // fin_T = tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x,y,z));
  
  // compute inliers and outliers
  printf("main_show_match_points: begin to compute Inlier and Outlier\n");
  vector<bool> inlier = computeInlierOutlier(eh_img_pre, dpt_pre, eh_img_cur, dpt_cur, fin_T, v_kp1, v_kp2);   
  
  // show outliers and inliers
  vector<DMatch> v_out_m;
  vector<DMatch> v_in_m;
  v_out_m.reserve(v_m.size());
  v_in_m.reserve(v_m.size());
  for(int i=0; i<inlier.size(); i++)
  {
    if(!inlier[i]) v_out_m.push_back(v_m[i]); 
    else v_in_m.push_back(v_m[i]); 
  }

  cv::Mat inlier_match_img, outlier_match_img;
  cv::drawMatches(eh_img_cur, v_kp2, eh_img_pre, v_kp1, v_out_m, outlier_match_img); 
  display_img(outlier_match_img, "outlier_match_img" );

  cv::drawMatches(eh_img_cur, v_kp2, eh_img_pre, v_kp1, v_in_m, inlier_match_img); 
  display_img(inlier_match_img, "inlier_match_img" );

  // prepare to display point cloud 
  Eigen::Matrix4f eigen_ini_T, eigen_fin_T; 
  pcl_ros::transformAsMatrix(ini_T, eigen_ini_T);
  pcl_ros::transformAsMatrix(fin_T, eigen_fin_T);

  // whether display point cloud 
  bool b_display_pc = false; 
  nh.param("show_point_cloud", b_display_pc, b_display_pc);
  if(b_display_pc)
  {
    CGraphPlane gp;
    CPlaneNode* old_node = gp.fromSR(pre_sr); 
    CPlaneNode* new_node = gp.fromSR(cur_sr); 
    
    pointcloud_type::Ptr old_pc(new pointcloud_type); 
    pointcloud_type::Ptr new_pc(new pointcloud_type); 
    pointcloud_type::Ptr new_pc_in_old(new pointcloud_type); 
    // old_pc = old_node->pc_col.make_shared();
    // new_pc = new_node->pc_col.make_shared();
    
    ROS_INFO("main_show_match_points.cpp: old_pc has %d, new_pc has %d", old_node->pc_col->points.size(), new_node->pc_col->points.size());

    pcl::transformPointCloud(*(new_node->pc_col), *new_pc_in_old, eigen_fin_T);
    markColor(*new_pc_in_old, RED);
    markColor(*(old_node->pc_col), GREEN);
    *old_pc += *new_pc_in_old;
    *old_pc += *(old_node->pc_col);
    
    ROS_INFO("main_show_match_points.cpp: merged_pc has %d points", old_pc->points.size());
    CVTKViewer<point_type> viewer; 
    viewer.getViewer()->addCoordinateSystem(0.2,0, 0, 0); 
    viewer.addPointCloud(old_pc, "final_pcs");
    while(!viewer.stopped())
    {
      viewer.runOnce(); 
      usleep(100000);
    }
  }

  return ;
}

vector<bool> computeInlierOutlier(Mat& eh_img_pre, Mat& dpt_pre, Mat& eh_img_cur, Mat& dpt_cur, tf::Transform fin_T, 
    vector<KeyPoint>& v_kp1, vector<KeyPoint>& v_kp2)
{
  vector<bool> ret; 
  ret.reserve(v_kp1.size());
  print_tf(std::cout, fin_T);
  // compute inlier and outlier 
  Undistorter* undistorter = getCalibUndist(); 
  if(undistorter == 0) return ret; 

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

  // compute 
  FrameSE3* pre = new FrameSE3(1, w, h, K, 0., eh_img_pre.data); 
  pre->setDepthFromGroundTruth((float*)(dpt_pre.data));

  FrameSE3* cur = new FrameSE3(2, w, h, K, 0.5, eh_img_cur.data); 
  cur->setDepthFromGroundTruth((float*)(dpt_cur.data));

  Eigen::Matrix4f eigen_T;
  pcl_ros::transformAsMatrix(fin_T, eigen_T);
  Sophus::SE3f referenceToFrame(eigen_T);
  Eigen::Matrix3f rotMat = referenceToFrame.rotationMatrix();
  Eigen::Vector3f transVec = referenceToFrame.translation();

  int inlier_cnt = 0;
  int outlier_cnt = 0; 
  for(int i=0; i<v_kp1.size(); i++)
  {
    Eigen::Vector3f refer_pt, query_pt, query_in_refer_pt;
    int idx1 = v_kp1[i].pt.y * w + v_kp1[i].pt.x; 
    int idx2 = v_kp2[i].pt.y * w + v_kp2[i].pt.x; 
    pre->getPointXYZ(idx1, refer_pt); 
    cur->getPointXYZ(idx2, query_pt); 
    query_in_refer_pt = rotMat * (query_pt) + transVec; 
    Eigen::Vector3f v_err = query_in_refer_pt - refer_pt; 
    float err = v_err.norm(); 
    
    // cout<<"query_pt"<<query_pt<<" refer_pt: "<<refer_pt<<" query_in_refer_pt: "<<query_in_refer_pt;
    // cout<<"error: "<<v_err<<" norm: "<<err<<endl;

    // if(err <= 0.05)
    if(err <= 0.2)
    {
      ++inlier_cnt; 
      ret.push_back(1); 
    }else
    {
      if (err < 1)
      {
        // cout<<"query_pt"<<query_pt<<" refer_pt: "<<refer_pt<<" query_in_refer_pt: "<<query_in_refer_pt;
        // cout<<"error: "<<v_err<<" norm: "<<err<<endl;
      }
      ++outlier_cnt;
      ret.push_back(0);
    }
    if(i<20)
    {
      // printf("main_show_match_points.cpp: idx1: %d idx2: %d refer_pt: %f %f %f query_pt: %f %f %f, query_pt_in_ref: %f %f %f, err %f \n", idx1, idx2, refer_pt(0), refer_pt(1), refer_pt(2), query_pt(0), query_pt(1), query_pt(2), query_in_refer_pt(0), query_in_refer_pt(1), query_in_refer_pt(2), err);
    }
  }
  
  printf("main_show_match_points.cpp: inlier_cnt %d outlier_cnt %d\n", inlier_cnt, outlier_cnt);
  // return inlier_cnt;
  return ret;
}

void generate_error_distribution()
{
  sr_data pre_sr, cur_sr; 
  Mat eh_img_pre, eh_img_cur; 
  Mat dpt_pre, dpt_cur;
  if(!getSRAndImg(pre_sr, eh_img_pre, dpt_pre, cur_sr, eh_img_cur, dpt_cur))
  {
    return ;
  }

  // display transformation 
  string trans_log("");
  ros::NodeHandle nh("~");
  nh.param("transform_log", trans_log, trans_log); 
  tf::Transform ini_T, fin_T;
  if(!getTransform(trans_log, ini_T, fin_T))
  {
    ROS_WARN("main_show_match_points.cpp: no transform available in file %s", trans_log.c_str());
    return ;
  }

  print_tf(std::cout, ini_T);
  print_tf(std::cout, fin_T); 

  Undistorter* undistorter = getCalibUndist(); 
  if(undistorter == 0) return; 

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

  // compute 
  TrackingReferenceSE3* trackingReference = new TrackingReferenceSE3(); 
  FrameSE3* pre = new FrameSE3(1, w, h, K, 0., eh_img_pre.data); 
  pre->setDepthFromGroundTruth((float*)(dpt_pre.data));
  trackingReference->importFrame(pre); 

  FrameSE3* cur = new FrameSE3(2, w, h, K, 0.5, eh_img_cur.data); 
  SE3Tracker* ptrack = new SE3Tracker(w, h, K); 
  Eigen::Matrix4f eigen_T;
  pcl_ros::transformAsMatrix(fin_T, eigen_T);

  SE3 sophus_trans(eigen_T.cast<double>());
  
  tfScalar x,y,z,roll,pitch,yaw; 
  getTF_xyzrpy(fin_T, x, y, z, roll, pitch, yaw); 
  tfScalar t_limit = 0.5; 
  tfScalar t_step = 0.02;
 
  ofstream ouf("./lsd_slam/xyz_error.log");

  float error; 
  for(tfScalar tx = x - t_limit; tx <= x + t_limit; tx += t_step)
    for(tfScalar ty = y - t_limit; ty <= y + t_limit; ty += t_step)
      for(tfScalar tz = z - t_limit; tz <= z + t_limit; tz += t_step)
      {
        fin_T.setOrigin(tf::Vector3(tx, ty, tz)); 
        pcl_ros::transformAsMatrix(fin_T, eigen_T);
        SE3 sophus_trans(eigen_T.cast<double>());
        error = ptrack->checkErrorOnParamref(trackingReference, cur, sophus_trans); 
       
        ouf<<tx<<" "<<ty<<" "<<tz<<" "<<error<<endl;
      }

  printf("main_show_match_points.cpp: error compute finished\n");

  return ;
  
}

void getGTTransform(vector<tf::Transform>& vgt) // these transforms are result from rgbd-SLAM 
{
  ifstream inf("/home/davidz/.ros/publish_3d_2d.log");
  char buf[1024];
  vgt.clear();
  float x,y,z,qx,qy,qz,qw;
  int id;
  inf.getline(buf,1024);  // the first frame 
  // inf.getline(buf,1024);  // the second frame, lsd_slam miss the second frame
  while(inf.getline(buf, 1024))
  {
    sscanf(buf, "%d %f %f %f %f %f %f %f ", &id, &x, &y, &z, &qx, &qy, &qz, &qw); 
    tf::Transform T = tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x,y,z));
    vgt.push_back(T);
  }
}

bool loadKeyPoints(std::string f, vector<KeyPoint>& kp_pre, vector<KeyPoint>& kp_cur, vector<DMatch>& matches, int step)
{
  ifstream inf(f.c_str());
  if(!inf.is_open())
  {
    ROS_ERROR("main_show_match_points.cpp: falied to load match file: %s", f.c_str());
    return false;
  }

  kp_pre.clear(); kp_cur.clear(); matches.clear(); 
  
  char tmp[1024];
  int index = 0; 
  int w = 176;
  int good_or_bad = 0; 
  while(inf.getline(tmp, 1024))
  {
    KeyPoint kp1, kp2; 
    sscanf(tmp, "%f %f %f %f %d", &kp1.pt.x, &kp1.pt.y, &kp2.pt.x, &kp2.pt.y, &good_or_bad);
    // if(good_or_bad == 1) continue;

    if(index == 0 || index%step==0)
    {
      kp_pre.push_back(kp1); 
      kp_cur.push_back(kp2); 
      DMatch m; m.queryIdx = kp_cur.size()-1; m.trainIdx = kp_pre.size()-1; m.imgIdx = (int)(kp1.pt.x + kp1.pt.y*w);
      matches.push_back(m);
    }
    ++index;
  }
  
  ROS_WARN("main_show_match_points.cpp: kp_pre.size = %d kp_cur.size = %d match.size = %d", kp_pre.size(), kp_cur.size(), matches.size()); 

  printf("main_show_match_points.cpp: kp_pre.size = %d kp_cur.size = %d match.size = %d \n", kp_pre.size(), kp_cur.size(), matches.size()); 


  return true; 
}

Undistorter* getCalibUndist()
{
  static Undistorter* undistorter = 0;
  if(undistorter == 0)
  {
    // get calibration parameter 
    std::string calibFile;
    if(ros::param::get("~calib", calibFile))
    {
      ROS_INFO("main_show_match_points.cpp: calibration file is %s", calibFile.c_str());
      undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
      // ros::param::del("~calib");
    }

    if(undistorter == 0)
    {
      printf("need camera calibration file! (set using _calib:=FILE)\n");
    }
  }
  return undistorter; 
}

bool getSRAndImg(sr_data& pre_sr, cv::Mat& eh_img_pre, Mat& rect_dpt_pre, sr_data& cur_sr, Mat& eh_img_cur, Mat& rect_dpt_cur)
{
  Undistorter* undistorter = getCalibUndist();
  if(undistorter == 0) return false;

  // load two sr_data 
  string path_pre(""); 
  string path_cur("");
  ros::NodeHandle nh("~");
  nh.param("previous_frame_path", path_pre, path_pre); 
  nh.param("current_frame_path", path_cur, path_cur);
  
  CSReader r; 
 // sr_data pre_sr, cur_sr; 
  if(!r.readOneFrame(path_pre,pre_sr) || !r.readOneFrame(path_cur, cur_sr))
  {
      ROS_ERROR("main_show_match_points.cpp: failed to load cur_sr_path or pre_sr_path!");
      return false;
  }
  
  // extract imgs from sr_data 
  cv::Mat img_pre, img_cur, dpt_pre, dpt_cur; 
  srCallback(pre_sr, img_pre, dpt_pre); 
  srCallback(cur_sr, img_cur, dpt_cur);

  // rectification 
  cv::Mat rect_img_pre, rect_img_cur; 
  undistorter->undistort(img_pre, rect_img_pre); 
  undistorter->undistort(img_cur, rect_img_cur);
  
  undistorter->undistort(dpt_pre, rect_dpt_pre); 
  undistorter->undistort(dpt_cur, rect_dpt_cur);

  // equalize Histogram 
  // cv::Mat eh_img_pre, eh_img_cur; 
  cv::equalizeHist(rect_img_pre, eh_img_pre); 
  cv::equalizeHist(rect_img_cur, eh_img_cur); 
 
  return true;
}


void display_img(Mat img, string window_name)
{
  cv::namedWindow(window_name.c_str(), 1); 
  cv::imshow(window_name.c_str(), img);
  cv::waitKey(0);
}

void getTF_xyzrpy(tf::Transform tT, tfScalar& tx, tfScalar& ty, tfScalar& tz, tfScalar& roll, tfScalar& pitch, tfScalar& yaw)
{
  tT.getBasis().getEulerYPR(yaw, pitch, roll); 
  tf::Vector3 t = tT.getOrigin(); 
  tx = t.getX(); ty = t.getY(); tz = t.getZ();
  return ;
}

void print_tf(ostream& out, tf::Transform tT)
{
  tfScalar r, p, y, tx, ty, tz;
  getTF_xyzrpy(tT, tx, ty, tz, r, p, y);
  out<<"main_show_match_points.cpp: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<" tx: "<<tx<<" ty: "<<ty<<" tz: "<<tz<<endl;
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
     dpt = cv::Mat(SR_HEIGHT, SR_WIDTH, CV_32FC1, &sr.z_[0]);
  }else
  {
    // reconstruct the sr distance data
    static float dis[SR_SIZE] = {0};
    for(int i=0; i<SR_SIZE; i++)
    {
      dis[i] = (SR_IMG_TYPE)(sr.dis_[i]*0.001);
    }

    dpt = cv::Mat(SR_HEIGHT, SR_WIDTH, CV_32FC1, dis);
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

bool getTransform(string trans_log, tf::Transform& ini_T, tf::Transform& fin_T)
{
  ifstream inf(trans_log.c_str());
  if(!inf.is_open())
  {
    return false; 
  }
  
  float x,y,z, qx,qy,qz,qw; 
  char buf[1024];
  // first, final transformation 
  inf.getline(buf, 1024); 
  printf("main_show_match_points.cpp: first line %s\n", buf);
  sscanf(buf, "%f %f %f %f %f %f %f", &x, &y, &z, &qx, &qy, &qz, &qw); 
  fin_T = tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x,y,z));

  // second, initial transformation 
  inf.getline(buf, 1024); 
  printf("main_show_match_points.cpp: second line %s\n", buf);
  sscanf(buf, "%f %f %f %f %f %f %f", &x, &y, &z, &qx, &qy, &qz, &qw); 
  ini_T = tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x,y,z));
  
  return true;
}


