/*
 *  Oct. 1, 2015 David Z
 *
 *  A test file for VRO 
 *
 * */


#include <ros/ros.h>
#include <string>
#include "graph_plane.h"
#include "plane_node.h"
#include "global_def.h"
#include "../sr_plane/mean_sigma.h"
#include "../sr_publish/SR_reader.h"
#include "../sr_plane/vtk_viewer.h"
#include "../sr_plane/glob_def.h"
#include "pcl_ros/transforms.h"
#include <ros/console.h>

using namespace std;

// #define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG

// template<typename T>
// extern bool compute_mu_sigma(T* in, int N, T& mu, T& sigma);
// extern bool compute_mu_sigma(tfScalar* in, int N, tfScalar& mu, tfScalar& sigma);

template<typename T>
int zero_filter(vector<T>&, vector<T>&, vector<T>& , vector<T>& , vector<T>& , vector<T>&);
void testVRO(); // single frame
void testVRO_pack(); // many frames

void decompose_tf(tf::Transform tT, tfScalar &r, tfScalar& p, tfScalar& y, tfScalar& tx, tfScalar& ty, tfScalar& tz);
void print_tf(ostream& out, tf::Transform t);
void init_parameters();   // for doing this test, we have to set several parameters 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_vro");
  ros::NodeHandle n;
  
  /* do not work
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ROS_INFO("test_vro.cpp: into here!?");
    ros::console::notifyLoggerLevelsChanged();
  }

  // log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  // my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  const char* logger_name = ROSCONSOLE_DEFAULT_NAME ".<name>";
  ROS_WARN("test_vro.cpp: logger_name %s", logger_name);
  ROS_DEBUG("test_vro.cpp: can you show me ?");
  */

  init_parameters();
  
  ros::NodeHandle nh("~");
  bool b_vro_single_match; 
  nh.param("vro_single_match", b_vro_single_match, true);
  if(b_vro_single_match)
  {
    ROS_WARN("test_vro.cpp: vro single match!");
    testVRO();
  }else
  {
    ROS_WARN("test_vro.cpp: vro pack match!");
    testVRO_pack();   // multiple matches
  }

  ROS_WARN("test_vro.cpp: finish testVRO");
  return 0;
}

void testVRO_pack()
{
  string path_tar("/home/davidz/work/data/SwissRanger4000/vro_results/gt/dataset_40");
  string path_src("/home/davidz/work/data/SwissRanger4000/vro_results/gt/dataset_41");
  
  ros::NodeHandle nh("~");
  nh.param("vro_src_node_path", path_src, path_src); 
  nh.param("vro_tar_node_path", path_tar, path_tar);
  
  tf::Transform T; 
  int N = 100; 
  nh.param("vro_pack_number", N, N);
  vector<tfScalar> er(N);
  vector<tfScalar> ey(N); 
  vector<tfScalar> ep(N);
  vector<tfScalar> etx(N);
  vector<tfScalar> ety(N);
  vector<tfScalar> etz(N);
  vector<tfScalar> e_t(N); 
  vector<tfScalar> e_r(N);

  double gt_tx; 
  double gt_pitch; 
  double gt_yaw; 
  nh.param("gt_tx", gt_tx, 0.); 
  nh.param("gt_pitch", gt_pitch, 0.);
  nh.param("gt_yaw", gt_yaw, 0.);

  int tar_id, src_id;
  string match_result_output("match_vro_result.log");
  string statistic_result_output("statistic_result.log");
  nh.param("vro_match_result_log", match_result_output, match_result_output); 
  nh.param("vro_statistic_result_log", statistic_result_output, statistic_result_output);
  // ofstream ouf("match_vro_result.log");
  ofstream ouf(match_result_output.c_str());
  for(int i = 0; i < N; i++)
  {
    src_id = tar_id = i+1;
    if(!((CGraphPlane*)0)->VRO(path_tar, tar_id, path_src, src_id, T))
    {
      ROS_ERROR("test_vro.cpp: what? VRO failed!");
    }else
    {
      // print_tf(T);
      print_tf(ouf, T);
      decompose_tf(T, er[i], ep[i], ey[i], etx[i], ety[i], etz[i]);
      e_t[i] = sqrt(SQ(etx[i] - gt_tx) + SQ(ety[i]) + SQ(etz[i])); 
      e_r[i] = sqrt(SQ(er[i]) + SQ(ep[i] - gt_pitch) + SQ(ey[i] - gt_yaw));
    }
  }
  
  int n_failed = zero_filter<tfScalar>(er, ep, ey, etx, ety, etz);
  ROS_ERROR("test_vro.cpp: pack all %d failed %d, fail ratio: %f", N, n_failed, (double)n_failed/(double)N);
  N = er.size();

  // compute mean and sigma
  double mu, sigma; 
  // ofstream f("statistic_result.log");
  ofstream f(statistic_result_output.c_str());
  compute_mu_sigma(er.data(), N, mu, sigma); 
  f<<mu<<" "<<sigma;
  compute_mu_sigma(ep.data(), N, mu, sigma);
  f<<" "<<mu<<" "<<sigma;
  compute_mu_sigma(ey.data(), N, mu, sigma); 
  f<<" "<<mu<<" "<<sigma;
  compute_mu_sigma(etx.data(), N, mu, sigma);
  f<<" "<<mu<<" "<<sigma;
  compute_mu_sigma(ety.data(), N, mu, sigma); 
  f<<" "<<mu<<" "<<sigma;
  compute_mu_sigma(etz.data(), N, mu, sigma);
  f<<" "<<mu<<" "<<sigma<<endl;
  compute_mu_sigma(e_t.data(), N, mu, sigma); 
  f<<"et: "<< mu<<" "<<sigma<<endl;
  compute_mu_sigma(e_r.data(), N, mu, sigma); 
  f<<"er: "<< mu<<" "<<sigma<<endl;

  return ;
}

void testTrans(string path_tar, int tar_id, string path_src, int src_id, tf::Transform& T)
{
  CPlaneNode* n_tar = new CPlaneNode(path_tar, tar_id); 
  CPlaneNode* n_src = new CPlaneNode(path_src, src_id);
  Eigen::Matrix4f eigen_T;
  n_src->testTrans(n_tar, eigen_T);
  // n_src->testTrans2(n_tar, eigen_T);
  T = eigenTransf2TF(eigen_T);
  cout<<"test_vro.cpp: eigen_T "<<eigen_T<<endl;
  print_tf(std::cout, T);
  return;
}

void testVRO()
{
  string path_tar("/home/davidz/work/data/SwissRanger4000/vro_results/gt/dataset_40");
  string path_src("/home/davidz/work/data/SwissRanger4000/vro_results/gt/dataset_41");

  int tar_id = 10; 
  int src_id = 10; 

  int times = 1;
  bool b_node_from_sr_data = false; 
  ros::NodeHandle nh("~");
  nh.param("vro_src_node_path", path_src, path_src); 
  nh.param("vro_tar_node_path", path_tar, path_tar);
  nh.param("vro_src_id", src_id, src_id);
  nh.param("vro_tar_id", tar_id, tar_id);
  nh.param("vro_single_times", times, 1); 
  nh.param("vro_node_from_sr_data", b_node_from_sr_data, false);

  CSReader reader; 
  sr_data src_data, tar_data;
  if(b_node_from_sr_data)
  {
    ROS_WARN("test_vro.cpp: read node from sr_data!");
    if(!reader.readOneFrame(path_src, src_data))
    {
      ROS_ERROR("test_vro.cpp: failed to load src_data: %s", path_src.c_str());
      return ;
    }
    if(!reader.readOneFrame(path_tar, tar_data))
    {
      ROS_ERROR("test_vro.cpp: failed to load tar_data: %s", path_tar.c_str());
      return ;
    }
  }

  CGraphPlane * gp = new CGraphPlane();
  tf::Transform T; 

  for(int i=0; i<times; i++)
  {
    // testTrans(path_tar, tar_id, path_src, src_id, T);
    if(!b_node_from_sr_data)
    {
      if(!gp->VRO(path_tar, tar_id, path_src, src_id, T))
      {
        ROS_ERROR("test_vro.cpp: what? VRO failed!");
      }else
      {
        print_tf(std::cout, T);
      }
    }else
    {
      // ROS_INFO("arrive here before VRO()");
      if(!gp->VRO(tar_data, src_data, T))
      {
        ROS_ERROR("test_vro.cpp: what? VRO failed!");
      }else
      {
        print_tf(std::cout, T);
      }
    }
  }

  // whether display point cloud 
  bool b_display_pc = false; 
  nh.param("show_point_cloud", b_display_pc, b_display_pc);
  if(b_display_pc)
  {
    CPlaneNode* old_node = gp->fromSR(tar_data); 
    CPlaneNode* new_node = gp->fromSR(src_data); 
    
    pointcloud_type::Ptr old_pc(new pointcloud_type); 
    pointcloud_type::Ptr new_pc(new pointcloud_type); 
    pointcloud_type::Ptr new_pc_in_old(new pointcloud_type); 
    // old_pc = old_node->pc_col.make_shared();
    // new_pc = new_node->pc_col.make_shared();
    
    ROS_INFO("main_show_match_points.cpp: old_pc has %d, new_pc has %d", old_node->pc_col->points.size(), new_node->pc_col->points.size());
  
    Eigen::Matrix4f eigen_T; 
    pcl_ros::transformAsMatrix(T, eigen_T);

    pcl::transformPointCloud(*(new_node->pc_col), *new_pc_in_old, eigen_T);
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

void decompose_tf(tf::Transform tT, tfScalar &r, tfScalar& p, tfScalar& y, tfScalar& tx, tfScalar& ty, tfScalar& tz)
{
  double ry, rp, rr;
  tT.getBasis().getEulerYPR(ry, rp, rr); 
  tf::Vector3 t = tT.getOrigin(); 
  tx = t.getX(); ty = t.getY(); tz = t.getZ();

  // transform from the camera coordinate reference, to the robot coordinate reference
  y = R2D(rp); p = R2D(rr); r = R2D(ry); 
 return;
}

void print_tf(ostream& out, tf::Transform tT)
{
  tfScalar r, p, y, tx, ty, tz;
  tT.getBasis().getEulerYPR(y, p, r); 
  tf::Vector3 t = tT.getOrigin(); 
  tx = t.getX(); ty = t.getY(); tz = t.getZ();
  out<<"test_vro: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<" tx: "<<tx<<" ty: "<<ty<<" tz: "<<tz<<" qx = "<<
    tT.getRotation().x()<<" qy = "<<tT.getRotation().y()<<" qz= "<<tT.getRotation().z()<<" qw = "<<tT.getRotation().w()<<endl;
}


void init_parameters()
{
  // some parameters for VRO 
  ParameterServer* ps = ParameterServer::instance(); 
  // ps->set<std::string>("feature_detector_type", string("SIFTGPU"));
  // ps->set<std::string>("feature_extrector_type", string("SIFTGPU"));
  // ps->set<int>("min_matches",3); 
  // ps->set<int>("min_keypoints", 10);
  // ps->set<double>("max_dist_for_inliers", 1.); // 3. 0.2
  ps->set<bool>("use_gui", false);
  ps->set<double>("sigma_depth", 0.01); //

}

template<typename T>
int zero_filter(vector<T>& er, vector<T>& ep, vector<T>& ey, vector<T>& tx, vector<T>& ty, vector<T>& tz)
{
  int M = er.size();
  vector<T> a1, a2, a3, a4 ,a5, a6; 
  a1.reserve(M); 
  a2.reserve(M); 
  a3.reserve(M); 
  a4.reserve(M); 
  a5.reserve(M); 
  a6.reserve(M); 
  int n_zeros = 0;

  int C = 0;
  for(int i=0; i<M; i++)
  {
    if(er[i] == 0 && ep[i] == 0 && ey[i] == 0 && \
        tx[i] == 0 && ty[i] == 0 && tz[i] == 0)
    {
      ++ n_zeros;
      continue;
    }
    ++ C;
    a1.push_back(er[i]); a2.push_back(ep[i]); a3.push_back(ey[i]);
    a4.push_back(tx[i]); a5.push_back(ty[i]); a6.push_back(tz[i]);
  }

  er.swap(a1); ep.swap(a2); ey.swap(a3); 
  tx.swap(a4); ty.swap(a5); tz.swap(a6);
  er.resize(C); ep.resize(C); ey.resize(C); 
  tx.resize(C); ty.resize(C); tz.resize(C);

  return n_zeros;
}

