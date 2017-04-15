/*
 *  Oct. 31, 2015 David Z
 *
 *  A test file for VRO using realsense 
 *
 * */


#include <ros/ros.h>
#include <string>
#include "graph_plane.h"
#include "plane_node.h"
#include "global_def.h"
#include "../sr_publish/realsense_reader.h"

using namespace std;

void print_tf(ostream& out, tf::Transform tT);
void test_vo_rs();
void test_vo_seq();

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_vo_rs");
  ros::NodeHandle n;

  string vo_model; 
  ros::NodeHandle nh("~"); 
  nh.param("vo_rs_model", vo_model, string("single_match")); 
  if(vo_model == "single_match")
  {
    test_vo_rs();
  }else if(vo_model == "sequence_match")
  {
    test_vo_seq();
  }
  return 0; 
}

void test_vo_seq()
{
  CRSReader rs_reader; 
  rs_data src_rs_d, tar_rs_d; 
  int rs_start_id = rs_reader.start_frame_; 
  int rs_end_id = rs_reader.end_frame_; 
  int rs_data_strip ; 
  ros::NodeHandle nh_p("~"); 
  nh_p.param<int>("rs_data_strip", rs_data_strip, 1);

  ofstream ouf("./rs_slam/vo_rs_seq.log");
  ofstream ouf_inc("./rs_slam/inc_result_2.log");
  Node* pTar = 0; 
  Node* pSrc = 0;
  tf::Transform cur_T; 
  CGraphPlane gp;

  for(int i= rs_start_id; i<rs_end_id; i+= rs_data_strip)
  {
    tf::Transform T;
    if(!rs_reader.readRSID(i, src_rs_d))
    {
      ROS_ERROR("test_vo_rs.cpp: failed to load id: %d", i); 
      return; 
    }
    pSrc = gp.fromRS(src_rs_d); 

    if(pTar == 0) // first node 
    {
      pTar = pSrc;
      cur_T = tf::Transform::getIdentity();
      print_tf(ouf, cur_T);
      continue;
    }

    if(!gp.VRO(pTar, pSrc, T)) // VRO failed
    {
      ROS_ERROR("test_vo_rs.cpp: what? VRO failed!");
      delete pSrc;
      continue;
    }else
    {
      delete pTar;
      pTar = pSrc;
      cur_T = cur_T*T;
      print_tf(ouf, cur_T);
      print_tf(ouf_inc, T);
      print_tf(std::cout, T);
    }
  }
  ouf.close();
  return ;
}

void test_vo_rs()
{
  string path_tar("/home/davidz/work/data/SwissRanger4000/vro_results/gt/dataset_40");
  string path_src("/home/davidz/work/data/SwissRanger4000/vro_results/gt/dataset_41");

  int tar_id = 10; 
  int src_id = 10; 

  int times = 1;
  ros::NodeHandle nh("~");
  nh.param("vro_src_node_path", path_src, path_src); 
  nh.param("vro_tar_node_path", path_tar, path_tar);
  nh.param("vro_src_id", src_id, src_id);
  nh.param("vro_tar_id", tar_id, tar_id);
  nh.param("vro_single_times", times, 1); 

  CRSReader rs_reader; 
  rs_data src_rs_d, tar_rs_d; 
  if(!rs_reader.readRSID(tar_id, tar_rs_d))
  {
    ROS_ERROR("test_vo_rs.cpp: failed to load id: %d", tar_id); 
    return; 
  }
  if(!rs_reader.readRSID(src_id, src_rs_d))
  {
    ROS_ERROR("test_vo_rs.cpp: failed to load id: %d", src_id);
    return ;
  }

  CGraphPlane gp;
  Node* pTar = gp.fromRS(tar_rs_d); 
  Node* pSrc = gp.fromRS(src_rs_d); 

  for(int i=0; i<times; i++)
  {
    tf::Transform T; 
    // testTrans(path_tar, tar_id, path_src, src_id, T);
    // if(!((CGraphPlane*)0)->VRO(path_tar, tar_id, path_src, src_id, T))
    if(!gp.VRO(pTar, pSrc, T))
    {
      ROS_ERROR("test_vro.cpp: what? VRO failed!");
    }else
    {
      print_tf(std::cout, T);
    }
  }
  return ;
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



