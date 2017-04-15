/*
 * Aug. 9, 2016 David Z
 * 
 * Read ground truth data, get the result 
 *
 * */

#include <iostream>
#include <ros/ros.h>
#include "graph_plane.h"
#include "plane_node.h"
#include <tf/tf.h>
#include "../sr_publish/SR_reader.h"
// pcl 
// #include "../sr_plane/plane_set.h"
#include "../sr_plane/vtk_viewer.h"
#include "../sr_plane/plane.h"
#include <pcl/io/pcd_io.h>
#include "pcl_ros/transforms.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/transforms.h>

#define CONVERT(a,b) (((a)*(b)<0)?-a:a)

template<typename PointT>
void filterPointCloud(typename pcl::PointCloud<PointT>::Ptr& in,
    typename pcl::PointCloud<PointT>::Ptr& out, double _voxel_size = 0.01);

void print_tf(char* name, tf::Transform t);
void test_gt(); 
void test_gt2();
void test_gt3();
bool readFromGT(vector<double>& tv, vector<vector<float> >& pv, string f);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_gt"); 
  ros::NodeHandle nh;
  // test_gt();
  // test_gt2();
  test_gt3();

  return 1;

}


void test_gt3()
{
  ros::NodeHandle nh("~"); 
  string gt_file = ""; 
  string gt_file_cp = ""; 
  nh.param("gt_file", gt_file, gt_file); 
  nh.param("gt_file_cp", gt_file_cp, gt_file_cp); 

  vector<double> ts1, ts2; 
  vector<vector<float> > vp1, vp2;

  if(!readFromGT(ts1, vp1, gt_file))
  {
    ROS_ERROR("test_gt.cpp: failed to load gt1 : %s", gt_file.c_str());
    return ; 
  }

  if(!readFromGT(ts2, vp2, gt_file_cp))
  {
    ROS_ERROR("test_gt.cpp: failed to load gt2 : %s", gt_file_cp.c_str());
    return ;
  }
  
  tf::Transform Tini1 = tf::Transform(tf::Quaternion(vp1[0][3], vp1[0][4], vp1[0][5], vp1[0][6]), 
      tf::Vector3(vp1[0][0], vp1[0][1], vp1[0][2])); 
  tf::Transform Tini2 = tf::Transform(tf::Quaternion(vp2[0][3], vp2[0][4], vp2[0][5], vp2[0][6]), 
      tf::Vector3(vp2[0][0], vp2[0][1], vp2[0][2])); 

  pointcloud_type::Ptr g_pc(new pointcloud_type());

  string pc_dir = "/home/davidz/work/data/SwissRanger4000/bdat/result/dataset_3/comp_lsd_vo";
  tf::Transform deltaT = tf::Transform(tf::Quaternion(0.01, 0.05, 0.01, 0.9985), tf::Vector3(0.2, 0.01, -0.2));
  for(int i=1, k=-1 ;i<ts1.size() && i < 2000; i++)
  {
    double timestamp = ts1[i]; 
    for(int j=1; j<ts2.size(); j++)
    {
      if(fabs(timestamp - ts2[j]) < 0.01)
      {
        tf::Transform Tcur1 = tf::Transform(tf::Quaternion(vp1[i][3], vp1[i][4], vp1[i][5], vp1[i][6]), 
            tf::Vector3(vp1[i][0], vp1[i][1], vp1[i][2])); 

        // due to different initial reference
        // vp2[j][3] = CONVERT(vp2[j][3], vp1[i][3]); 
        // vp2[j][4] = CONVERT(vp2[j][4], vp1[i][4]);
        // vp2[j][5] = CONVERT(vp2[j][5], vp1[i][5]);
        // vp2[j][0] = CONVERT(vp2[j][0], vp1[i][0]); 
        // vp2[j][1] = CONVERT(vp2[j][1], vp1[i][1]);

        tf::Transform Tcur2 = tf::Transform(tf::Quaternion(vp2[j][3], vp2[j][4], vp2[j][5], vp2[j][6]), 
             tf::Vector3(vp2[j][0], vp2[j][1], vp2[j][2])); 
        // tf::Transform Tcur2 = tf::Transform(tf::Quaternion(vp1[i][3], vp1[i][4], vp1[i][5], vp1[i][6]), 
         //   tf::Vector3(vp2[j][0], vp2[j][1], vp2[j][2])); 

        print_tf("test_gt.cpp: vo_T: ", Tcur1); 
        print_tf("test_gt.cpp: gt_T: ", Tcur2);
        tf::Transform dT12 = Tcur1.inverse()*Tcur2; 
        print_tf("test_gt.cpp: diff_T: dT12", dT12);
        // print_tf("tesg_gt.cpp: gt_T_inverse: ", Tcur2.inverse());
        tf::Transform T = Tcur2; // Tini1*(Tini2.inverse()*Tcur2); 

        Eigen::Matrix4f eT; 
        pcl_ros::transformAsMatrix(T, eT); 

        if(++k%5 == 0)
        {
          // 2. get pc 
          stringstream ss; 
          ss<<pc_dir<<"/pcs/quicksave_"<<setfill('0')<<setw(4)<<i<<".pcd"; 
          pointcloud_type::Ptr t_pc(new pointcloud_type());
          if(pcl::io::loadPCDFile(ss.str(), *t_pc) == 0)
          {
            pointcloud_type::Ptr dt_pc(new pointcloud_type()); 
            filterPointCloud<point_type>(t_pc, dt_pc, 0.02); 
            pcl::transformPointCloud(*dt_pc, *t_pc, eT); 
            *g_pc = *g_pc + *t_pc;
          }else
          {
            ROS_ERROR("test_gt.cpp: failed to load PCD %s", ss.str().c_str());
          }
        }

      }
    }
  }
  // 3. show it 
  pointcloud_type::Ptr dg_pc(new pointcloud_type());
  filterPointCloud<point_type>(g_pc, dg_pc, 0.05);

  // show the final result 
  CVTKViewer<point_type> viewer;
  viewer.getViewer()->addCoordinateSystem(0.2, 0,0,0);
  viewer.addPointCloud(dg_pc, "planes"); 

  while(!viewer.stopped())
  {
    viewer.runOnce(); 
    usleep(100000);
  }

  return ;
}

void test_gt2()
{
  ros::NodeHandle nh("~"); 
  string gt_file = ""; 
  nh.param("gt_file", gt_file, gt_file); 
  
  ifstream inf(gt_file.c_str());
  if(!inf.is_open())
  {
    ROS_ERROR("test_gt.cpp: failed to open gt file %s ", gt_file.c_str()); 
    return; 
  }
  char buf[1024] = {0};
  int i = 0; 
  double timestamp; 
  float x,y,z,qx,qy,qz,qw;

  pointcloud_type::Ptr g_pc(new pointcloud_type());
  string pc_dir = "/home/davidz/work/data/SwissRanger4000/bdat/result/dataset_3/comp_lsd_vo";
  int k = -1;

  while(inf.getline(buf, 1024))
  {
    // 1. get pc pose
    sscanf(buf, "%lf %f %f %f %f %f %f %f", &timestamp, &x, &y, &z, &qx, &qy, &qz, &qw); 
    tf::Transform T = tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x, y, z)); 
    Eigen::Matrix4f eT; 
    pcl_ros::transformAsMatrix(T, eT); 
    
    if(++k%5 == 0)
    {
      // 2. get pc 
      stringstream ss; 
      ss<<pc_dir<<"/pcs/quicksave_"<<setfill('0')<<setw(4)<<i<<".pcd"; 
      pointcloud_type::Ptr t_pc(new pointcloud_type());
      if(pcl::io::loadPCDFile(ss.str(), *t_pc) == 0)
      {
        pointcloud_type::Ptr dt_pc(new pointcloud_type()); 
        filterPointCloud<point_type>(t_pc, dt_pc, 0.02); 
        pcl::transformPointCloud(*dt_pc, *t_pc, eT); 
        *g_pc = *g_pc + *t_pc;
      }else
      {
        ROS_ERROR("test_gt.cpp: failed to load PCD %s", ss.str().c_str());
      }
    }
    ++i; 
  }

  // 3. show it 
  pointcloud_type::Ptr dg_pc(new pointcloud_type());
  filterPointCloud<point_type>(g_pc, dg_pc, 0.05);

  // show the final result 
  CVTKViewer<point_type> viewer;
  viewer.getViewer()->addCoordinateSystem(0.2, 0,0,0);
  viewer.addPointCloud(dg_pc, "planes"); 

  while(!viewer.stopped())
  {
    viewer.runOnce(); 
    usleep(100000);
  }
  return ;
}

void test_gt()
{
  ros::NodeHandle nh("~"); 
  string gt_file = ""; 
  nh.param("gt_file", gt_file, gt_file); 

  CSReader r; 
  if(!r.loadAllData() || !r.synFromGT(gt_file))
  {
    ROS_ERROR("test_gt.cpp: failed to loadAllData or synFromGT, byebye");
    ROS_ERROR("test_gt.cpp: gt_file:%s", gt_file.c_str());
    return ;
  }
  
  CGraphPlane g; 

  bool finished = false;
  bool first = true;
  Eigen::Matrix4f pre_T = Eigen::Matrix4f::Identity();  
  pointcloud_type::Ptr g_pc(new pointcloud_type());
  Eigen::Matrix4f cur_T, inc_T, new_T; 
  
  int k = -1;

  while(!finished)
  {
    sr_data sr_frame = r.get_current_frame(finished);
    while(!sr_frame.b_gt_has_been_set_)
    {
      sr_frame = r.get_current_frame(finished); 
      if(finished) break; 
    }
    if(finished) break; 

    CNodeWrapper* n = (CNodeWrapper*)(g.fromSR(sr_frame)); 
    pcl_ros::transformAsMatrix(n->gt_T_, cur_T); 

    if(first)
    {
      pre_T = cur_T; 
      // new_T = Eigen::Matrix4f::Identity();
      new_T = pre_T; 
      first = false; 
      print_tf("first_pose: ", n->gt_T_);
    }else
    {
      inc_T = pre_T.inverse()*cur_T; 
      new_T = new_T*inc_T; 
      tf::Transform my_tf = eigenTransf2TF(new_T); 
      print_tf("cur_pose: ", n->gt_T_); 
      print_tf("my_cur_pose: ", my_tf);
      pre_T = cur_T; 
    }
    
    if(++k%5==0)
    { 
      pointcloud_type::Ptr p_pc(new pointcloud_type());
      pointcloud_type::Ptr d_pc(new pointcloud_type());

      filterPointCloud<point_type>(n->pc_col, d_pc, 0.03);
      pcl::transformPointCloud(*d_pc, *p_pc, new_T); 
      *g_pc = *g_pc + *p_pc;

      ROS_WARN("test_gt.cpp: g_pc has points : %d", g_pc->size());
    }
    delete n; 
  }
  
  // 3. show it 
  pointcloud_type::Ptr dg_pc(new pointcloud_type());
  filterPointCloud<point_type>(g_pc, dg_pc, 0.05);

  // show the final result 
  CVTKViewer<point_type> viewer;
  viewer.getViewer()->addCoordinateSystem(0.2, 0,0,0);
  viewer.addPointCloud(dg_pc, "planes"); 

  while(!viewer.stopped())
  {
    viewer.runOnce(); 
    usleep(100000);
  }
  return ;
}


void print_tf(char* name, tf::Transform t)
{
  ROS_INFO_STREAM(name << ": Translation " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z());
  ROS_INFO_STREAM(name << ": Rotation " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW());
}

template<typename PointT>
void filterPointCloud(typename pcl::PointCloud<PointT>::Ptr& in,typename pcl::PointCloud<PointT>::Ptr& out, double _voxel_size)
{
    // typename pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
    // passthrough filter
    // pcl::PassThrough<PointT > pass;
    // pass.setInputCloud(in);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.0, _depth_limit);
    // pass.filter(*tmp);
    // g_passFilter<PointT>(in, tmp, 0.0, _depth_limit, "z");

    // voxelgrid filter
    pcl::VoxelGrid<PointT> vog;
    vog.setInputCloud(in); 
    vog.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
    vog.filter(*out);
}

bool readFromGT(vector<double>& tv, vector<vector<float> >& pv, string f)
{
  ifstream inf(f.c_str());
  if(!inf.is_open())
  {
    ROS_ERROR("SR_reader.cpp: failed to get GT from file %s", f.c_str());
    return false; 
  }
  const int N = 1024; 
  tv.reserve(N); 
  pv.reserve(N); 

  char buf[N] = {0};
  double timestamp; 
  float x,y,z,qx,qy,qz,qw;
 
  vector<float> p(7); 
  while(inf.getline(buf, N))
  {
    sscanf(buf, "%lf %f %f %f %f %f %f %f", &timestamp, &x, &y, &z, &qx, &qy, &qz, &qw); 
    tv.push_back(timestamp); 
    p[0] = x; p[1] = y; p[2] = z; p[3] = qx; p[4] = qy; p[5] = qz; p[6] = qw; 
    pv.push_back(p); 
  }
 return true;
}


