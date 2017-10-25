/*
 *  Mar. 11, 2017 David Z
 *
 *  A test file reading imgs and run VRO 
 *
 * */


#include <ros/ros.h>
#include <string>
#include <sstream>
#include "graph_plane.h"
#include "plane_node.h"
#include "global_def.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../sr_plane/vtk_viewer.h"
#include "../sr_plane/glob_def.h"
#include "pcl_ros/transforms.h"
#include "cam_model.h"

using namespace std; 
using namespace cv; 

bool loadImg(string dir, int id, cv::Mat& rgb, cv::Mat& dpt);
void matchTwoFrames(); 
void showCloud(); 
void generatePC( pointcloud_type::Ptr& pc, Mat& rgb, Mat& dpt, CamModel& cam);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_vo_img");
  ros::NodeHandle n;

  matchTwoFrames(); 
  // showCloud(); 

  return 0; 
}

void showCloud()
{
  string dir("/media/work/work/data/realsense/Q_desk_loop");
  int from_nj, to_ni; 
  ros::NodeHandle nh("~"); 
  nh.param<int>("img_from", from_nj, 17);
  nh.param<int>("img_to", to_ni, 15); 
  nh.param<string>("img_dir", dir, dir); 
  CGraphPlane gp;
 
  // load rgb and dpt 
  cv::Mat rgb_ni, dpt_ni; 
  cv::Mat rgb_nj, dpt_nj; 
  if(!loadImg(dir, from_nj, rgb_nj, dpt_nj))
  {
    ROS_ERROR("%s failed to load img_nj = %d", __FILE__, from_nj); 
    return ; 
  }

  // generate the camera models 
  // CamModel cam_info(606.508/2, 607.075/2, 316.00/2, 244.682/2, 0.11064, -0.55174);
  CamModel cam_info(606.508, 607.075, 316.00, 244.682, 0.11064, -0.55174);
  cam_info.width = 640; // 320 
  cam_info.height = 480;  // 240

  // Node* pNi = gp.fromRGBD(rgb_ni, dpt_ni, cam_info); 
  Node* pNj = gp.fromRGBD(rgb_nj, dpt_nj, cam_info); 

  pointcloud_type::Ptr old_pc(new pointcloud_type); 

  // generatePC(old_pc, rgb_nj, dpt_nj, cam_info); 

  // ROS_INFO("test_vo_img.cpp: pNj_pc has %d points", pNj->pc_col->points.size());

  *old_pc += *(pNj->pc_col);

   CVTKViewer<point_type> viewer; 
   viewer.getViewer()->addCoordinateSystem(0.2,0, 0, 0); 
   viewer.addPointCloud(old_pc, "final_pcs");
   while(!viewer.stopped())
   {
     viewer.runOnce(); 
     usleep(100000);
   }

  return ; 
}

void matchTwoFrames()
{
  string dir("/media/work/work/data/realsense/Q_desk_loop");
  int from_nj, to_ni; 
  ros::NodeHandle nh("~"); 
  nh.param<int>("img_from", from_nj, 17);
  nh.param<int>("img_to", to_ni, 15); 
  nh.param<string>("img_dir", dir, dir); 
  CGraphPlane gp;
 
  // load rgb and dpt 
  cv::Mat rgb_ni, dpt_ni; 
  cv::Mat rgb_nj, dpt_nj; 
  if(!loadImg(dir, from_nj, rgb_nj, dpt_nj))
  {
    ROS_ERROR("%s failed to load img_nj = %d", __FILE__, from_nj); 
    return ; 
  }

  if(!loadImg(dir, to_ni, rgb_ni, dpt_ni))
  {
    ROS_ERROR("%s failed to load img_ni = %d", __FILE__, to_ni); 
    return ;
  }

  // generate the camera models 
  // CamModel cam_info(606.508/2, 607.075/2, 316.00/2, 244.682/2, 0.11064, -0.55174);
  // cam_info.width = 320; 
  // cam_info.height = 240; 

  // CamModel cam_info(606.508/2, 607.075/2, 316.00/2, 244.682/2, 0.11064, -0.55174);
  // CamModel cam_info(606.508, 607.075, 316.00, 244.682, 0.11064, -0.55174);
  
  CamModel cam_info(581.902, 581.902, 319.5, 239.5); 
  cam_info.width = 640; // 320 
  cam_info.height = 480;  // 240

  Node* pNi = gp.fromRGBD(rgb_ni, dpt_ni, cam_info); 
  Node* pNj = gp.fromRGBD(rgb_nj, dpt_nj, cam_info); 
  
  tf::Transform T;

  ROS_INFO("Before VRO !");
  if(!gp.VRO(pNi, pNj, T))
  {
    ROS_ERROR("VRO failed from nj = %d to ni = %d !", from_nj, to_ni); 
  }else
  {
    ROS_WARN("VRO succeed from nj = %d to ni = %d !", from_nj, to_ni); 
  }

  // show the result  
  // whether display point cloud 
  bool b_display_pc = false; 
  nh.param("show_point_cloud", b_display_pc, b_display_pc);
  if(b_display_pc)
  {
    // CPlaneNode* old_node = gp->fromSR(tar_data); 
    // CPlaneNode* new_node = gp->fromSR(src_data); 
    
    pointcloud_type::Ptr old_pc(new pointcloud_type); 
    pointcloud_type::Ptr new_pc(new pointcloud_type); 
    pointcloud_type::Ptr new_pc_in_old(new pointcloud_type); 
    // old_pc = old_node->pc_col.make_shared();
    // new_pc = new_node->pc_col.make_shared();
    
    ROS_INFO("test_vo_img.cpp: pNi_pc has %d, pNj_pc has %d", pNi->pc_col->points.size(), pNj->pc_col->points.size());
  
    Eigen::Matrix4f eigen_T; 
    pcl_ros::transformAsMatrix(T, eigen_T);

    pcl::transformPointCloud(*(pNj->pc_col), *new_pc_in_old, eigen_T);
    markColor(*new_pc_in_old, RED); 
    markColor(*(pNi->pc_col), GREEN); 
    *old_pc += *new_pc_in_old;
    *old_pc += *(pNi->pc_col);
    
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
void generatePC( pointcloud_type::Ptr& pc, Mat& rgb, Mat& depth, CamModel& cam)
{
    // pointcloud_type::Ptr new_pc(new pointcloud_type); 
    // PointCloud::Ptr cloud ( new PointCloud );
    // 遍历深度图
    //
    double camera_factor = 0.001; 
    double camera_fx = 581.902; // 316.00314 ; 
    double camera_fy = 581.902; // 244.68197 ; 
    double camera_cx = 319.5; // 606.50798 ; 
    double camera_cy = 239.5; // 607.07558 ; 
    double dx, dy, dz; 
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            // PointT p;
            point_type p; 

            // 计算这个点的空间坐标
            p.z = double(d) * camera_factor;
            dz = p.z; 

            cam.convertUVZ2XYZ(n, m, dz, dx, dy, dz); 
            
            p.x = dx; p.y = dy; p.z = dz; 

            // p.x = (n - camera_cx) * p.z / camera_fx;
            // p.y = (m - camera_cy) * p.z / camera_fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            pc->points.push_back( p );
        }
    // 设置并保存点云
    pc->height = 1;
    pc->width = pc->points.size();
return ; 
}

bool loadImg(string dir, int id, cv::Mat& rgb, cv::Mat& dpt)
{
  stringstream ss_rgb, ss_dpt; 
  ss_rgb << dir<<"/color/"<<setfill('0')<<setw(6)<<id<<".png";
  ss_dpt << dir<<"/depth/"<<setfill('0')<<setw(6)<<id<<".pgm"; // ".png"
  ROS_INFO("rgb_file = %s", ss_rgb.str().c_str()); 
  ROS_INFO("dpt_file = %s", ss_dpt.str().c_str());

  rgb = imread(ss_rgb.str().c_str(), -1); 
  dpt = imread(ss_dpt.str().c_str(), -1); 

  // imshow("rgb", rgb); 
  // waitKey(0); 

  return (rgb.data != NULL) && (dpt.data != NULL); 
}



