/*
 *  Aug. 16, 2016, David Z 
 * 
 *  Read a binary Point Cloud, filter it, and save it into a .ply
 *
 * */

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>

// typedef pcl::PointXYZ   Point;
typedef pcl::PointXYZRGBA  Point;
typedef pcl::PointCloud<Point>  Cloud;  
typedef typename pcl::PointCloud<Point>::Ptr CloudPtr;

using namespace std;

string pcd_file = "/home/davidz/.ros/rgbd_slam/gt_pc_dataset3.pcd"; 
string ply_file = "/home/davidz/.ros/rgbd_slam/gt_pc_dataset3.ply"; 

void do_it(); 

int main(int argc, char* argv[])
{
  if(argc >= 2)
  {
    pcd_file = argv[1]; 
    if(argc >=3)
      ply_file = argv[2]; 
  }

  cout<<"bin_pc_2_ply.cpp: try to convert "<<pcd_file<<" into "<<ply_file<<endl;

  do_it(); 
  return 1; 

}

template<typename PointT>
void filterPointCloud(typename pcl::PointCloud<PointT>::Ptr& in,
    typename pcl::PointCloud<PointT>::Ptr& out, double _voxel_size = 0.01);

void do_it()
{
   // 1. load pcd 
   CloudPtr g_pc(new Cloud);  // global point cloud  
   if(pcl::io::loadPCDFile(pcd_file, *g_pc) == 0)
   {
     cout<<"bin_pc_2_ply.cpp: succeed to load point cloud "<<pcd_file<<endl;
   }else
   {
     cout<<"bin_pc_2_ply.cpp: failed to load point cloud "<<pcd_file<<endl;
     return ;
   }
  
   // 2. filter it 
   CloudPtr dg_pc(new Cloud); 
   filterPointCloud<Point>(g_pc, dg_pc, 0.02);

   // 3. write header to ply
   ofstream ouf(ply_file); 
   if(!ouf.is_open())
   {
    cout<<"bin_pc_2_ply: failed to open ply_file: "<<ply_file<<endl;
    return ;
   }

   unsigned int n = dg_pc->size();
   unsigned int m = 0;
   unsigned int c_thre = 240; 
   for(unsigned int i=0; i<n; i++)
   {
     Point& pt = dg_pc->points[i]; 
     // ouf<<pt.x<<" "<<pt.y<<" "<<pt.z<<" "<<(unsigned int)pt.r<<" "<<(unsigned int)pt.g<<" "<<(unsigned int)pt.b<<endl;
     if(pt.r + pt.g + pt.b > c_thre)
     {
      ++m;
     }
   }

   ouf<<"ply"<<endl<<"format ascii 1.0"<<endl<<"element vertex "<<m<<endl<<"property float x"<<endl
     <<"property float y"<<endl<<"property float z"<<endl<<"property uchar red"<<endl
     <<"property uchar green"<<endl<<"property uchar blue"<<endl<<"end_header"<<endl;

   // 4. write points to ply 
   for(unsigned int i=0; i<n; i++)
   {
     Point& pt = dg_pc->points[i]; 
     if(pt.r + pt.g + pt.b > c_thre)
       ouf<<pt.x<<" "<<pt.y<<" "<<pt.z<<" "<<(unsigned int)pt.r<<" "<<(unsigned int)pt.g<<" "<<(unsigned int)pt.b<<endl;
   }
   ouf.close(); 
   cout<<"bin_pc_2_ply: succeed to save "<<m<<" points in ply_file: "<<ply_file<<endl;
   return ;
 }

template<typename PointT>
void filterPointCloud(typename pcl::PointCloud<PointT>::Ptr& in,typename pcl::PointCloud<PointT>::Ptr& out, double _voxel_size)
{
   // voxelgrid filter
    pcl::VoxelGrid<PointT> vog;
    vog.setInputCloud(in); 
    vog.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
    vog.filter(*out);
}





