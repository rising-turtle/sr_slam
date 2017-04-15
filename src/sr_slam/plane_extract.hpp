/*
 *  Jun 19, 2015, David Z
 *  Plane extraction, 
 *    input a point cloud, 
 *    output the parameter of this plane
 * */

#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

// for debug
// #include <ros/ros.h>


using namespace std;

template<typename PointT>
CPlaneExtract<PointT>::CPlaneExtract()
{
  minimal_plane_points_ = 5000; // 30% * 144 * 176 ~ 7000
  distance_threshold_ = 0.03;   // search distance 3 cm
}

template<typename PointT>
CPlaneExtract<PointT>::~CPlaneExtract(){}

template<typename PointT>
bool CPlaneExtract<PointT>::extract( CloudPtr& in, Eigen::Vector4f& nv)
{
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); 
  
  bool ret = extract(in, inliers, coefficients); 
  if(ret)
  {
    nv[0] = coefficients->values[0];
    nv[1] = coefficients->values[1];
    nv[2] = coefficients->values[2]; 
    nv[3] = coefficients->values[3]; 

    if(nv[1] < 0) // ny < 0
    // if(nv[2] < 0) // nz<0, 
      nv*=-1.;

    // record the y coordinate of the floor 
    int mid = inliers->indices.size();
    floor_y_ = in->points[inliers->indices[mid]].y; 
    floor_z_ = in->points[inliers->indices[mid]].z;
    floor_x_ = in->points[inliers->indices[mid]].x;
  }else
  {
    cerr<<"plane_extract.hpp: failed to extract floor!"<<endl;
  }
  return ret;
}

template<typename PointT>
bool CPlaneExtract<PointT>::extract( CloudPtr& in, 
    pcl::PointIndices::Ptr& inliers, pcl::ModelCoefficients::Ptr& coefficients)
{ 
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);    
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold_);
  seg.setInputCloud(in);
  seg.segment(*inliers, *coefficients);

  if(inliers->indices.size() < minimal_plane_points_) 
    return false;
  
  nx_ = coefficients->values[0]; 
  ny_ = coefficients->values[1]; 
  nz_ = coefficients->values[2];
  d_ = coefficients->values[3];

  return true;
}

template<typename PointT>
bool CPlaneExtract<PointT>::extractFloor(CloudPtr& in, CloudPtr& out)
{
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); 
  
  bool ret = extract(in, inliers, coefficients); 
  
  if(!ret)
  {
    return false;
  }
  
  for(int i=0; i<inliers->indices.size(); ++i)
  {
    PointT& pt = in->points[inliers->indices[i]]; 
    out->points.push_back(pt);
  }
  out->width = inliers->indices.size(); 
  out->height = 1; 
  return true;
}

template<typename PointT>
float CPlaneExtract<PointT>::getPitchByGround(CloudPtr& in)
{
  Eigen::Vector4f nv; 
  // filter first 
  CloudPtr filtered(new Cloud); 
  // filterPointCloud(in, filtered);
  nanFilter(in, filtered);

  if(!extract(filtered, nv))
  {
    return -1; 
  }
  float pitch =  getPitchByGround(nv);
  

  /*
  // for debug 
  Eigen::Matrix4f R = Eigen::Matrix4f::Identity();
  float cp = cos(pitch); float sp = sin(pitch); 
  // this rotate around x axis 
  R(1,1) = cp; R(1,2) = -sp; 
  R(2,1) = sp; R(2,2) = cp; 
    
  // this rotate around y axis 
  // R(2,2) = cp; R(0,2) = -sp; 
  // R(2,0) = sp; R(0,0) = cp;

  // Eigen::Vector4f tran_v = R.inverse()*nv; 
  Eigen::Vector4f tran_v = R*nv; 
  ROS_ERROR("plane_extract.hpp: before R %f %f %f", nv(0), nv(1), nv(2)); 
  ROS_ERROR("plane_extract.hpp: after R %f %f %f", tran_v(0), tran_v(1), tran_v(2)); 
*/
  return pitch; 
}
/*
template<typename PointT>
float CPlaneExtract<PointT>::getPitchByGround(Eigen::Vector4f& nv) // align plane to the ground, and extract pitch angle 
{
  //  coordinate reference, ground normal would be {0, 0, 1}
    z   y
    |  /
    | /
    |/____ x  
  
  float nx = fabs(nv[0]);
  if(nx < 1e-4)
  {
    return 0;
  }else
  {
    return atan2(nv[0], nv[2]);
  }
  return 0;
}
*/
template<typename PointT>
float CPlaneExtract<PointT>::getPitchByGround(Eigen::Vector4f& nv) // align plane to the ground, and extract pitch angle 
{
  /*
  // coordinate reference, ground normal would be {0, 1, 0}
    y   x
    |  /
    | /
    |/____ z            

        x
      /
     / 
    /_____ z
    |
    |
    | y
  */

  float nz = fabs(nv[2]);
  if(nz < 1e-4)
  {
    return 0;
  }else
  {
    float p =  atan2(nv[2], nv[1]);
    p *= -1.;
    return p;
  }
  return 0;
}

template<typename PointT>
void CPlaneExtract<PointT>::nanFilter(CloudPtr& in, CloudPtr& out)
{
  for(int i=0; i<in->points.size(); i++)
  {
    PointT& pt = in->points[i]; 
    if(pt.x != pt.x || pt.y != pt.y || pt.z != pt.z)
    {
      continue; 
    }
    out->points.push_back(pt);
  }
  out->width = out->points.size(); 
  out->height = 1;
  return; 
}

template<typename PointT>
void CPlaneExtract<PointT>::filterPointCloud(CloudPtr& in, CloudPtr& out)
{
  CloudPtr tmp(new pcl::PointCloud<PointT>);
  // passthrough filter
  pcl::PassThrough<PointT > pass;
  pass.setInputCloud(in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, depth_limit_);
  pass.filter(*tmp);
  // g_passFilter<PointT>(in, tmp, 0.0, _depth_limit, "z");

  // voxelgrid filter
  pcl::VoxelGrid<PointT> vog;
  vog.setInputCloud(tmp); 
  vog.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  vog.filter(*out);
  return ;
}


