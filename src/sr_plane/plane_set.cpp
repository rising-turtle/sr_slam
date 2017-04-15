#include "plane_set.h"
#include <ros/ros.h>

CPlaneSet::CPlaneSet()
{
  ros::NodeHandle nh_p("~"); 
  // nh_p.param<double>("plane_number_threshold", d_percent_threshold_, 0.33);   
  nh_p.param<double>("plane_percent_threshold", d_percent_threshold_, 0.12);  
  nh_p.param<int>("plane_number_threshold", d_threhold_num_, 10000); 
}

CPlaneSet::~CPlaneSet()
{
  clearPlanes();
}

CPlane* CPlaneSet::planeAt(int id)
{
  if(id < 0 || id >= v_plane_set_.size())
  {
    return NULL; 
  }
  return v_plane_set_[id];
}

void CPlaneSet::clearPlanes()
{
  for(int i=0; i<v_plane_set_.size(); i++)
  {
    delete v_plane_set_[i]; 
    v_plane_set_[i] = NULL; 
  }
  v_plane_set_.clear();
}

float CPlaneSet::isHorizontal(CPlane* p)
{
  float nx = fabs(p->nx_); 
  float ny = fabs(p->ny_); 
  float nz = fabs(p->nz_);
  // ROS_WARN("plane_set.cpp: nx %f, ny %f, nz %f", nx, ny, nz);
  if(ny < nx || ny < nz)
  {
    return -1; 
  }
  return ny; 
}

CPlane* CPlaneSet::selectFloor()
{
  CPlane* ret = NULL; 
  double horizontal_scale = -1; 
  for(int i=0; i<v_plane_set_.size(); i++)
  {
    double horizontal_scale_tmp = isHorizontal(v_plane_set_[i]); 
    if(horizontal_scale_tmp != -1 && (horizontal_scale == -1 ||  horizontal_scale_tmp < horizontal_scale))
    {
      horizontal_scale = horizontal_scale_tmp;
      ret = v_plane_set_[i]; 
    }
  }
  return ret;
}


