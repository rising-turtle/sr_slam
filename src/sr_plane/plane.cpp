#include "plane.h"
#include <stdlib.h>
#include "normal.h"
#include <iostream>
#include <algorithm>
#include "mean_sigma.h"


template<>
void toPC(vector<CPointF3>& pts, CloudPtr& pc)
{
  int N = pts.size();
  pc->points.resize(N); 
  pc->width = N;
  pc->height = 1;
  for(int i=0; i<N; i++)
  {
    Point& pt = pc->points[i]; 
    CPointF3& p = pts[i]; 
    pt.x = p[0]; pt.y = p[1]; pt.z = p[2]; 
  }
}

template<>
void toPC(vector<CPointF3>& pts, CloudPtr& pc, pcl::PointIndices::Ptr& inliers)
{
  int N = inliers->indices.size();
  pc->points.resize(N); 
  pc->width = N;
  pc->height = 1;
  for(int i=0; i<N; i++)
  {
    Point& pt = pc->points[i]; 
    CPointF3& p = pts[inliers->indices[i]]; 
    pt.x = p[0]; pt.y = p[1]; pt.z = p[2]; 
  } 
}

template<>
void fromPC(vector<CPointF3>& pts, CloudPtr& pc)
{
  int N = pc->points.size(); 
  pts.resize(N); 
  for(int i=0; i<N; i++)
  {
    CPointF3& p = pts[i]; 
    Point& pt = pc->points[i]; 
    p[0] = pt.x; p[1] = pt.y; p[2] = pt.z;
  }
}

template<>
void fromPC(vector<CPointF3>& pts, CloudPtr& pc, pcl::PointIndices::Ptr& inliers)
{
  int N = inliers->indices.size(); 
  pts.resize(N); 
  for(int i=0; i<N; i++)
  {
    CPointF3& p = pts[i]; 
    Point& pt = pc->points[inliers->indices[i]]; 
    p[0] = pt.x; p[1] = pt.y; p[2] = pt.z;
  }
}


void transformPC(VectorPF3& out, VectorPF3& in, tf::Transform tr)
{
  int N = in.size();
  out.resize(N);
  for(int i=0; i<N; i++)
  {
    CPointF3 & p = in[i];
    tf::Vector3 from(p[0], p[1], p[2]); 
    tf::Vector3 to = tr*from; 
    out[i] = CPointF3(to.x(), to.y(), to.z());
  }
}



CPointF3::CPointF3()
{
}

CPointF3::CPointF3(float x, float y, float z)
{
  _p[0] = x; _p[1] = y; _p[2] = z;
}

CPointF3::CPointF3(float v)
{
  _p[0] = _p[1] = _p[2] = v;
}

bool CPointF3::isValid()
{
  return (!IS_INF(_p[0]) && !IS_INF(_p[1]) && !IS_INF(_p[2]));
}

////////////////////////////////////////////////////////////

CPlane::CPlane(): 
pNormal_(new CSNormal()),
inliers_(new pcl::PointIndices),
  nx_(0), 
  ny_(0),
  nz_(1),
  d1_(0)
{}
CPlane::~CPlane(){}

CPlane::CPlane(CPointF3 nv, float d):
pNormal_(new CSNormal()),
inliers_(new pcl::PointIndices)
{
  nx_ = nv[0]; ny_ = nv[1]; nz_ = nv[2]; 
  d1_ = d;
}


//TODO compute m2 parameters from m1  
// void CPlane::m1_2_m2() //
// {}
//
//

vector<CPointF3> CPlane::projOnPlane(vector<CPointF3> pts)
{
  int N = pts.size();
  vector<CPointF3> ret(N); 
  for(int i=0; i<N; i++)
    ret[i] = projOnPlane(pts[i]);
  return ret;
}

CPointF3 CPlane::projOnPlane(CPointF3 p)
{
  float d = p[0]*nx_ + p[1]*ny_ + p[2]*nz_ + d1_;
  if(fabs(d) <= 1e-6)
  {
    return p;
  }
  CPointF3 ret;
  if(d >= 0) 
  {
    double l = d ; //sqrt(d); 
    ret[0] = p[0] - l*nx_; ret[1] = p[1] - l*ny_; ret[2] = p[2] - l*nz_;
  }else
  {
    double l = -1.*d; //sqrt(-1.*d); 
    ret[0] = p[0] + l*nx_; ret[1] = p[1] + l*ny_; ret[2] = p[2] + l*nz_;
  }
  return ret;
}

float CPlane::dis2plane(CPointF3 p)
{
  float ret = nx_*p[0] + ny_*p[1] + nz_*p[2] + d1_; 
  // cout<<p<<" nx_: "<<nx_<<" ny_: "<<ny_<<" nz: "<<nz_<< " d: "<<d1_<<endl; 
  // cout<< "dot: "<< nx_*p[0] + ny_*p[1] + nz_*p[2]<<endl;
  return fabs(ret);
}

float CPlane::computeZ(float x, float y)
{
  float z = d1_; 
  assert(nz_ != 0); 
  z = (-nx_*x - ny_*y + d1_) / nz_;
  return z;
}

vector<CPointF3>  
CPlane::genRanom(int N, int BX, int BY, int U) // generate random points on this plane 
{
  srand(NULL);
  vector<CPointF3> ret(N);
  float ux = (float)(BX)/(float)(U); 
  float uy = (float)(BY)/(float)(U);
  float px, py, pz;
  for(int i=0; i<N; i++) 
  {
    px = (rand()%U)*ux; 
    py = (rand()%U)*uy;
    pz = computeZ(px, py);
    ret[i] = CPointF3(px, py, pz);

    // cout<<px<<" "<<py<<" "<<pz<<" dis: "<<dis2plane(ret[i])<<endl;
  }
  return ret;
}

void CPlane::print_m1()
{
  cout<<"model_1: nv: "<<nx_<<", "<<ny_<<", "<<nz_<<", d: "<<d1_<<" pitch: "<<R2D(pitch())<<endl;
}

void CPlane::setGaussSigma(float sigma)
{
  if(pNormal_->sigma_ != sigma)
    pNormal_->setSigma(sigma);
}

double CPlane::gaussValue()
{
  return pNormal_->normal(); 
}

vector<CPointF3> CPlane::genGauss_dis(vector<CPointF3>& pts, float sigma)
{
  setGaussSigma(sigma);
  int N = pts.size();
  vector<CPointF3> ret(N); 
  double lambda; 
  float px, py,pz;
  for(int i=0; i<N; i++)
  {
    CPointF3 & p = pts[i]; 
    lambda = pNormal_->normal(); 
    if(lambda < 0)
    {
      // lambda = -sqrt(-1.*lambda);
      lambda = -1.*lambda;
    }else
    {
      // lambda = sqrt(lambda);
      lambda = lambda;
    }
    px = p[0] + lambda * nx_; 
    py = p[1] + lambda * ny_; 
    pz = p[2] + lambda * nz_;
    ret[i] = CPointF3(px, py, pz);
  }
  return ret;
}

double CPlane::pitch()
{
  double theta ; 
  if(ny_ < 0)
  {
    theta = atan2(-nz_, -ny_); 
  }
  else{
    theta = atan2(nz_, ny_); 
  }
  return theta;
}

void CPlane::computeParameters(vector<CPointF3>& pts)
{
  pts_ = pts; 
  computePCL();
}

void CPlane::saveParameters(vector<float>& p)
{
  p.resize(4); 
  p[0] = nx_; p[1] = ny_; p[2] = nz_; p[3] = d1_;
  
  // TODO: add theta  
}

void CPlane::loadParameters(vector<float>& p)
{
  nx_ = p[0]; ny_ = p[1]; nz_ = p[2]; d1_ = p[3];
  
  // TODO: add theta
}

/*
void CPlane::computePCL(CloudPtr& in)
{
  // Create the segmentation object
  pcl::SACSegmentation<Point> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);    
  seg.setMethodType(pcl::SAC_RANSAC);
  // seg.setDistanceThreshold(0.5);
  seg.setDistanceThreshold(0.01); // 0.0025

  // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // int N = in->points.size();
  // inliers_->indices.resize(N);
  // for(int i=0; i<N; i++) 
  //  inliers_->indices[i] = i;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setInputCloud(in);
  seg.segment(*inliers_, *coefficients);
  
  nx_ = coefficients->values[0];
  ny_ = coefficients->values[1];
  nz_ = coefficients->values[2];
  d1_ = - coefficients->values[3];
  return ;
}*/

void CPlane::computePCL()
{
  // 
  CloudPtr in(new Cloud);
  toPC(pts_, in);
  
  // cout<<"plane.cpp: fit plane parameters: nv: "<<nx_<<" "<<ny_<<" "<<nz_<<" d: "<<d1_<<endl;
  
  computePCL(in, inliers_);

  return ;
}

void CPlane::keepInliers()
{
  int N = inliers_->indices.size(); 
  if(N <=0 || pts_.size() <=0 ) return; 
  VectorPF3 tmp(N); 
  // cout<<"keepInliers() N = "<<N<<endl;
  // ofstream ouf("tmp2.log");
  for(int i=0; i<N; i++)
  {
    tmp[i] = pts_[inliers_->indices[i]];
    // ouf<<inliers_->indices[i]<<" "<<tmp[i]<<" "<<tmp[i].dis_()<<endl;
  }
  // dumpPC2File(tmp, "tmp.log");
  pts_.swap(tmp);
}

double CPlane::computeSigma()
{
  int M = inliers_->indices.size(); 
  vector<double> dis(M); 
  for(int i=0; i<M; i++)
  {
    CPointF3& pt = pts_[inliers_->indices[i]]; 
    // distance of a point to this plane 
    dis[i] = fabs(pt[0]*nx_ + pt[1]*ny_ + pt[2]*nz_ - d1_);
  }
  
  double mean, sigma ; 
  compute_mu_sigma(&dis[0], M, mean, sigma);
  ROS_INFO("plane.cpp: dis mean %f and sigma %f", mean, sigma);
  
  return sigma;
}

void CPlane::refineParameters()
{
  //
  keepInliers();  // delete outliers 
  VectorPF3 ori_pts = copyPercent(pts_, 0.5); 
  CloudPtr ori_plane(new Cloud);
  toPC(ori_pts, ori_plane);

  // for test 
  Eigen::Matrix3f cov; 
  calPCCorvariance<Point>(ori_plane, cov); 
  Eigen::Vector3f nv; 
  computeSmallestEivenVector(cov, nv); 

  print_m1();

  nx_ = nv(0); ny_ = nv(1); nz_ = nv(2);
  cout<<"plane.cpp: using PCA nv: "<<nv(0)<<" "<<nv(1)<<" "<<nv(2)<<endl;

  print_m1(); 

  pts_ = ori_pts; 
  computePCL(); 
  cout<<"plane.cpp: using PCL compute: "<<endl;
  print_m1();

  // 
  return ;
}

bool computeSmallestEivenVector(Eigen::Matrix3f& cov, Eigen::Vector3f& minor_axis)
{
  float major_v, middle_v, minor_v; 
  Eigen::Vector3f middle_axis, major_axis; 
  bool ret = computeEigenVector(cov, major_axis, middle_axis, minor_axis, major_v, middle_v, minor_v);
  return ret;
}

bool computeEigenVector(Eigen::Matrix3f& covariance_matrix, Eigen::Vector3f& major_axis, Eigen::Vector3f& middle_axis,
    Eigen::Vector3f& minor_axis, float& major_value, float& middle_value, float& minor_value)
{
  Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> > eigen_solver;
  eigen_solver.compute (covariance_matrix);

  Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> >::EigenvectorsType eigen_vectors;
  Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> >::EigenvalueType eigen_values;
  eigen_vectors = eigen_solver.eigenvectors ();
  eigen_values = eigen_solver.eigenvalues ();

  unsigned int temp = 0;
  unsigned int major_index = 0;
  unsigned int middle_index = 1;
  unsigned int minor_index = 2;

  if (eigen_values.real () (major_index) < eigen_values.real () (middle_index))
  {
    temp = major_index;
    major_index = middle_index;
    middle_index = temp;
  }

  if (eigen_values.real () (major_index) < eigen_values.real () (minor_index))
  {
    temp = major_index;
    major_index = minor_index;
    minor_index = temp;
  }

  if (eigen_values.real () (middle_index) < eigen_values.real () (minor_index))
  {
    temp = minor_index;
    minor_index = middle_index;
    middle_index = temp;
  }

  major_value = eigen_values.real () (major_index);
  middle_value = eigen_values.real () (middle_index);
  minor_value = eigen_values.real () (minor_index);

  major_axis = eigen_vectors.col (major_index).real ();
  middle_axis = eigen_vectors.col (middle_index).real ();
  minor_axis = eigen_vectors.col (minor_index).real ();

  major_axis.normalize ();
  middle_axis.normalize ();
  minor_axis.normalize ();

  float det = major_axis.dot (middle_axis.cross (minor_axis));
  if (det <= 0.0f)
  {
    major_axis (0) = -major_axis (0);
    major_axis (1) = -major_axis (1);
    major_axis (2) = -major_axis (2);
  }
  return true;
} 

