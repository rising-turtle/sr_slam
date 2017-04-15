#include <iostream>
#include <fstream>
#include <ros/ros.h>

#include "pcl_ros/transforms.h"
#include "plane.h"
#include "vtk_viewer.h"
#include <tf/tf.h>

// #define SHOW 

using namespace std;

Eigen::Matrix4f computeTransWithGauss(vector<CPointF3>& pfs, vector<CPointF3>& pts, double sigma, bool bProj = false); 
double computeRPYWithGauss(vector<CPointF3>& pfs, vector<CPointF3>& pts, double sigma, tfScalar& r, tfScalar& p, tfScalar& y, bool bProj= false);
vector<CPointF3> getRandom(int N);
vector<CPointF3> getTPts(vector<CPointF3>& pts, float r, float p, float y);

void test_plane1();
void test_plane2();
void test_projOnPlane();
void test_sum_dis();

void test_plane3();

int main(int argc, char* argv[])
{
  // test_plane1();
  test_plane2();
  // test_projOnPlane();
  // test_sum_dis();
  // test_plane3();
  return 0;
}

void test_plane3()
{
  tfScalar r, p, y;
  r = D2R(30); p = D2R(30); y = D2R(30);
  int N = 10000;
 
  double s_dis = 0;
  
  for(double sigma = 0.03; sigma < 0.5; sigma += 0.03)
  {
    // 
    // ofstream fs()
    // for()
    {

    vector<CPointF3> pts1 = getRandom(N); 
    vector<CPointF3> pts2 = getTPts(pts1, r, p, y); 
 
    s_dis =  computeRPYWithGauss(pts1, pts2, sigma, r, p, y);
    // cout<<"m1: sigma: "<<sigma<<" yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<" dis: "<<s_dis<<endl;
    s_dis = computeRPYWithGauss(pts1, pts2, sigma, r, p, y, true); 
    // cout<<"m2: sigma: "<<sigma<<" yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<" dis: "<<s_dis<<endl;
    }
  }
  
}

void test_plane2()
{
  tfScalar r, p, y;
  r = D2R(30); p = D2R(30); y = D2R(30);
  tfScalar er, ep, ey;
  int N = 10000;
  vector<CPointF3> pts1 = getRandom(N); 
  vector<CPointF3> pts2 = getTPts(pts1, r, p, y); 
  
  double s_dis = 0;

  for(double sigma = 0.03; sigma < 0.5; sigma += 0.03)
  {
    s_dis =  computeRPYWithGauss(pts1, pts2, sigma, er, ep, ey);
    cout<<"m1: sigma: "<<sigma<<" yaw: "<<fabs(R2D(ey)-R2D(y))<<" pitch: "<<fabs(R2D(ep)-R2D(p))<<" roll: "<<fabs(R2D(er) - R2D(r))<<" dis: "<<s_dis<<endl;
    s_dis = computeRPYWithGauss(pts1, pts2, sigma, er, ep, ey, true); 
    cout<<"m2: sigma: "<<sigma<<" yaw: "<<fabs(R2D(ey)-R2D(y))<<" pitch: "<<fabs(R2D(ep)-R2D(p))<<" roll: "<<fabs(R2D(er) - R2D(r))<<" dis: "<<s_dis<<endl;
  }
}

void test_sum_dis()
{
  tfScalar r, p, y;
  r = D2R(30); p = 0; y = 0;
  int N = 10000;
  vector<CPointF3> pts1 = getRandom(N); 
  vector<CPointF3> pts2 = getTPts(pts1, r, p, y); 
  double d = computeRPYWithGauss(pts1, pts2, 0, r, p, y);
  cout<<"m1: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<endl;
  cout<<"d should be zero: "<<d<<endl;
}

void test_projOnPlane()
{
  int N = 20;
  // CPlane p(CPointF3(0 ,0 ,1), 1); 
  CPlane p(CPointF3(0, -0.5, sqrt(0.75)), 1);
  vector<CPointF3> pts1 = p.genRanom(N); // getRandom(N); 
  vector<CPointF3> pts_G = p.genGauss_dis(pts1, 0.02); 
  vector<CPointF3> pts2 = p.projOnPlane(pts_G); 
  p.print_m1();
  for(int i=0; i<N; i++)
  {
    cout<<pts1[i]<<" dis: "<<p.dis2plane(pts1[i])<<" "<<pts2[i]<<" dis :"<<p.dis2plane(pts2[i]) << " "<<
      pts_G[i]<<" dis: "<<p.dis2plane(pts_G[i])<<endl;
  }
}

void test_plane1()
{
  CPlane p1; 
  p1.print_m1(); 

  int N = 10000;
  int M = N/100;
  vector<CPointF3> pts = p1.genRanom(N); 
  CloudPtr pc(new Cloud); 
  toPC(pts, pc);

#ifdef SHOW
  CVTKViewer<Point> viewer;
  viewer.getViewer()->addCoordinateSystem(0.2, 0,0,0);
  viewer.addPointCloud(pc, "planeXY"); 
  while(!viewer.stopped())
  {
    viewer.runOnce(); 
    usleep(100000);
  }
#endif

  // compute the parameters of this plane
  p1.computeParameters(pts); 
  p1.print_m1(); 
  
  // construct a transformation 
  float roll = D2R(-30); 
  tf::Matrix3x3 R12;
  R12.setValue(1, 0, 0, 
                0, cos(roll), -sin(roll), 
                0, sin(roll), cos(roll));
  tf::Transform T12 = tf::Transform(R12); 
  Eigen::Matrix4f eigen_T12; 
  pcl_ros::transformAsMatrix(T12, eigen_T12);
  
  // transform the point cloud  
  CloudPtr pc2(new Cloud); 
  pcl::transformPointCloud(*pc, *pc2, eigen_T12); 
  
  // compute the new plane 
  CPlane p2; 
  vector<CPointF3> pts2;
  fromPC(pts2, pc2); 
  p2.computeParameters(pts2); 
  
  cout<<" p2: "; 
  p2.print_m1();

  // compute the transformation bewteen the two point clouds 
  Match m(M);
  for(int i=0; i<M; i++)
  {
    m[i].first = i;
    m[i].second  = i; 
  }
  
  Eigen::Matrix4f eT12 = getTransformFromMatches(pc2, pc, m); 
  tf::Transform tT12 = eigenTransf2TF(eT12); 
  tfScalar r, p, y;
  tT12.getBasis().getEulerYPR(y, p, r); 
  cout<<"getEulerYPY: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<endl;
  
  return;
}

vector<CPointF3> getRandom(int N)
{
  static CPlane p; 
  vector<CPointF3> ret = p.genRanom(N);
  return ret;
}
vector<CPointF3> getTPts(vector<CPointF3>& pts, float r, float p, float y)
{
  tf::Transform tt; 
  tf::Quaternion q;
  q.setRPY(r, p, y); 
  tf::Matrix3x3 R(q);
  tt.setBasis(R);
  Eigen::Matrix4f et; 
  pcl_ros::transformAsMatrix(tt, et);
  CloudPtr pf(new Cloud);
  CloudPtr pt(new Cloud); 
  toPC(pts, pf);
  pcl::transformPointCloud(*pf, *pt, et); 
  vector<CPointF3> ret;
  fromPC(ret, pt); 
  return ret;
}


double computeRPYWithGauss(vector<CPointF3>& pfs, vector<CPointF3>& pts, double sigma, tfScalar& r, tfScalar& p, tfScalar& y, bool bProj)
{
  Eigen::Matrix4f et = computeTransWithGauss(pfs, pts, sigma, bProj); 
  tf::Transform tt = eigenTransf2TF(et); 
  tt.getBasis().getEulerYPR(y, p, r); 
  
  // compute total error 
  int M = 100;
  double dis = 0;
  for(int i=0; i<M; i++)
  {
    tf::Vector3 p; 
    p.setX(pfs[i][0]); p.setY(pfs[i][1]); p.setZ(pfs[i][2]); p.setW(1); 
    p = tt * p;
    dis += sqrt(SQ(p.x() - pts[i][0]) + SQ(p.y() - pts[i][1]) + SQ(p.z() - pts[i][2])); 
  }

  return (dis/(double)M);
}

Eigen::Matrix4f computeTransWithGauss(vector<CPointF3>& pfs, vector<CPointF3>& pts, double sigma, bool bProj)
{
  CPlane p;
  p.computeParameters(pfs); 
  vector<CPointF3> pfs_G = p.genGauss_dis(pfs, sigma); 
  if(bProj)
  {
    p.computeParameters(pfs_G);
    pfs_G = p.projOnPlane(pfs_G);
  }

  p.computeParameters(pts); 
  vector<CPointF3> pts_G = p.genGauss_dis(pts, sigma); 
  if(bProj)
  {
    p.computeParameters(pts_G); 
    pts_G = p.projOnPlane(pts_G);
  }

  CloudPtr pc_f(new Cloud); 
  CloudPtr pc_t(new Cloud); 
  toPC(pfs_G, pc_f); 
  toPC(pts_G, pc_t); 
  
  int N = pfs.size();
  int M = N/100; 
  Match m(M);
  for(int i=0; i<M; i++)
  {
    m[i].first = i;
    m[i].second = i;
  }

  Eigen::Matrix4f T = getTransformFromMatches(pc_f, pc_t, m); 
  return T;
}

