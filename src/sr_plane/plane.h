/*
 *  Sep 14 2015, David Z 
 *  
 *  to represent a plane collection
 *
 * */


#ifndef PLANE_H
#define PLANE_H

#include <vector>

// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
#include "glob_def.h"
#include "point_xyz.h"
#include <limits>
#include <cassert>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

class CSNormal; 

void transformPC(VectorPF3& out, VectorPF3& in, tf::Transform tr);

extern bool computeEigenVector(Eigen::Matrix3f& covariance_matrix, Eigen::Vector3f& major_axis, Eigen::Vector3f& middle_axis,
    Eigen::Vector3f& minor_axis, float& major_value, float& middle_value, float& minor_value);
extern bool computeSmallestEivenVector(Eigen::Matrix3f& cov, Eigen::Vector3f& minor_axis);

class CPlane
{
  public:
    CPlane();
    ~CPlane(); 
    CPlane(CPointF3 nv, float d = 0);
    
  public:
    vector<CPointF3> genRanom(int N, int BX = 10, int BY = 10, int U=100000); // generate random points on this plane within [BX, BY]
    vector<CPointF3> genGauss_dis( vector<CPointF3>& ,  float sigma);  // generate points on a plane with d ~ N(0, sigma^2)
    void setGaussSigma(float s);   // set gauss sigma 
    double gaussValue();                            // output a random value satisfying gauss distribution 

    double pitch();                                 // ouput pitch angle given nx, ny, nz
    float dis2plane(CPointF3);                      // compute the distance between a point to plane 
    float computeZ(float x, float z);               // use plane function to compute Z value 
    void computeParameters(vector<CPointF3>& pts);  // copy point cloud and computePCL(); 
    
    template<typename PointT>
    void computeParameters(boost::shared_ptr<pcl::PointCloud<PointT> >&); 

    void computePCL();            // compute the plane parameters using PCL function
    template<typename PointT>
    void computePCL(boost::shared_ptr<pcl::PointCloud<PointT> >& in, pcl::PointIndices::Ptr& inliers, double dis_threshold = 0.005);   // compute the plane parameters using PCL function, with point cloud in 
    double computeSigma();        // compute the distance sigma of the plane
    void keepInliers();           // only maintain the inliers, delete points not on the plane
    void refineParameters();      // try to refine plane's parameters by casting out more noise data

    CPointF3 projOnPlane(CPointF3 p);                   // project a point into plane 
    vector<CPointF3> projOnPlane(vector<CPointF3> pts); // project these points into plane

  public:
    boost::shared_ptr<CSNormal> pNormal_;     // gauss seed 

  public:
    vector<CPointF3> pts_;
    pcl::PointIndices::Ptr inliers_;  // indicate which point belongs to the plane

    template<typename PointT>
    void computeCenter(boost::shared_ptr<pcl::PointCloud<PointT> >& in, pcl::PointIndices::Ptr& inliers);   // compute the plane parameters using PCL function, with point cloud in 

    // center point 
    float cx_; 
    float cy_; 
    float cz_;

    // different models 
    // // model 1: nx, ny, nz, d, <nx, p> - d = 0
    float nx_; 
    float ny_; 
    float nz_; 
    float d1_;
    void print_m1();

    void loadParameters(vector<float>& p);
    void saveParameters(vector<float>& p);

    // model 2: theta, phi, d, x*cos(theta)*cos(phi) + y*cos(theta)*sin(phi) + z*sin(theta) + d2 = 0
    float theta_; 
    float phi_; 
    float d2_;
    
    // model 3: a, b, d Z = aX + bY + d
    // model 4: a, b, c aX + bY + cZ + 1 = 0
};

#include "plane.hpp"

#endif 
