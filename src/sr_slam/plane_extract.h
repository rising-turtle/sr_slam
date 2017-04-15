/*
 *  Jun 19, 2015, David Z
 *  Plane extraction, 
 *    input a point cloud, 
 *    output the parameter of this plane
 * */

#ifndef PLANE_EXTRACT_H
#define PLANE_EXTRACT_H

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <Eigen/Eigen>

template<typename PointT>
class CPlaneExtract
{
  public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    CPlaneExtract(); 
    ~CPlaneExtract();
    bool extract( CloudPtr& in, pcl::PointIndices::Ptr& inliers, pcl::ModelCoefficients::Ptr& coefficients);
    bool extract( CloudPtr& in, Eigen::Vector4f& nv);
    bool extractFloor(CloudPtr& in, CloudPtr& floor);

    float getPitchByGround(Eigen::Vector4f& nv); // align plane to the ground, and extract pitch angle 
    float getPitchByGround(CloudPtr& in); // align plane to the ground, and extract pitch angle 

    double nx_; 
    double ny_;
    double nz_; 
    double d_;

    float floor_y_;       // the y coordinate of the floor 
    float floor_z_;       // the z coordinate of the floor
    float floor_x_;       // the x coordinate of the floor

    int minimal_plane_points_; // minimal points to be counted as a plane
    float distance_threshold_;  // region grow distance 0.03
    

    // for filter 
    void nanFilter(CloudPtr& in, CloudPtr& out);
    void filterPointCloud(CloudPtr& in, CloudPtr& out);
    float depth_limit_; // 5m
    float voxel_size_;  // voxel grid size 0.01
};


#include "plane_extract.hpp"

#endif
