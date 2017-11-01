/*
 *  Oct. 26, 2015 David Z
 *
 *  collection of planes, 
 *
 * */

#ifndef PLANE_SET_H
#define PLANE_SET_H

#include "plane.h"
#include <vector>

class CPlaneSet
{
  public:
    CPlaneSet();
    ~CPlaneSet();

    void clearPlanes();

    // extract planes 
    template<typename PointT>
    int extractPlanes(boost::shared_ptr<pcl::PointCloud<PointT> > &in, double squared_dis_threshold = 0.005);

    // extract planes and return plane points
    template<typename PointT>
    int extractPlanes(boost::shared_ptr<pcl::PointCloud<PointT> > &in, boost::shared_ptr<pcl::PointCloud<PointT> >& out, 
        std::vector<int>& plane_num, double squared_dis_threshold = 0.005);

    template<typename PointT>
    void extractOutlier(boost::shared_ptr<pcl::PointCloud<PointT> > &in, boost::shared_ptr<pcl::PointCloud<PointT> > &out, 
        pcl::PointIndices::Ptr& inliers);

    template<typename PointT>
    void extractInlier(boost::shared_ptr<pcl::PointCloud<PointT> >& in, 
          boost::shared_ptr<pcl::PointCloud<PointT> >& out, pcl::PointIndices::Ptr& inliers);


    // select floor 
    CPlane* selectFloor();
    float isHorizontal(CPlane* p);

    double d_percent_threshold_; // number of points below this threshold is not considered as a plane
    int d_threhold_num_;  // 
    CPlane* planeAt(int id);
    vector<CPlane*> v_plane_set_; // collection of planes 
};


#include "plane_set.hpp"

#endif 
