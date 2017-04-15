/*
 * March 8, 2016 
 * 
 * point xyz type
 *
 * */

#ifndef POINT_XYZ_H
#define POINT_XYZ_H

#include <vector>
#include <iostream>
#include <string>
#include <assert.h>

using namespace std; 

#define SQ(x) ((x)*(x))

template<int N> 
class CPointF
{
  public:
  float _p[N];
  float& operator[](int i)
  {
    assert(i>= 0 && i<N); 
    return _p[i]; 
  }
};

#define FINF std::numeric_limits<float>::infinity() 
#define IS_INF(x) (std::isinf(x))

class CPointF3 : public CPointF<3>
{ 
  public:
    CPointF3();
    CPointF3(float v);
    CPointF3(float x, float y, float z);
    bool isValid();
    float dis_(){return (SQ(_p[0]) + SQ(_p[1]) + SQ(_p[2]));} // distance to camera 
    float dis_() const
    {return (SQ(_p[0]) + SQ(_p[1]) + SQ(_p[2]));}

    bool operator<(CPointF3& other)
    { return (dis_() < other.dis_()); }
    bool operator<(const CPointF3& other) const
    { return (dis_() < other.dis_()); }
};

typedef std::vector<CPointF3> VectorPF3;

extern ostream& operator<<(ostream& out, CPointF3& p);
extern void dumpPC2File(VectorPF3&, string fname);
extern VectorPF3 copyPercent(VectorPF3& in, float percent);  // maintain percent portion of the original points according to distance


// template<typename PointT>
// void toPC(vector<CPointF3>& pts, boost::shared_ptr<pcl::PointCloud<PointT> >& pc);

// template<typename PointT>
// void toPC(vector<CPointF3>& pts, boost::shared_ptr<pcl::PointCloud<PointT> >& pc, pcl::PointIndices::Ptr& inliers);

// template<typename PointT>
// void fromPC(vector<CPointF3>& pts, boost::shared_ptr<pcl::PointCloud<PointT> >& pc);

// template<typename PointT>
// void fromPC(vector<CPointF3>& pts, boost::shared_ptr<pcl::PointCloud<PointT> >& pc, pcl::PointIndices::Ptr& inliers);

// template<typename PointT>
// void calPCCorvariance(boost::shared_ptr<pcl::PointCloud<PointT> >& pc, Eigen::Matrix3f& cov);


#endif
