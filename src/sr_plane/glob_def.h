/*
 *  Sep 14, 2015, David Z
 *
 *  Some global functions
 *
 * */
#ifndef GLOB_DEF_PLANE_H
#define GLOB_DEF_PLANE_H

#include <Eigen/Eigen>
#include <map>
#include <cmath>
#include <tf/tf.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;

// typedef pcl::PointXYZ   Point;
typedef pcl::PointXYZRGBA  Point;
typedef pcl::PointCloud<Point>  Cloud;  
typedef typename pcl::PointCloud<Point>::Ptr CloudPtr;

typedef pair<int, int>  Index;
typedef vector<Index>   Match;


#define D2R(d) (((d)*M_PI)/180.)
#define R2D(r) (((r)*180.)/M_PI)
#define SQ(x)  ((x)*(x))

// for the color of the point cloud 
typedef enum {RED = 0, GREEN, BLUE, PURPLE, WHITE, YELLOW} COLOR; 
extern bool markColor( Cloud& , COLOR);
extern bool markColor(pcl::PointCloud<pcl::PointXYZRGB>& , COLOR);
extern int g_color[][3];

Eigen::Matrix4f getTransformFromMatches(CloudPtr& pc1, CloudPtr& pc2, Match);

tf::Transform getTranRPYt(double r, double p, double y, double tx, double ty, double tz);
tf::Transform getTranRPYt(double r, double p, double y, tf::Vector3 t);

template <typename T >
tf::Transform eigenTransf2TF(const T& transf) 
{
    tf::Transform result;
    tf::Vector3 translation;
    translation.setX(transf.translation().x());
    translation.setY(transf.translation().y());
    translation.setZ(transf.translation().z());

    tf::Quaternion rotation;
    Eigen::Quaterniond quat;
    quat = transf.rotation();
    rotation.setX(quat.x());
    rotation.setY(quat.y());
    rotation.setZ(quat.z());
    rotation.setW(quat.w());

    result.setOrigin(translation);
    result.setRotation(rotation);
    //printTransform("from conversion", result);
    return result;
}

template<>
tf::Transform eigenTransf2TF(const Eigen::Matrix4f& tf);

#endif
