
#include "glob_def.h"
#include <pcl/common/transformation_from_correspondences.h>

int g_color[][3] = {255, 0, 0,  //RED
                            0, 255, 0, // GREEN
                            0, 0, 255,  // BLUE
                            255, 0, 255, // PURPLE
                            255, 255,255, // WHITE
                            255, 255, 0 // YELLOW
                        };

bool markColor(Cloud& cloud, COLOR c)
{
  if(cloud.points.size() <= 0)
  {
    // cout<<"global_def.cpp: cloud has no points!"<<endl;
    return false;
  }
  int N = cloud.points.size();
  for(int i=0; i<N; i++)
  {
    cloud.points[i].r = g_color[c][0];
    cloud.points[i].g = g_color[c][1];
    cloud.points[i].b = g_color[c][2];
  }
  return true;
}

bool markColor(pcl::PointCloud<pcl::PointXYZRGB>& cloud, COLOR c)
{
  if(cloud.points.size() <= 0)
  {
    // cout<<"global_def.cpp: cloud has no points!"<<endl;
    return false;
  }
  int N = cloud.points.size();
  for(int i=0; i<N; i++)
  {
    cloud.points[i].r = g_color[c][0];
    cloud.points[i].g = g_color[c][1];
    cloud.points[i].b = g_color[c][2];
  }
  return true;
}

Eigen::Matrix4f getTransformFromMatches(CloudPtr& pc_f, CloudPtr& pc_t, Match m) 
{
  pcl::TransformationFromCorrespondences tfc;
  // std::vector<Eigen::Vector3f> t, f;

  // BOOST_FOREACH(const cv::DMatch& m, matches)
  for(int i=0; i<m.size(); i++)
  {
    int queryIdx = m[i].first; 
    int trainIdx = m[i].second;
    Eigen::Vector3f from ;  
    Eigen::Vector3f to  ;
    Point& pfrom  = pc_f->points[queryIdx];
    Point& pto = pc_t->points[trainIdx]; 

    from(0) = pfrom.x; from(1) = pfrom.y; from(2) = pfrom.z; 
    to(0) = pto.x;     to(1) = pto.y;     to(2) = pto.z;
    if(isnan(from(2)) || isnan(to(2)))
      continue;
    float weight = 1.0;

   // f.push_back(from);
   // t.push_back(to);    

    tfc.add(from, to, weight);// 1.0/(to(2)*to(2)));//the further, the less weight b/c of quadratic accuracy decay
  }

  // get relative movement from samples
  return tfc.getTransformation().matrix();
}

tf::Transform getTranRPYt(double r, double p, double y, double tx, double ty, double tz)
{
  tf::Vector3 t(tx, ty, tz);
  return getTranRPYt(r, p, y, t);
}
tf::Transform getTranRPYt(double r, double p, double y, tf::Vector3 t)
{
  tf::Transform tt; 
  tf::Quaternion q;
  q.setRPY(r, p, y); 
  tf::Matrix3x3 R(q);
  tt.setBasis(R);
  tt.setOrigin(t);
  return tt;
}

template<>
tf::Transform eigenTransf2TF(const Eigen::Matrix4f& tf)
{
  tf::Transform result;
  tf::Vector3 translation;
  translation.setX(tf(0,3));
  translation.setY(tf(1,3));
  translation.setZ(tf(2,3));
  tf::Matrix3x3 R(tf(0,0), tf(0,1), tf(0,2),
                  tf(1,0), tf(1,1), tf(1,2), 
                  tf(2,0), tf(2,1), tf(2,2));
  result.setOrigin(translation);
  result.setBasis(R);
  return result;
}

