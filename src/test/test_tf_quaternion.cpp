#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include "pcl_ros/transforms.h"
#include <pcl/common/transformation_from_correspondences.h>

using namespace std; 

void print_tf(char* name, tf::Transform t);

bool read_total_pts(string f, vector<vector<float> >& pts);

void test_5_points_trans();

void test_example();

void decompose(tf::Transform T, vector<float>& p);

tf::Transform eigenTransf2TF(const Eigen::Matrix4f& tf);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_tf_quaternion"); 

  /*
  tf::Transform T = tf::Transform(tf::Quaternion(0.091916, -0.710675, -0.376525, 0.58713), tf::Vector3(0, 0, 0));
  Eigen::Matrix4f eT; 
  pcl_ros::transformAsMatrix(T, eT); 

  cout<<"test_tf_quaternion.cpp: eigen matrix : "<<eT<<endl;
  */
  
  // test_5_points_trans();
  
  test_example();

  return 0; 
}

void test_example()
{
  pcl::TransformationFromCorrespondences tfc;

  // point 1
  // Eigen::Vector3f pf1(3.2867, 0.8038, -1.1012); 
  Eigen::Vector3f pf1(3.3603, 0.7747, -1.0545); 
  Eigen::Vector3f pt1(0, 0.0430, -0.0225); 
  tfc.add(pf1, pt1, 1.);

  // point 2
  // Eigen::Vector3f pf2(3.3616, 0.8403, -1.0947); 
  Eigen::Vector3f pf2(3.4357, 0.8076, -1.039); 
  Eigen::Vector3f pt2(0, 0.0430, -0.1063);
  tfc.add(pf2, pt2, 1.);

  // point 3
  // Eigen::Vector3f pf3(3.4369, 0.8772, -1.0855); 
  Eigen::Vector3f pf3(3.511, 0.8418, -1.0212); 
  Eigen::Vector3f pt3(0, 0.0430, -0.1906); 
  tfc.add(pf3, pt3, 1.);

  // point 4
  // Eigen::Vector3f pf4(3.2812, 0.7928, -0.9697); 
  Eigen::Vector3f pf4(3.3872, 0.7772, -1.182); 
  Eigen::Vector3f pt4(0.1302, 0.0430, -0.0225); 
  tfc.add(pf4, pt4, 1.);

  // point 5
  // Eigen::Vector3f pf5(3.2941, 0.8155, -1.2305); 
  Eigen::Vector3f pf5(3.3349, 0.7741, -0.9254); 
  Eigen::Vector3f pt5(-0.1302, 0.0430, -0.0225); 
  tfc.add(pf5, pt5, 1.);

  Eigen::Matrix4f T = tfc.getTransformation().matrix(); 
  cout<<"T: "<<endl<<T<<endl; 
  Eigen::Vector4f pt1_t = T * Eigen::Vector4f(pf1(0), pf1(1), pf1(2), 1.); 
  cout<<"pt1: "<<pt1<<endl; 
  cout<<"pt1_t: "<<pt1_t<<endl; 
  return ;

}


void test_5_points_trans()
{
  vector<vector<float> > pts; 
  string f = "/home/davidz/.ros/lsd_slam/gt/dataset_3.total_wp"; 
  if(!read_total_pts(f, pts))
  {
    cout<<"failed to read pts "<<f<<endl;
    return ;
  }
  
  ofstream ouf("tmp_gt_pose.log");

  // compute trans 
  vector<vector<float> > gt_p; 
  vector<float> p;
  for(int i=1; i<pts.size(); i++)
  {  
    pcl::TransformationFromCorrespondences tfc;
    for(int k=0; k<5; k++)
    {
      Eigen::Vector3f from = Eigen::Vector3f(pts[i][k*3], pts[i][k*3+1], pts[i][k*3+2]); 
      Eigen::Vector3f to = Eigen::Vector3f(pts[0][k*3], pts[0][k*3+1], pts[0][k*3+2]); 
      tfc.add(from, to, 1.); 
      // tfc.add(to, from, 1.);
    }
    Eigen::Matrix4f trans = tfc.getTransformation().matrix();
    if(i==500)
    {
      cout<<"trans: "<<endl<<trans<<endl;
      cout<<"trans: inverse: "<<endl<<trans.inverse()<<endl;
    }

    tf::Transform tf_T = eigenTransf2TF(trans); 
    decompose(tf_T, p); 
    for(int j=0; j<7; j++)
      ouf<<p[j]<<" ";
    ouf<<endl;
  }
  return ;
}

bool read_total_pts(string f,vector<vector<float> >& pts )
{
  ifstream inf(f.c_str());
  if(!inf.is_open()){return false;}
  
  char buf[1024];
  vector<float> pt(15, 0); 
  double t; 
  while(inf.getline(buf, 1024))
  {
    sscanf(buf, "%lf %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &t, &pt[0], &pt[1], &pt[2], 
        &pt[3], &pt[4], &pt[5], &pt[6], &pt[7], &pt[8], &pt[9], &pt[10], &pt[11], &pt[12], &pt[13], &pt[14]);
    pts.push_back(pt);
  }
  return true;
}

void decompose(tf::Transform T, vector<float>& p)
{
  p.resize(7); 
  p[0] = T.getOrigin().x(); p[1] = T.getOrigin().y(); p[2] = T.getOrigin().z(); 
  p[3] = T.getRotation().getX(); p[4] = T.getRotation().getY(); p[5] = T.getRotation().getZ(); p[6] = T.getRotation().getW();
}


void print_tf(char* name, tf::Transform t)
{
  ROS_INFO_STREAM(name << ": Translation " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z());
  ROS_INFO_STREAM(name << ": Rotation " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW());
}

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

