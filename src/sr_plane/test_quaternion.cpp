
#include <ros/ros.h>
#include <tf/tf.h>
// #include "global_def.h"
#include "glob_def.h"
#include <iostream>


using namespace std;

void test_quaternion();
void test_quaternion2();

void displayMatrix(tf::Matrix3x3& m);
void displayQuaternion(tf::Quaternion& q );


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "quaternion_test"); 
  ros::NodeHandle n;

  // test_quaternion();
  test_quaternion2();

  return 0; 
}

void test_quaternion2()
{
  // roll, pitch, yaw
  tfScalar r, p, y;
  r = D2R(10); p = D2R(90); y= D2R(5);
  tf::Quaternion q; 
#ifndef TF_EULER_DEFAULT_ZYX
  cout<<"test_quaternion: TF_EULER_DEFAULT_ZYX"<<endl;
#else
  cout<<"test_quaternion: else"<<endl;
#endif
  // q.setEuler(y, p, r);
  q.setRPY(r, p , y);

  displayQuaternion(q);
  cout<<"m.getEulerYPY: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<endl;

  // project q into matrix and reproject back
  tf::Matrix3x3 m (q); 
  displayMatrix(m);
  // m.setEulerYPR(y, p, r);
  // displayMatrix(m);
  m.getEulerYPR(y, p, r); 
  m.getRotation(q); 
  displayQuaternion(q);
  cout<<"m.getEulerYPY: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<endl;

}


void test_quaternion()
{
  // increase yaw 30d each time 
  float roll = 0, pitch = 0, yaw = D2R(30); 
  tf::Quaternion q(roll, pitch, yaw); 
  tf::Quaternion k = q; 
  float y;
  for(int i=0; i<20; i++)
  { 
    q = q*k; 
    y = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1-2*(SQ(q.y()) + SQ(q.z())));
    cout<<" q.length = "<<q.length()<<" q.angle = "<<R2D(q.getAngle())<<" yaw: "<<R2D(y)<<endl;
  }
}



void displayMatrix(tf::Matrix3x3& m)
{
  tf::Matrix3x3FloatData mf; 
  m.serializeFloat(mf);
  for(int i=0; i<3; i++)
  {
    for(int j=0;j<3;j++)
      cout<< mf.m_el[i].m_floats[j]<<" ";
    cout<<endl;
  }
}

void displayQuaternion(tf::Quaternion& q )
{
  cout<<"qx: "<<q.x()<<" qy: "<<q.y()<<" qz: "<<q.z()<<" q.w: "<<q.w()<<endl;
}


