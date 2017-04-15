/*
 *  Sep. 25, 2015, this is a simulation test 
 *
 * */

#include <iostream>
#include <fstream>
#include <ros/ros.h>

#include "pcl_ros/transforms.h"
#include "plane.h"
#include "vtk_viewer.h"
#include <tf/tf.h>
#include "cam_observation.h"
#include "mean_sigma.h"

#define SHOW 
#define TEST_STATISTIC

using namespace std;

void test_plane1(); 
tf::Transform gauss_match(CPlane* p1, CPlane* p2, Match match, int m, bool b_proj = false);
Match random_sub(Match & match, int m);

void test_plane2();  // test showing projected plane

void test_yaw();  // test transformation with only yaw 

int main(int argc, char* argv[])
{
  test_plane1();
  // test_plane2();
  // test_yaw();
  return 0; 
}

tf::Transform gauss_match(CPlane* p1, CPlane* p2, Match match, int m, bool b_proj)
{
  int N = match.size();
  if(m > N)
  {
    m = N; 
  }
  
  // randomly get m pairs 

  Match sub_match = random_sub(match, m); 

  // obtain the simulated observation point cloud
  VectorPF3 pc_tar = genSR4k_Noise(p1, 0.01);
  VectorPF3 pc_src = genSR4k_Noise(p2, 0.01);
  
  if(b_proj)  // project the points onto the plane
  {
    // TODO 
    // 1 plane fitting 
    p1->pts_.swap(pc_tar); 
    p2->pts_.swap(pc_src);

    // save plane's parameters
    vector<float> para_1(4);
    vector<float> para_2(4);
    p1->saveParameters(para_1);
    p2->saveParameters(para_2);
    p1->computePCL(); 
    p2->computePCL(); 

    // each time the 
    // CPlane tmp_p1;
    // CPlane tmp_p2;
    // tmp_p1.computeParameters(pc_tar);
    // tmp_p2.computeParameters(pc_src);

    // 2 proj the point on the fitted plane
    pc_tar = genSR4k(p1);
    pc_src = genSR4k(p2);
    // pc_tar = genSR4k(&tmp_p1); 
    // pc_src = genSR4k(&tmp_p2);
    p1->loadParameters(para_1);
    p2->loadParameters(para_2);
  }
  
  CloudPtr pcl_tar(new Cloud); 
  CloudPtr pcl_src(new Cloud); 
  toPC(pc_tar, pcl_tar);
  toPC(pc_src, pcl_src);

  Eigen::Matrix4f eT12 = getTransformFromMatches(pcl_src, pcl_tar, sub_match);
  tf::Transform tT12 = eigenTransf2TF(eT12); 
  return tT12;
}

void test_plane1()
{
  // generate a plane and display the points observed from SR4k
  // generate plane1
  CPointF3 nv1(0, 0, 1);
  double d1 = 2;
  CPlane p1(nv1, d1);

  // generate observation for plane1
  VectorPF3 pts = genSR4k(&p1); 
  // VectorPF3 pts = genSR4k_Noise(&p1, 0.01);
  CloudPtr pc(new Cloud); 
  toPC(pts, pc); 

  // generate a transformation 
  double r, p, y, tx, ty, tz; 
  double gr = 20 ; // 20; 
  double gp = 10 ;// 10; 
  double gy = 5 ; // 20 ;
  r = D2R(gr); 
  p = D2R(gp); 
  y = D2R(gy); 
  tx = 0.1; ty = 0.2; tz = 0.3; 
  tf::Transform T12 = getTranRPYt(r, p, y, tx, ty, tz); 
  tf::Transform T21 = T12.inverse();

  // generate another plane 
  tf::Vector3 tnv1(nv1[0], nv1[1], nv1[2]);
  tf::Vector3 tnv2 = T21.getBasis()*tnv1;
  tnv2.normalize();
  CPointF3 nv2(tnv2.x(), tnv2.y(), tnv2.z());
  tf::Vector3 move_t = T21.getBasis()*tf::Vector3(tx, ty, tz);
  // double d2 = d1 - tnv2.dot(tf::Vector3(tx, ty, tz));
  double d2 = d1 - tnv2.dot(move_t);
  CPlane p2(nv2, d2);

  // observation for plane2 
  VectorPF3 pts2 = genSR4k(&p2);
  CloudPtr pc2(new Cloud); 
  toPC(pts2, pc2); 

  // transform the point cloud into the global reference 
  Eigen::Matrix4f eigen_T12;
  pcl_ros::transformAsMatrix(T12, eigen_T12);
  CloudPtr pc3(new Cloud);
  pcl::transformPointCloud(*pc2, *pc3, eigen_T12);

  // find matched points between plane1 and plane2 
  VPair match = findMatchSR4k(pts2, T12);  // match, <src_id, tar_id>
  VectorPF3 pts4, pts5; 
  
  // ofstream ouf("match_points.log");

  for(int i=0; i<match.size(); i++)
  {
    // pts4.push_back(pts2[match[i].second]);
    pts4.push_back(pts2[match[i].first]);
    pts5.push_back(pts[match[i].second]);
    // ouf<<match[i].first<<" "<<match[i].second<<endl;
  }
  
  // ouf.close();
  cout<<"test_plane2.cpp: match size: "<<match.size()<<endl;

  CloudPtr pc4(new Cloud); 
  toPC(pts4, pc4);
  CloudPtr pc5(new Cloud); 
  toPC(pts5, pc5);

  // compute transformation 
  Eigen::Matrix4f eT12 = getTransformFromMatches(pc2, pc, match);
  // cout<<"eT12 : "<<eT12<<endl;
  tf::Transform tT12 = eigenTransf2TF(eT12); 
  // tfScalar r, p, y;
  tT12.getBasis().getEulerYPR(y, p, r); 
  tf::Vector3 t = tT12.getOrigin(); 
  tx = t.getX(); ty = t.getY(); tz = t.getZ();
  cout<<"getEulerYPY: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<" tx: "<<tx<<" ty: "<<ty<<" tz: "<<tz<<endl;
   
#ifdef SHOW
  CVTKViewer<Point> viewer;
  viewer.getViewer()->addCoordinateSystem(0.2, 0,0,0);
  markColor(*pc, GREEN);
  markColor(*pc2, BLUE);
  markColor(*pc3, RED);
  markColor(*pc4, YELLOW);
  markColor(*pc5, PURPLE);
  viewer.addPointCloud(pc, "planeXY"); 
  viewer.addPointCloud(pc2, "plane2r");
  viewer.addPointCloud(pc3, "plane2");
  // viewer.addPointCloud(pc4, "src_pc");
  // viewer.addPointCloud(pc5, "tar_pc");
  while(!viewer.stopped())
  {
    viewer.runOnce(); 
    usleep(100000);
  }
#endif
/*
  // test gauss_proj

  int tmp = 20;

 for(int i=0; i< 10; i++)
 {
  T12 = gauss_match(&p1, &p2, match, tmp, true); 
  T12.getBasis().getEulerYPR(y, p, r); 
  tf::Vector3 t = T12.getOrigin(); 
  tx = t.getX(); ty = t.getY(); tz = t.getZ();
  cout<<"getEulerYPY with proj: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<" tx: "<<tx<<" ty: "<<ty<<" tz: "<<tz<<endl;
 }
*/
  // test transformation using gaussian noise 
#ifdef TEST_STATISTIC
  {
    // TEST WITHOUT PROJECT FEATURES ONTO PLANE
    int m[] = {5, 10, 20, 40, 80, 160, 320}; 
    int k = sizeof(m)/sizeof(int);

    // save the statistic data into file
    stringstream ss; 
    ss<<"./sim_result/gauss_with_pro_"<<m[0]<<"-"<<m[k-1]<<"_roll_"<<gr<<"_pitch_"<<gp<<"_yaw_"<<gy<<".log";
    // ss<<"./sim_result/gauss_without_pro_"<<m[0]<<"-"<<m[k-1]<<"_roll_"<<gr<<"_pitch_"<<gp<<"_yaw_"<<gy<<".log";
    ofstream f(ss.str().c_str()); 

    srand((NULL));

    for(int j=0; j<k; j++)
    {
      int T = 5000;
      vector<double> er(T);
      vector<double> ey(T); 
      vector<double> ep(T);
      vector<double> ex(T);
      vector<double> ey2(T);
      vector<double> ez(T);

      for(int i=0; i<T; i++)
      {
        T12 = gauss_match(&p1, &p2, match, m[j], true); 
        // T12 = gauss_match(&p1, &p2, match, m[j], false); 
        T12.getBasis().getEulerYPR(y, p, r); 
        // cout<<"getEulerYPY: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<endl;
        tf::Vector3 t = T12.getOrigin(); 
        er[i] = R2D(r); ep[i] = R2D(p); ey[i] = R2D(y); 
        ex[i] = t.getX(); ey2[i] = t.getY(); ez[i] = t.getZ();
      }

      double mu, sigma; 
      compute_mu_sigma(er.data(), T, mu, sigma); 
      f<<m[j]<<" "<<mu<<" "<<sigma;
      compute_mu_sigma(ep.data(), T, mu, sigma);
      f<<" "<<mu<<" "<<sigma;
      compute_mu_sigma(ey.data(), T, mu, sigma); 
      f<<" "<<mu<<" "<<sigma;
      compute_mu_sigma(ex.data(), T, mu, sigma);
      f<<" "<<mu<<" "<<sigma;
      compute_mu_sigma(ey2.data(), T, mu, sigma); 
      f<<" "<<mu<<" "<<sigma;
      compute_mu_sigma(ez.data(), T, mu, sigma);
      f<<" "<<mu<<" "<<sigma<<endl;
    }
    f.close();
  }
#endif 

  return ;
}

void test_yaw()
{
  CPointF3 nv1(0, 0, 1);
  double d1 = 2;
  CPlane p1(nv1, d1);

  // generate observation for plane1
  VectorPF3 pts = genSR4k(&p1); 
  // VectorPF3 pts = genSR4k_Noise(&p1, 0.01);
  CloudPtr pc(new Cloud); 
  toPC(pts, pc); 

  // generate a transformation 
  double r, p, y, tx, ty, tz; 
  double gr = 0 ; // 20; 
  double gp = 0 ; // 10; 
  double gy = 25 ;
  r = D2R(gr); 
  p = D2R(gp); 
  y = D2R(gy); 
  tx = 0; ty = 0; tz = 0; 
  tf::Transform T12 = getTranRPYt(r, p, y, tx, ty, tz); 
  tf::Transform T21 = T12.inverse();

  // cout<<"test_plane2.cpp: T12: "<<endl<<T12<<endl;

  // generate another plane 
  tf::Vector3 tnv1(nv1[0], nv1[1], nv1[2]);
  tf::Vector3 tnv2 = T21*tnv1;
  tnv2.normalize();
  CPointF3 nv2(tnv2.x(), tnv2.y(), tnv2.z());
  double d2 = d1 - tnv2.dot(tf::Vector3(tx, ty, tz));
  CPlane p2(nv2, d2);

  // observation for plane2
  Eigen::Matrix4f eigen_T21;
  pcl_ros::transformAsMatrix(T21, eigen_T21);
  CloudPtr pc2(new Cloud);
  pcl::transformPointCloud(*pc, *pc2, eigen_T21);

  VectorPF3 pts_tar; 
  transformPC(pts_tar, pts, T12); 
  CloudPtr pc_tar(new Cloud); 
  toPC(pts_tar, pc_tar);

  // find matched points between plane1 and plane2 
  VPair match = findMatchSR4k(pts, T12);  // match, <src_id, tar_id>
  VectorPF3 pts4, pts5; 
  
  ofstream ouf("match_points.log");

  for(int i=0; i<match.size(); i++)
  {
    // pts4.push_back(pts2[match[i].second]);
    // pts4.push_back(pts2[match[i].first]);
    // pts5.push_back(pts[match[i].second]);
    ouf<<match[i].first<<" "<<match[i].second<<endl;
  }
  
  ouf.close();
  cout<<"test_plane2.cpp: match size: "<<match.size()<<endl;

#ifdef SHOW
  CVTKViewer<Point> viewer;
  viewer.getViewer()->addCoordinateSystem(0.2, 0,0,0);
  markColor(*pc, GREEN);
  markColor(*pc2, BLUE);
  // markColor(*pc3, RED);
  markColor(*pc_tar, YELLOW);
  // markColor(*pc5, PURPLE);
   viewer.addPointCloud(pc, "planeXY"); 
  // viewer.addPointCloud(pc2, "plane2r");
  // viewer.addPointCloud(pc3, "plane2");
  viewer.addPointCloud(pc_tar, "src_pc");
  while(!viewer.stopped())
  {
    viewer.runOnce(); 
    usleep(100000);
  }
#endif

  // Eigen::Matrix4f eT12 = getTransformFromMatches(pc2, pc, match);
  // cout<<"eT12 : "<<eT12<<endl;
  // tf::Transform tT12 = eigenTransf2TF(eT12); 
  // tfScalar r, p, y;
  // tT12.getBasis().getEulerYPR(y, p, r); 
  // cout<<"getEulerYPY: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<endl;
  
  return ;
}

void test_plane2()
{
  // generate a plane and display the points observed from SR4k
  // generate plane1
  // CPointF3 nv1(0, 0, 1);
  CPointF3 nv1(0., 0.6, 0.8);
  double d1 = 2;
  CPlane p1(nv1, d1);
 
  VectorPF3 pts1 = genSR4k(&p1);
  VectorPF3 pts1_noise = genSR4k_Noise(&p1, 0.01);
  
  p1.computeParameters(pts1_noise); 
  VectorPF3 pts1_fit = genSR4k(&p1); 
  
  // show them 
#ifdef SHOW
  CloudPtr pc1_gt(new Cloud); 
  CloudPtr pc1_noise(new Cloud); 
  CloudPtr pc1_fit(new Cloud); 
  
  toPC(pts1_noise, pc1_noise);
  toPC(pts1_fit, pc1_fit);
    
  CVTKViewer<Point> viewer;
  viewer.getViewer()->addCoordinateSystem(0.2, 0,0,0);
  // markColor(*pc, GREEN);
  //  markColor(*pc2, BLUE);
  markColor(*pc1_noise, RED);
  markColor(*pc1_fit, YELLOW);
  //  markColor(*pc5, PURPLE);
  // viewer.addPointCloud(pc, "planeXY"); 
  // viewer.addPointCloud(pc2, "plane2r");
  // viewer.addPointCloud(pc3, "plane2");
  viewer.addPointCloud(pc1_noise, "pc1_noise");
  viewer.addPointCloud(pc1_fit, "pc1_fit");
  while(!viewer.stopped())
  {
    viewer.runOnce(); 
    usleep(100000);
  }
#endif

}

Match random_sub(Match & match, int m)
{
  int N = match.size();
  
  vector<int> index(N, 0); 
  for(int i=0; i<N; i++) 
    index[i] = i;
  int rt, tmp, S; 
  for(int i=0, S = N; i<m; i++)
  {
    rt = rand()%S; 
    // swap rt and S-1
    tmp = index[--S];
    index[S] = index[rt]; 
    index[rt] = tmp;
  }
  Match ret; 
  for(int i=0; i<m; i++)
  {
     ret.push_back(match[index[N-1-i]]);
  }
  return ret;
}





