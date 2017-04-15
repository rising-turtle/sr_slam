/*
 * July. 22, 2016 David Z
 *
 * test whether the graph based on SE3 work or not 
 *
 *
 * */
#include <tf/tf.h>
#include "GlobalMapping/KeyFrameGraphSE3.h"
#include "GlobalMapping/g2oTypeSE3Sophus.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/estimate_propagator.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include "pcl_ros/transforms.h"
#include <iostream>

using namespace std;
using namespace g2o;
using namespace lsd_slam;

void test_graph_se3();
Sophus::SE3d computeTrans(float *p); 

int main(int argc, char* argv[])
{
  test_graph_se3();
  return ;
}

void test_graph_se3()
{
  g2o::SparseOptimizer g;

  typedef g2o::BlockSolver_6_3 BlockSolver;
  typedef g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType> LinearSolver;
  //typedef g2o::LinearSolverPCG<BlockSolver::PoseMatrixType> LinearSolver;
  LinearSolver* solver = new LinearSolver();
  BlockSolver* blockSolver = new BlockSolver(solver);
  g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
  g.setAlgorithm(algorithm);

  VertexSE3 * v0 = new VertexSE3();
  v0->setFixed(true); 
  v0->setMarginalized(false); 

  VertexSE3 * v1 = new VertexSE3();  
  VertexSE3 * v2 = new VertexSE3();
  VertexSE3 * v3 = new VertexSE3();

  v0->setId(0); v1->setId(1); v2->setId(2); v3->setId(3);
  g.addVertex(v0); 
  g.addVertex(v1); 
  g.addVertex(v2); 
  g.addVertex(v3);
  
  Eigen::Matrix<double,6,6,Eigen::ColMajor> information = Eigen::Matrix<double,6,6,Eigen::ColMajor>::Identity();
  EdgeSE3* e01 = new EdgeSE3();
  EdgeSE3* e12 = new EdgeSE3(); 
  EdgeSE3* e02 = new EdgeSE3();
  e01->setId(0);  e12->setId(1);  e02->setId(2); 
  e01->resize(2); e12->resize(2); e02->resize(2);
 
  float p[7] = {1, 1, 1, 0, 0, 0, 1};
  Sophus::SE3d T = computeTrans(p);
  e01->setMeasurement(T);   e12->setMeasurement(T);
  
  p[3] = 0.5; p[4] = 0.; p[6] = sqrt(1-p[3]*p[3]-p[4]*p[4]);
  T = computeTrans(p); 
  e02->setMeasurement(T);
  e01->setMeasurement(T);

  {
    Sophus::SE3d T01 = v0->estimate().inverse()*v1->estimate(); 
    Eigen::Matrix4d eT01 = T01.matrix();
    cout<<"before optimation T01 = "<<endl<<eT01; 
  }

  e01->setInformation(information); e12->setInformation(information); e02->setInformation(information);
  
  e01->setVertex(0, v0);  e12->setVertex(0, v1);  e02->setVertex(0, v0);
  e01->setVertex(1, v1);  e12->setVertex(1, v2);  e02->setVertex(1, v2);
  g.addEdge(e01); // g.addEdge(e12); g.addEdge(e02);

  g.setVerbose(true);
  g.initializeOptimization(); 
  g.optimize(20);
  g.save("test_graph.log");
  
  {
    Sophus::SE3d T01 = v0->estimate().inverse()*v1->estimate(); 
    Eigen::Matrix4d eT01 = T01.matrix();
    cout<<"after optimation T01= "<<endl<<eT01; 
  }

  g.clear();
  return ;

}

Sophus::SE3d computeTrans(float *p)
{
  tf::Transform tf_T = tf::Transform(tf::Quaternion(p[3], p[4], p[5], p[6]), tf::Vector3(p[0],p[1],p[2]));
  Eigen::Matrix4f eigen_T;
  pcl_ros::transformAsMatrix(tf_T, eigen_T);
  // cout<<"e02: trans: "<<endl<<eigen_T<<endl;
  Sophus::SE3d referenceToFrame(eigen_T.cast<double>());
  // Eigen::Matrix4d T_d = referenceToFrame.matrix(); 
  Sophus::Vector6d v6 = referenceToFrame.log(); 
  for(int i=0; i<6; i++)
  {
    // cout<<v6[i]<<" "<<p[i]<<endl;
  }
  // cout<<"e02: Td: "<<endl<<T_d<<endl;
  return referenceToFrame;
}
