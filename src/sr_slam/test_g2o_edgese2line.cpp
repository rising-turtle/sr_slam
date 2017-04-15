/*
 *  Jan. 14 2016, David Z 
 *  
 *  test edgeSE2Line2D in g2o, write a simulated case
 *
 * */

#include <iostream>
#include <cmath>

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d_addons/edge_se2_line2d.h"
#include "edge_se2_line2d_zh.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

using namespace std; 
using namespace g2o; 

/**
 *  * normalize the angle
 *   */
/*
inline double normalize_theta(double theta)
{
  if (theta >= -M_PI && theta < M_PI)
    return theta;

  double multiplier = floor(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}*/

int main(int argc, char* argv[])
{
  /*  1. create the optimization problem 
   * */    
  typedef BlockSolver<BlockSolverTraits<-1,-1> > SlamBlockSolver; 
  // typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver; 
  // typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
  typedef LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
  typedef LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;

  // allocating optimizer 
  SparseOptimizer optimizer; 
  // SlamLinearSolver* linearSolver = new SlamLinearSolver(); 
  SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver(); 
  linearSolver->setBlockOrdering(false); 
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver); 
  OptimizationAlgorithmLevenberg* algo = new OptimizationAlgorithmLevenberg(blockSolver); 
  
  optimizer.setAlgorithm(algo); 
  
  // 2, set the root node , id 0
  SE2 t0(2680, 550, -3.1416); 
  // SE2 t0(2680, 550, 0);
  SE2 st0 = t0;
  int vertex_id = 0; 
  VertexSE2 * pose = new VertexSE2; 
  pose->setId(vertex_id++); 
  pose->setEstimate(t0); 
  pose->setFixed(true);  
  optimizer.addVertex(pose); 

  // 3, add 2 line node , id 1
  VertexLine2D * vline = new VertexLine2D; 
  vline->setTheta(M_PI/2.);
  vline->setRho(582);  // 2.
  vline->setId(vertex_id++); 
  vline->setFixed(true); // true
  optimizer.addVertex(vline); 
  
  vline = new VertexLine2D; 
  vline->setTheta(0); 
  vline->setRho(5.); 
  vline->setId(vertex_id++);  // id 2
  vline->setFixed(true); // true
  optimizer.addVertex(vline); 

  // 4, add one motion step 
  SE2 gt_inc01(1., 0, 0); 
  // SE2 odo_inc01(0.8, 0.1, M_PI/18.); 
  SE2 odo_inc01(2, -2, -0.03625); 
  VertexSE2 * pose1 = new VertexSE2; 
  pose1->setId(vertex_id++);  // id 3
  pose1->setEstimate(t0*odo_inc01); 
  optimizer.addVertex(pose1); 
  
  // 4.2, add another motion 
  VertexSE2 * pose2 = new VertexSE2; 
  pose2->setId(vertex_id++);  // id 4
  pose2->setEstimate(t0*odo_inc01*odo_inc01); 
  // optimizer.addVertex(pose2);

  // 5, add 2 odo edge
  Eigen::Matrix3d information = Eigen::Matrix3d::Identity(); 
  double odo_certainty = 100; 
  double obs_certainty = 100; 
  information(0,0) = information(1,1) = information(2,2) = odo_certainty; 

  EdgeSE2 * odo = new EdgeSE2; 
  odo->vertices()[0] = optimizer.vertex(0); 
  odo->vertices()[1] = optimizer.vertex(3); 
  odo->setMeasurement(odo_inc01); 
  odo->setInformation(information); 
  optimizer.addEdge(odo); 
  
  odo = new EdgeSE2; 
  odo->vertices()[0] = optimizer.vertex(3); 
  odo->vertices()[1] = optimizer.vertex(4); 
  odo->setMeasurement(odo_inc01); 
  odo->setInformation(information); 
  // optimizer.addEdge(odo);

  // 6, add se2_line edge 
  Eigen::Matrix2d information_l = Eigen::Matrix2d::Identity();
  information_l(0,0) = information_l(1,1) = obs_certainty; 
  EdgeSE2Line2DZH* obs_l1 = new EdgeSE2Line2DZH; 
  obs_l1->vertices()[0] = optimizer.vertex(3); 
  obs_l1->vertices()[1] = optimizer.vertex(1); 
  SE2 gt_pose = t0*gt_inc01; 
  SE2 iT = gt_pose.inverse();
  const VertexLine2D *l1 = static_cast<const VertexLine2D*>(obs_l1->vertices()[1]); 
  Vector2D prediction = l1->estimate();
  prediction[0] = -M_PI/2.; 
  prediction[1] = 36; 
  // prediction[0] += iT.rotation().angle(); 
  // prediction[0] = normalize_theta(prediction[0]);
  Vector2D n(cos(prediction[0]), sin(prediction[0])); 
  // prediction[1] += n.dot(iT.translation());
  obs_l1->setMeasurement(prediction); 
  obs_l1->setInformation(information_l); 
  optimizer.addEdge(obs_l1); 

  // pose2 observe line 2
  EdgeSE2Line2DZH* obs_l2 = new EdgeSE2Line2DZH; 
  obs_l2->vertices()[0] = optimizer.vertex(4); // 3 
  obs_l2->vertices()[1] = optimizer.vertex(2); 
  const VertexLine2D *l2 = static_cast<const VertexLine2D*>(obs_l2->vertices()[1]); 
  gt_pose = gt_pose*gt_inc01; 
  iT = gt_pose.inverse();
  prediction = l2->estimate(); 
  prediction[0] += iT.rotation().angle(); 
  prediction[0] = normalize_theta(prediction[0]);
  n = Vector2D(cos(prediction[0]), sin(prediction[0])); 
  prediction[1] += n.dot(iT.translation());
  obs_l2->setMeasurement(prediction); 
  obs_l2->setInformation(information_l); 
  // optimizer.addEdge(obs_l2); 

  // then optimization 
  optimizer.save("test_g2o_edgese2line_before.g2o");
  optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  double ini_chi2 = optimizer.activeChi2();
  cout<<"test_g2o_edgese2line.cpp: initial chi2 is "<<ini_chi2<<endl;
  optimizer.optimize(5);
  optimizer.save("test_g2o_edgese2line_after.g2o");

  // result after optimization 
  SE2 cur_pose = static_cast<VertexSE2*>(optimizer.vertex(vertex_id-1))->estimate(); 
  cout<<"test_g2o_edgese2line.cpp: "<<cur_pose[0]<<" "<<cur_pose[1]<<" "<<cur_pose[2]<<endl;

  // clear
  optimizer.clear();
  // Factory::destroy();
  // OptimizationAlgorithmFactory::destroy();
  // HyperGraphActionLibrary::destroy();
  return 0;
}










