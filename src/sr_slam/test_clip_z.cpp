/*
 *  Mar.9, 2016 David Z 
 *
 *  clip the z error using floor's information 
 *
 * */

#include <iostream>
#include <cmath>
#include <map>
#include <sstream>
#include <ros/ros.h>

// g2o 
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "../printf_color.h"

using namespace g2o; 

SparseOptimizer* createOptimizeClass();

int main(int argc, char* argv[])
{
  // 1. load graph structure 
  SparseOptimizer * op1 = createOptimizeClass(); 
  if(!op1->load("./subject_eit/incremental_plane3d_opt.g2o"))
  {
    printf(KRED "failed to load incremental_plane3d_opt.g2o, return -1\n");
    return -1;
  }
  SparseOptimizer * op2 = createOptimizeClass(); 
  // if(!op2->load("./subject_eit/incremental_plane3d_opt_offline.g2o"))
  if(!op2->load("./subject_eit/single_incremental_plane3d_opt.g2o"))
  {
    printf(KRED "failed to load incremental_plane3d_opt_offline.g2o, return -1\n"); 
    return -1; 
  }

  double pose1[7]; 
  double pose2[7];

  for(HyperGraph::VertexIDMap::iterator it = op1->vertices().begin(), 
      it2 = op2->vertices().begin();
      it!= op1->vertices().end(); ++it, ++it2)
  {
    // 2.1 add vertex and its edges into pg 
    VertexSE3 * p1 = (VertexSE3*)(it->second); 
    VertexSE3 * p2 = (VertexSE3*)(it2->second); 
    p1->getEstimateData(pose1); 
    p2->getEstimateData(pose2);
    
    pose1[2] = pose2[2]; // replace z value which is cliped by floor plane 
    p1->setEstimateData(pose1);
  }
  op1->save("./subject_eit/plane3d_opt.g2o");
}

void initTypes()
{
  VertexSE3 * v1 = new VertexSE3; 
  EdgeSE3 * e1 = new EdgeSE3;
}

SparseOptimizer* createOptimizeClass()
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
  SparseOptimizer* optimizer = new SparseOptimizer; 
  // SlamLinearSolver* linearSolver = new SlamLinearSolver(); 
  SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver(); 
  linearSolver->setBlockOrdering(false); 
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver); 
  OptimizationAlgorithmLevenberg* algo = new OptimizationAlgorithmLevenberg(blockSolver); 
  
  optimizer->setAlgorithm(algo); 
  return optimizer;
}


