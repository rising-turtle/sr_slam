/*
 *  Mar. 9, 2016 David Z 
 *
 *  compute the trajectory of the given g2o file 
 *
 * */

#include <iostream>
#include <cmath>
#include <map>
#include <sstream>

#include "../printf_color.h"
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

using namespace std;
using namespace g2o; 
SparseOptimizer* createOptimizeClass(); 

int test1(char* );  // offline optimization 
int main(int argc, char* argv[])
{
  if(argc <=1)
  {
    printf("needs to identify the input g2o.log"); 
    return -1;
  }

  test1(argv[1]);
  return 0; 
}

int test1(char * f_g2o)
{

  // 1. load graph structure 
  SparseOptimizer * op = createOptimizeClass(); 
  if(op->load(f_g2o))
  {
    cout<<"test_traj_length.cpp: succeed to load g2o_file!"<<endl; 
    ((g2o::VertexSE3*)op->vertices()[0])->setFixed(true);
  }else
  {
    cout<<"test_traj_length.cpp: failed to load g2o_file!"<<endl;
  }
  
  // traverse all the nodes in the graph, compute distance 
  double len_xyz = 0; 
  double len_xy = 0; 
  VertexSE3 * last_pg = 0; 
  int last_id = 1000; 
 for(HyperGraph::VertexIDMap::iterator it = op->vertices().begin(); it!= op->vertices().end(); ++it)
  {
    VertexSE3 * p1 = (VertexSE3*)(it->second); 
    // printf("last_id = %d, current_id = %d\n", last_id, it->first);

    if(last_pg == 0) // first node 
    {
      last_pg = p1; 
      last_id = it->first;
      continue; 
    }
    if(it->first != last_id + 1) // not sequential 
    {
      printf(KRED "not sequential last_id = %d, current_id = %d\n", last_id, it->first);
      continue; 
    }
      // compute distance 
      // Eigen::Isometry3d inc_odo = last_pg->estimate().inverse()*p1->estimate(); 
      // Eigen::Vector3d dt = inc_odo.matrix().block<3,1>(0,3); 
      Eigen::Vector3d pi = last_pg->estimate().matrix().block<3,1>(0,3); 
      Eigen::Vector3d pj = p1->estimate().matrix().block<3,1>(0,3); 
      Eigen::Vector3d dt = pi - pj; 

      len_xyz += sqrt(dt.dot(dt));
      len_xy += sqrt(dt(0)*dt(0) + dt(1)*dt(1));
      
      last_pg = p1; 
      last_id = it->first; 
  }
  
 printf(KGRN "total trajectory length 3d = %f\n", len_xyz); 
 printf(KYEL "total trajectory length 2d = %f\n", len_xy); 
 printf(KWHT);
return 0;
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

void initTypes()
{
  VertexSE3 * v1 = new VertexSE3; 
  EdgeSE3 * e1 = new EdgeSE3;
  VertexPlane * v2 = new VertexPlane; 
  EdgeSE3PlaneSensorCalib * e2 = new EdgeSE3PlaneSensorCalib;
}


