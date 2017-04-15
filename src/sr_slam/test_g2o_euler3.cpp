/*
 *  Jan. 28, 2016, David Z
 *  
 *  try to use Euler angle to stand for rotation 
 *  convert the graph from se3:QUAT to se3:EULER
 *
 * */

#include <iostream>
#include <cmath>
#include <map>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_prior.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/vertex_se3_euler.h"
#include "g2o/types/slam3d_addons/edge_se3_euler.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include "edge_se3_euler_zh.h"
#include "edge_se3_euler2_zh.h"
#include "vertex_se3_euler_zh.h"
#include "g2o_copy_graph.h"

using namespace std; 
using namespace g2o; 

SparseOptimizer* createOptimizeClass();
void initTypes(); // declare types
// void copyGraph(SparseOptimizer* g1, SparseOptimizer* g2); // copy graph from g1->g2
// VertexSE3Euler* copyNode(VertexSE3* pin); 
// EdgeSE3EulerZH* copyEdge(EdgeSE3* pin, SparseOptimizer* g);

string input_graph("./subject_eit/graph.g2o");

int main(int argc, char* argv[])
{
  SparseOptimizer * optimizer_ = createOptimizeClass(); 

  if(argc >= 2)
    input_graph = string(argv[1]);
  // 1. load g2o structure 
  // if(optimizer_->load("./subject_eit/graph.g2o"))
  if(optimizer_->load(input_graph.c_str()))
  {
    cout<<"test_g2o_prior3d.cpp: succeed to load g2o_file!"<<endl; 
  }else
  {
    cout<<"test_g2o_prior3d.cpp: failed to load g2o_file!"<<endl;
  }
  
  SparseOptimizer * op2 = createOptimizeClass(); 
  // 2. transform from quternion to euler 
  copyGraph<VertexSE3, EdgeSE3, VertexSE3EulerZH, EdgeSE3Euler2ZH>(optimizer_, op2);
  
  cout<<"test_g2o_euler3.cpp: op2 has "<<op2->vertices().size()<<" vertexes and "<<op2->edges().size()<<" edges!" <<endl;
  // 3. save as another name 
  op2->save("./subject_eit/graph_euler2.g2o");
  // op2->setVerbose(true);
  // op2->initializeOptimization(); 
  // op2->optimize(3);
  // op2->save("./subject_eit/graph_euler2_opt.g2o");

  // 4. save into loadable graph 
  // SparseOptimizer * op3 = createOptimizeClass(); 
  // copyGraph<VertexSE3EulerZH, EdgeSE3Euler2ZH, VertexSE3, EdgeSE3>(op2, op3); 
  // op3->save("./subject_eit/graph_quat2_opt.g2o");

  return 1;
}
/*
void copyGraph(SparseOptimizer* g1, SparseOptimizer* g2) // copy graph from g1->g2
{
  int n_nodes = 0; 
  int n_edges = 0;
  // copy node first 
  for(HyperGraph::VertexIDMap::iterator it = g1->vertices().begin(); 
      it!= g1->vertices().end(); ++it)
  {
    VertexSE3* vin = (VertexSE3*)(it->second); 
    VertexSE3Euler* vout = copyNode(vin); 
    g2->addVertex(vout); 
    ++n_nodes;
  }
  // copy edge second
  for(HyperGraph::EdgeSet::iterator it = g1->edges().begin(); 
      it != g1->edges().end(); ++it)
  {
    EdgeSE3* ein = (EdgeSE3*)(*it); 
    EdgeSE3EulerZH* eout = copyEdge(ein, g2); 
    g2->addEdge(eout);
    ++n_edges;
  }
  cout<<"test_g2o_euler3.cpp: add "<<n_nodes<<" vertex and "<<n_edges<<" edges!"<<endl;
  return ;
}

VertexSE3Euler* copyNode(VertexSE3* pin)
{
  VertexSE3Euler* pr = new VertexSE3Euler; 
  pr->setId(pin->id()); 
  pr->setEstimate(pin->estimate()); 
  pr->setFixed(pin->fixed());
  return pr;
}
EdgeSE3EulerZH* copyEdge(EdgeSE3* pin, SparseOptimizer* g)
{
  EdgeSE3EulerZH * er = new EdgeSE3EulerZH; 
  int id1 = (pin->vertices()[0])->id(); 
  int id2 = (pin->vertices()[1])->id(); 
  er->vertices()[0] = g->vertex(id1); 
  er->vertices()[1] = g->vertex(id2); 
  er->setMeasurement(pin->measurement());
  er->setInformation(pin->information()); 
  return er;
}
*/
void initTypes()
{
  VertexSE3 * vse3 = new VertexSE3; 
  EdgeSE3 * ese3 = new EdgeSE3;
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










