/*  
 *  Jan. 23 2016, David Z 
 *  
 *  test adding prior 3d Line in g2o, 
 *  first, using VertexSE3EulerZH EdgeSE3Euler to represent the graph structure,
 *  second, add priorSE3Euler to this graph, and then optimize it, and then save it

 *  this does not work, because the positions of the priorSE3Euler is the result after optimization, 
 *  therefore, try another way, replace the position where priorSE3Euler is, and use incremental transformation to 
 *  construct the final trajectory 
 *
    this does not work, because the incremental transformation is the result after optimization, 
    therefore, try another way, record every position of 2D SLAM, replacing x, y, and roll of every 6dof pose 
    with the result of 2D SLAM
    
    use incremental increase and last pose to get current pose, and then replace its x, y, and yaw, 
 *   
 * */

#include <iostream>
#include <cmath>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_prior.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include "g2o/types/slam3d_addons/vertex_se3_euler.h"
#include "edge_se3_euler_zh.h"
#include "edge_se3_euler2_zh.h"
#include "vertex_se3_euler_zh.h"
#include "edge_se3_prior_zh.h"
#include "g2o_copy_graph.h"

using namespace std; 
using namespace g2o; 
typedef Eigen::Matrix<double, 6, 6, Eigen::ColMajor> Matrix6d;

SparseOptimizer* createOptimizeClass();
void initTypes();
void addPriorEdge(SparseOptimizer* op);
void extractOdoToEuler(SparseOptimizer* optimizer_, SparseOptimizer * op_);  // only extract odometry edges between pose nodes

void readPos2D(map<int, vector<float> >& pos_2d, string pos_name="./subject_eit/graph.g2o_pos2D");

void test1(SparseOptimizer* optimizer_); // this is the first idea 
void test2(SparseOptimizer* optimizer_); // this is the second idea 
void test3(SparseOptimizer* optimizer_); // this is the third idea
void test4(SparseOptimizer* optimizer_); // this is the fourth idea

int main(int argc, char* argv[])
{
  SparseOptimizer* optimizer_ = createOptimizeClass(); 
  // 1. load g2o structure 
  // if(optimizer_->load("./subject_eit/graph.g2o"))
  // if(optimizer_->load("./subject_eit/graph_euler2.g2o"))
  //if(optimizer_->load("./subject_eit/gh_plane_after_opt.g2o"))
  if(optimizer_->load("./subject_eit/single_incremental_plane3d_opt.g2o"))
  {
    cout<<"test_g2o_prior3d.cpp: succeed to load g2o_file!"<<endl; 
  }else
  {
    cout<<"test_g2o_prior3d.cpp: failed to load g2o_file!"<<endl;
  }

  // test1(optimizer_);
  // test2(optimizer_);
  // test3(optimizer_);
  test4(optimizer_);  

  return 1; 
}

void test4(SparseOptimizer * op1)
{
  SparseOptimizer* op_ = createOptimizeClass(); 

  // read 2d pos first 
  map<int, vector<float> > pos_2d;
  readPos2D(pos_2d, "./subject_eit/vertex_2d.log");
  
  // traverse every node 
  VertexSE3 * plast = NULL; 
  VertexSE3 * plast_gt = NULL; 
  // map<int, vector<float> >::iterator it_2d = pos_2d.begin(); 
  for(HyperGraph::VertexIDMap::iterator it = op1->vertices().begin(); it!= op1->vertices().end(); ++it)
  {
    VertexSE3* vin = dynamic_cast<VertexSE3*>(it->second); 
    if(vin == NULL)
      continue; 
    VertexSE3 * pin = copyNode<VertexSE3, VertexSE3>(vin); 
    if(plast == NULL)
    {
      op_->addVertex(pin); 
      plast = vin; 
      plast_gt = pin; 
    }else
    {   
      // whether to be adjusted by prior 2D pos 
      map<int, vector<float> >::iterator it_2d = pos_2d.find(vin->id()); 
      if(it_2d == pos_2d.end())
      {
        // ROS_ERROR("test_g2o_prior3d.cpp: cannot find node id %d, something is not right", vin->id());
        cerr<<"test_g2o_prior3d.cpp: cannot find node id "<<vin->id()<<", something is not right"<<endl;
        continue;
      }

      // replace x,y,yaw from 2d result 
      static double est[6]; 
      
      // 1. use last_op * inc_odo as current estimation
      Isometry3D inc_odo = plast->estimate().inverse()*vin->estimate(); 
      pin->setEstimate(plast_gt->estimate()*inc_odo);

      VertexSE3EulerZH * pinEuler = copyNode<VertexSE3, VertexSE3EulerZH>(pin); 
      pinEuler->getEstimateData(est); 

      // 2. replace it with the ground truth value
      // est[0] = it_2d->second[0];  
      est[1] = it_2d->second[1]; // est[5] = it_2d->second[2]; 
      pinEuler->setEstimateDataImpl(est); 
      pin->setEstimate(pinEuler->estimate()); 
      delete pinEuler; 

      // Incrementally add node  
      Isometry3D incremental_trans = plast_gt->estimate().inverse()*pin->estimate(); 
      // pin->setEstimate(plast_gt->estimate()*incremental_trans);

      // add this vertex 
      op_->addVertex(pin); 

      // create an edge connecting plast_gt and pin
      EdgeSE3 * ein = new EdgeSE3; 
      ein->vertices()[0] = plast_gt;
      ein->vertices()[1] = pin; 
      ein->setMeasurement(plast_gt->estimate().inverse()*pin->estimate());
      Matrix6d info = Matrix6d::Identity(); 
      ein->setInformation(info); 
      op_->addEdge(ein); 
      
      // update the last pose 
      plast = vin ;
      plast_gt = pin; 
    }
  }

  // try to save it 
  op_->save("./subject_eit/replace_result.g2o");
  // op_->initializeOptimization();
  // op_->setVerbose(true);
  // op_->optimize(3); 
  // op_->save("./subject_eit/combine_result_after.g2o");
  
  cout<<"test_g2o_prior3d.cpp: finsih test3()"<<endl;

 return ;
 
}

void test3(SparseOptimizer* op1)
{
  SparseOptimizer* op_ = createOptimizeClass(); 

  // read 2d pos first 
  map<int, vector<float> > pos_2d;
  readPos2D(pos_2d, "./subject_eit/vertex_2d.log");
  
  // traverse every node 
  VertexSE3 * plast = NULL; 
  VertexSE3 * plast_gt = NULL; 
  // map<int, vector<float> >::iterator it_2d = pos_2d.begin(); 
  for(HyperGraph::VertexIDMap::iterator it = op1->vertices().begin(); it!= op1->vertices().end(); ++it)
  {
    VertexSE3* vin = dynamic_cast<VertexSE3*>(it->second); 
    if(vin == NULL)
      continue; 
    VertexSE3 * pin = copyNode<VertexSE3, VertexSE3>(vin); 
    if(plast == NULL)
    {
      op_->addVertex(pin); 
      plast = vin; 
      plast_gt = pin; 
    }else
    {   
      // whether to be adjusted by prior 2D pos 
      map<int, vector<float> >::iterator it_2d = pos_2d.find(vin->id()); 
      if(it_2d == pos_2d.end())
      {
        // ROS_ERROR("test_g2o_prior3d.cpp: cannot find node id %d, something is not right", vin->id());
        cerr<<"test_g2o_prior3d.cpp: cannot find node id "<<vin->id()<<", something is not right"<<endl;
        continue;
      }

      // replace x,y,yaw from 2d result 
      static double est[6]; 
      VertexSE3EulerZH * pinEuler = copyNode<VertexSE3, VertexSE3EulerZH>(pin); 
      pinEuler->getEstimateData(est); 
      // est[0] = it_2d->second[0];  
      est[1] = it_2d->second[1]; // est[5] = it_2d->second[2]; 
      pinEuler->setEstimateDataImpl(est); 
      pin->setEstimate(pinEuler->estimate()); 
      delete pinEuler; 

      // Incrementally add node  
      Isometry3D incremental_trans = plast_gt->estimate().inverse()*pin->estimate(); 
      // pin->setEstimate(plast_gt->estimate()*incremental_trans);

      // add this vertex 
      op_->addVertex(pin); 

      // create an edge connecting plast_gt and pin
      EdgeSE3 * ein = new EdgeSE3; 
      ein->vertices()[0] = plast_gt;
      ein->vertices()[1] = pin; 
      ein->setMeasurement(plast_gt->estimate().inverse()*pin->estimate());
      Matrix6d info = Matrix6d::Identity(); 
      ein->setInformation(info); 
      op_->addEdge(ein); 
      
      // update the last pose 
      plast = vin ;
      plast_gt = pin; 
    }
  }

  // try to save it 
  op_->save("./subject_eit/replace_result.g2o");
  // op_->initializeOptimization();
  // op_->setVerbose(true);
  // op_->optimize(3); 
  // op_->save("./subject_eit/combine_result_after.g2o");
  
  cout<<"test_g2o_prior3d.cpp: finsih test3()"<<endl;

 return ;
}

void test2(SparseOptimizer* op1) //
{
  SparseOptimizer* op_ = createOptimizeClass(); 

  // read 2d pos first 
  map<int, vector<float> > pos_2d;
  readPos2D(pos_2d);
  
  // traverse every node 
  VertexSE3 * plast = NULL; 
  VertexSE3 * plast_gt = NULL; 
  // map<int, vector<float> >::iterator it_2d = pos_2d.begin(); 
  for(HyperGraph::VertexIDMap::iterator it = op1->vertices().begin(); it!= op1->vertices().end(); ++it)
  {
    VertexSE3* vin = dynamic_cast<VertexSE3*>(it->second); 
    if(vin == NULL)
      continue; 
    VertexSE3 * pin = copyNode<VertexSE3, VertexSE3>(vin); 
    if(plast == NULL)
    {
      op_->addVertex(pin); 
      plast = vin; 
      plast_gt = pin; 
    }else
    {
      // Incrementally add node  
      Isometry3D incremental_trans = plast->estimate().inverse()*pin->estimate(); 
      pin->setEstimate(plast_gt->estimate()*incremental_trans);
      
      // whether to be adjusted by prior 2D pos 
      map<int, vector<float> >::iterator it_2d = pos_2d.find(vin->id()); 
      if(it_2d != pos_2d.end())
      {
        static double est[6]; 
        VertexSE3EulerZH * pinEuler = copyNode<VertexSE3, VertexSE3EulerZH>(pin); 
        pinEuler->getEstimateData(est); 
        est[0] = it_2d->second[0];  est[1] = it_2d->second[1]; est[5] = it_2d->second[2]; 
        pinEuler->setEstimateDataImpl(est); 
        pin->setEstimate(pinEuler->estimate()); 
        delete pinEuler; 
      }
      
      // add this vertex 
      op_->addVertex(pin); 

      // create an edge connecting plast_gt and pin
      EdgeSE3 * ein = new EdgeSE3; 
      ein->vertices()[0] = plast_gt;
      ein->vertices()[1] = pin; 
      ein->setMeasurement(plast_gt->estimate().inverse()*pin->estimate());
      Matrix6d info = Matrix6d::Identity(); 
      ein->setInformation(info); 
      op_->addEdge(ein); 
      
      // update the last pose 
      plast = vin ;
      plast_gt = pin; 
    }
  }

  // try to save it 
  op_->save("./subject_eit/combine_result_before.g2o");
  op_->initializeOptimization();
  op_->setVerbose(true);
  op_->optimize(3); 
  op_->save("./subject_eit/combine_result_after.g2o");

  return ;
}

void test1(SparseOptimizer* optimizer_) // this is the first idea 
{
  // 2. represent the graph structure using Euler3 
  SparseOptimizer * op_ = createOptimizeClass(); 
  extractOdoToEuler(optimizer_, op_);  // only extract odometry edges between pose nodes
  // copyGraph<VertexSE3, EdgeSE3, VertexSE3EulerZH, EdgeSE3Euler2ZH>(optimizer_, op_);
  // cout<<"test_g2o_prior3d.cpp: op2 has "<<op_->vertices().size()<<" vertexes and "<<op_->edges().size()<<" edges!" <<endl;
  // op_->save("./subject_eit/graph_euler_prior.g2o");

  // 3. addPriorSE3 
  addPriorEdge(op_);
  
  // 4. save graph 
  op_->save("./subject_eit/euler_prior_before.g2o");
  op_->initializeOptimization();
  op_->setVerbose(true);
  op_->optimize(3); 
  op_->save("./subject_eit/euler_prior_after.g2o");

  // 5. to display the result 
  // SparseOptimizer * op2 = createOptimizeClass(); 
  // copyGraph<VertexSE3EulerZH, EdgeSE3Euler2ZH, VertexSE3, EdgeSE3>(op_, op2);
  // op2->save("./subject_eit/qse3_prior_after.g2o");
  return ;
}

void extractOdoToEuler(SparseOptimizer* optimizer_, SparseOptimizer * op_)  // only extract odometry edges between pose nodes
{
  int n_ver = 0; 
  int n_edge = 0;
  // 1. copy node 
  VertexSE3* plast = NULL;
  for(HyperGraph::VertexIDMap::iterator it = optimizer_->vertices().begin(); it!= optimizer_->vertices().end(); ++it)
  {
    VertexSE3* vin = dynamic_cast<VertexSE3*>(it->second); 
    if(vin == NULL)
      continue;
    if(plast == NULL) // the first node
    {
      VertexSE3EulerZH * pin = copyNode<VertexSE3, VertexSE3EulerZH>(vin); 
      op_->addVertex(pin); 
      plast = vin; 
      ++n_ver;
    }else 
    {
      // whether its odometry sequence 
      if(vin->id() != plast->id() + 1)
      {
        cout<<"test_g2o_prior3d.cpp: not odometric sequence, at id = "<<vin->id()<<" continue"<<endl;
        plast = vin; 
        continue; 
      }

      // add node 
      VertexSE3EulerZH * pin = copyNode<VertexSE3, VertexSE3EulerZH>(vin); 
      // cout<<"pin->type_id.name() = "<<typeid(*(pin)).name()<<endl;
      op_->addVertex(pin); 
      ++n_ver;

      // add odometric edge
      // find this edge in 
      for(HyperGraph::EdgeSet::iterator it = plast->edges().begin(); it!= plast->edges().end(); ++it)
      {
        // cout<<"vin->id = "<<vin->id()<<" plast->edge[0].id2 = "<<(*it)->vertices()[1]->id()<<endl;
        if((*it)->vertices()[1]->id() == vin->id()) // find this edge 
        {
          EdgeSE3* e_in = dynamic_cast<EdgeSE3*>(*it); 
          EdgeSE3Euler2ZH * edge_in = copyEdge<EdgeSE3, EdgeSE3Euler2ZH>(e_in, op_); 
          Isometry3D odo_mea = plast->estimate().inverse()*pin->estimate(); 
          edge_in->setMeasurement(odo_mea); 
          op_->addEdge(edge_in);
          ++n_edge;
          break;
        }
        
        // cout<<"test_g2o_prior_3d.cpp: the first edge is not the right one? break;"<<endl;
        // break;
      }
      plast = vin;
    }
  }
  
  cout<<"test_g2o_prior3d.cpp: add vertex: "<<n_ver<<" and edge: "<<n_edge<<endl;
return ;
}

void readPos2D(map<int, vector<float> >& pos_2d, string pos_name)
{
  // read pos2d in file
  // ifstream inf("./subject_eit/graph.g2o_pos2D");
  ifstream inf(pos_name.c_str());
  int id; 
  vector<float> pos(3); 
  // map<int, vector<float> > pos_2d;
  while(!inf.eof())
  {
    inf>>id>>pos[0]>>pos[1]>>pos[2];
    pos_2d[id] = pos;
  }
  inf.close();
  return ;
}

void addPriorEdge(SparseOptimizer* op)
{
  // read adjusted pos 2d 
  map<int, vector<float> > pos_2d;
  readPos2D(pos_2d);

  ParameterSE3Offset* odomOffset=new ParameterSE3Offset();
  odomOffset->setId(0);
  op->addParameter(odomOffset);

  int n_added_prior_edge = 0;
  // create prior 3D 
  map<int, vector<float> >::iterator it = pos_2d.begin(); 

  double buf[6];
  while(it != pos_2d.end())
  {
    // VertexSE3 * v = static_cast<VertexSE3*>(op->vertex(it->first)); 
    VertexSE3EulerZH * v = static_cast<VertexSE3EulerZH*>(op->vertex(it->first)); 
    Matrix6d info = Matrix6d::Zero(); 
    info(0,0) = 50;   // x 
    info(1,1) = 50;   // y
    info(5,5) = 1000;  // yaw 

    EdgeSE3PriorZH * pe = new EdgeSE3PriorZH;  // use euler angle to represent rotation
    pe->setInformation(info);
    // pe->vertices()[0] = v; 
    pe->setVertex(0, v);
    
    // set measurement 
    vector<float>& p2d = it->second; 
    v->getEstimateData(buf); 
    Eigen::Map<Vector6d> pv(buf);
    pv(0) = p2d[0];  // x 
    pv(1) = p2d[1];  // y
    pv(5) = p2d[2];  // yaw
    pe->setMeasurement(g2o::internal::fromVectorET(pv));
    // pe->resolveCacheAndParameters();
    pe->setParameterId(0,0); // set its parameter in graph, the first index_1 0 is the parameter index in this edge, the second index_2 0 is the the parameter index in the graph, pe->_parameters[index_1] = g->getParameter(index_2) 

    ++it; 
    ++n_added_prior_edge; 
    op->addEdge(pe);
  }
  cout<<"test_g2o_prior3d.cpp: added "<<n_added_prior_edge<<" new prior edges!"<<endl;
}

void initTypes()
{
  VertexSE3 * vse3 = new VertexSE3; 
  VertexSE3EulerZH * vse3euler = new VertexSE3EulerZH;
  EdgeSE3 * ese3 = new EdgeSE3;
  EdgeSE3EulerZH * ese3euler = new EdgeSE3EulerZH;
  EdgeSE3Euler2ZH * ese3euler2 = new EdgeSE3Euler2ZH;
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


