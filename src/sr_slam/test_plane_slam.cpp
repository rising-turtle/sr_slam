/*
 *  Oct. 28 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  implement Plane based SLAM, 
 *  input: the VO's estimation, *.g2o [pcds]/quick_save*.pcd
 *  output: plane_slam_opt.g2o
 *          incrementally add plane features into the graph structure, and 
 *          save the result of graph optimization into plane_slam.g2o
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

// pcl 
// #include "../sr_plane/plane_set.h"
#include "../sr_plane/vtk_viewer.h"
#include "../sr_plane/plane.h"
#include "../sr_plane/plane_set.h"
#include <pcl/io/pcd_io.h>
#include "pcl_ros/transforms.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/transforms.h>
#include "g2o_copy_graph.h"
#include "../time/timestamp.h"

#include "../printf_color.h"

using namespace g2o; 
SparseOptimizer* createOptimizeClass(); 
bool loadPCAtNode(CloudPtr& in, int id);

// global parameters 
string g2o_input_file("./subject_eit/graph.g2o"); 
string pcds_folder("./subject_eit/pcds");

static int g_offset_id = 5000; 

typedef enum RET{NO = 1, YES, REVERSE}; 
RET samePlane(Eigen::Vector4d& v1, Eigen::Vector4d& v2); 

int planeAssociate(Plane3D lp, int node_id, SparseOptimizer* op, map<int, vector<int> >& plane_map); 
bool add_plane_node_incrementally(CloudPtr& pc, int id, SparseOptimizer* op, map<int, vector<int> >&);  // add planes measured at each pose 
void addPlaneIncrementally(SparseOptimizer* op);  // main function 

// implementation of adding plane vertex and plane edge into graph 
void add_plane_node_impl(SparseOptimizer* op, Plane3D * obs, int plane_id, bool fixed); 
void add_plane_edge_impl(SparseOptimizer* op, Plane3D* obs, int node_id, int plane_id, int offset_id);


int main(int argc, char* argv[])
{
  // 1. Initialization, assign parameters 
  ros::init(argc, argv, "plane_slam"); 
  ros::NodeHandle n; 
  if(argc > 1)
  {
    g2o_input_file = string(argv[1]); 
  }
  if(argc > 2)
  {
    pcds_folder = string(argv[2]); 
  }

  // 2. Load g2o graph structure 
  SparseOptimizer * op = createOptimizeClass(); 
  if(op->load(g2o_input_file.c_str()))
  {
    cout<<"test_plane_slam: succeed to load g2o_file: "<<g2o_input_file<<endl; 
    ((g2o::VertexSE3*)op->vertices()[0])->setFixed(true);
  }else
  {
    cout<<"test_plane_slam: failed to load g2o_file "<<g2o_input_file<<endl;
    return -1; 
  }
  
  // 3. Incrementally add node 
  addPlaneIncrementally(op); 

  
  return 0; 
}

void addPlaneIncrementally(SparseOptimizer* op)
{
 // 1. construct a new graph structure 
 SparseOptimizer * pg = createOptimizeClass(); 
 
  CloudPtr pc(new Cloud);    // each node's local point cloud 
  VertexSE3 * last_op = 0; 
  VertexSE3 * last_pg = 0;

  // plane set 
  map<int, vector<int> > planeMap;  // <node_id, <plane_id> >
  int cnt = 0; // times of added plane measurement

  for(HyperGraph::VertexIDMap::iterator it = op->vertices().begin(); it!= op->vertices().end(); ++it)
  {
    // 2.1 add vertex and its edges into pg 
    VertexSE3 * p1 = (VertexSE3*)(it->second); 
    VertexSE3 * p2 = copyNode<VertexSE3, VertexSE3>(p1);
    if(last_pg == 0) // first node 
    {
      pg->addVertex(p2); 
      last_op = p1; 
      last_pg = p2;
      ((g2o::VertexSE3*)pg->vertices()[0])->setFixed(true);
    }else
    {
      Eigen::Isometry3d inc_odo = last_op->estimate().inverse()*p1->estimate(); 
      p2->setEstimate(last_pg->estimate()*inc_odo);
      pg->addVertex(p2); 
      // copy edges in p1 to p2 
      for(HyperGraph::EdgeSet::iterator it = p1->edges().begin(); it!= p1->edges().end(); ++it)
      {
        // cout<<"vin->id = "<<vin->id()<<" plast->edge[0].id2 = "<<(*it)->vertices()[1]->id()<<endl;
        if((*it)->vertices()[0]->id() < p1->id()) // find the loop edge
        {
          EdgeSE3* e_in = dynamic_cast<EdgeSE3*>(*it); 
          EdgeSE3* e2 = copyEdge<EdgeSE3, EdgeSE3>(e_in, pg);
          pg->addEdge(e2);
        }
      }
    }
    last_op = p1; 
    last_pg = p2;

    // 2.2 extract planes from this node 
    // load the point cloud of this node 
    bool b_getPC = loadPCAtNode(pc, p1->id());
    // if(!loadPCAtNode(pc, p1->id())) 
    //  continue; 
    
    // time for plane associate 
    // 2.3 add plane into the graph 
    if(b_getPC && !add_plane_node_incrementally(pc, p1->id(), pg, planeMap))
    {
      continue; 
    }
    
    if(++cnt % 5 == 0)
    {
      pg->setVerbose(true);
      pg->initializeOptimization(); 
      pg->optimize(5);
    }
    last_pg = (VertexSE3*)(pg->vertices()[p2->id()]);
  }

  // save g2o structure 
  pg->save("plane_slam_opt.g2o");
  // delete op; 
  // pg->setVerbose(true);
  // pg->initializeOptimization(); 
  // int result_op = pg->optimize(5); 
  // pg->save("plane_slam_opt_after.g2o");
  return ;
}

bool add_plane_node_incrementally(CloudPtr& pc, int node_id, SparseOptimizer* op, map<int, vector<int> >& plane_map)
{
  CPlaneSet p_pSet; 
  int num_p = p_pSet.extractPlanes(pc, 0.04); 
  printf("%s Extract %d planes from node : %d\n",__FILE__, num_p, node_id); 
  if(num_p == 0) return false;

  // compute the normal vector of the extracted planes
  vector<Plane3D*> pVec(num_p, NULL); 
  Eigen::Vector4d v; 
  for(int i=0; i<num_p; ++i)
  {
    CPlane* p_plane = p_pSet.planeAt(i); 
    // Plane3D 
    v << p_plane->nx_, p_plane->ny_, p_plane->nz_, p_plane->d1_; 
    Plane3D* tmp_p = new Plane3D();
    tmp_p->fromVector(v); 
    pVec[i] = tmp_p; 
  }
  
  static bool first_node = true; 
  static int plane_id = g_offset_id + 1;

  bool b_old_plane_observed = false;
  vector<int> plane_ids(num_p); 
  if(first_node) // first node set up floor plane and wall planes
  { 
    // offset vertex 
    Eigen::Isometry3d offset3d = Eigen::Isometry3d::Identity();
    g2o::VertexSE3 * offsetVertex = new g2o::VertexSE3; 
    offsetVertex->setId(g_offset_id); 
    offsetVertex->setEstimate(offset3d); 
    offsetVertex->setFixed(true);
    op->addVertex(offsetVertex); 

    // fix planes observed at first node 
    for(int i = 0; i < num_p; i++)
    {
      Plane3D g_plane = ((g2o::VertexSE3*)op->vertices()[node_id])->estimate() * (*pVec[i]);
      // add plane and prior initial wall planes    
      add_plane_node_impl(op, &g_plane, plane_id, true); 
      
      // add plane edge
      add_plane_edge_impl(op, pVec[i], node_id, plane_id, g_offset_id);

      // remember plane id
      plane_ids[i] = plane_id++;
    }
    // map node' id to the ids of observed planes 
    plane_map[node_id] = plane_ids;
    first_node = false; 
  }else
  {
    // add these planes into the graph 
    for(int i=0; i<pVec.size(); i++)
    {
      Plane3D g_plane = ((g2o::VertexSE3*)op->vertices()[node_id])->estimate() * (*pVec[i]);

      int plane_associated = planeAssociate(*pVec[i], node_id, op, plane_map);
      // if(planeAssociate(*pVec[i], id, op, plane_map) >= 0) // associate with old plane
      Eigen::Vector4d l_coeff = pVec[i]->coeffs();
      Eigen::Vector4d g_coeff = g_plane.coeffs(); 
      // printf(KWHT "%s current plane %d lp: %f %f %f %f  gp: %f %f %f %f \n", __FILE__, i, l_coeff(0), l_coeff(1),
      //    l_coeff(2), l_coeff(3), g_coeff(0), g_coeff(1), g_coeff(2), g_coeff(3)); 
      if(plane_associated >= 0) // associate with old plane
      {
        plane_ids[i] = plane_associated; 
        b_old_plane_observed = true; 
        // printf(KGRN "%s current plane associates with plane id: %d\n", __FILE__, plane_associated); 
      }else  // create a new plane and add it into the graph structure 
      {
         add_plane_node_impl(op, &g_plane, plane_id, false); 
         add_plane_edge_impl(op, pVec[i], node_id, plane_id, g_offset_id);
         // printf(KRED "%s current plane is new, set id: %d \n", __FILE__, plane_id);
         // new_plane.push_back(plane_id++); 
         plane_ids[i] = plane_id++;
      }
    }
    // link plane to pose 
    plane_map[node_id] = plane_ids;
  }
  return b_old_plane_observed; 
}

void add_plane_node_impl(SparseOptimizer* op, Plane3D * obs, int plane_id, bool fixed)
{
  g2o::VertexPlane* p = new g2o::VertexPlane; 
  p->setEstimate(*obs); 
  p->setId(plane_id); 
  p->setFixed(fixed); 
  op->addVertex(p); 
  return ; 
}

void add_plane_edge_impl(SparseOptimizer* op, Plane3D* obs, int node_id, int plane_id, int offset_id)
{
  g2o::EdgeSE3PlaneSensorCalib * e = new g2o::EdgeSE3PlaneSensorCalib; 
  e->vertices()[0] = op->vertices()[node_id]; // [it->first]; 
  e->vertices()[1] = op->vertices()[plane_id]; // [floor_id];
  e->vertices()[2] = op->vertices()[offset_id]; 
  // e->setMeasurement(*(it->second)); 
  e->setMeasurement(*obs);
  Eigen::Matrix3d w_m = Eigen::Matrix3d::Zero(); 
  w_m(0,0) = 100;  // aminth contribution
  w_m(1,1) = 100;   // elevation 
  w_m(2,2) = 10000; 

  e->setInformation(w_m); 
  op->addEdge(e);
  return ;
}

RET samePlane(Eigen::Vector4d& ni, Eigen::Vector4d& nj)
{
  RET ret = YES; 
  static double COS10 = cos(10.*M_PI/180.);
  double COSA = ni.transpose()*nj - ni(3)*nj(3) ;
  
  if(COSA < 0)
  {
    // reverse 
    COSA = -1*COSA; 
    nj = nj*-1.; 
    ret = REVERSE; 
  }

  if(COSA < COS10) return NO; 
  if(fabs(ni(3) - nj(3)) > 0.1)
  {
    return NO; 
  }
  
  return ret; 
}

int planeAssociate(Plane3D lp, int node_id, SparseOptimizer* op, map<int, vector<int> >& plane_map)
{
  int ret = -1; 
  Plane3D wplane = ((g2o::VertexSE3*)op->vertices()[node_id])->estimate()*(lp); 
  map<int, vector<int> >::reverse_iterator rit = plane_map.rbegin(); 

  int N = 5; 
  int i = 0; 
  // rit++; // skip self
  while(rit != plane_map.rend())
  {
    int node_id = (*rit).first; 
    vector<int>& plane_ids = (*rit).second; 
    
    for(int j=0; j<plane_ids.size(); j++)
    {
      g2o::VertexPlane* p = ((g2o::VertexPlane*)op->vertices()[plane_ids[j]]); 
      
      // whether wp and p are the same plane
      Eigen::Vector4d v1 = p->estimate().toVector(); 
      Eigen::Vector4d v2 = wplane.toVector(); 
      
      // whether these two planes match
      RET pret = samePlane(v1, v2); 
      if(pret != NO) // same plane
      {
        if(pret == REVERSE) // need reverse plane measurement
        {
          v2 = lp.toVector(); 
          v2 = v2 * -1.;
          lp.fromVector(v2); 
        }
        
        // add plane edge into graph 
        add_plane_edge_impl(op, &lp, node_id, plane_ids[j], g_offset_id);
    
        return plane_ids[j]; 
      }else // not the same
      {
        // new plane, generate plane node and an edge
        // add_plane_node_impl(op, )
      }
    }

    ++rit; 
    if(i++ > N) break; 
  }
  return ret; 
}

bool loadPCAtNode(CloudPtr& pc, int nid)
{
  bool ret = true;
  stringstream ss; 
  ss<<pcds_folder<<"/quicksave_"<<std::setfill('0')<<std::setw(4)<<nid<<".pcd";
  if(pcl::io::loadPCDFile(ss.str(), *pc) == 0)
  {
    // cout<<"test_mul_plane_3d.cpp: succeed to load node "<<nid<<" 's point cloud"<<endl;
  }else
  {
    cout<<"test_plane_slam: failed to load node "<<nid<<" 's point cloud"<<endl;
    cout<<"ss = "<<ss.str()<<endl;
    ret = false;
    // return p_set; 
  }
  return ret;
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




