/*
 *  Jan. 31, 2016 David Z 
 *
 *  plane extraction and transform them into the same coordinate 
 *
 * */

#include <iostream>
#include <cmath>
#include <map>
#include <sstream>

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
#include <pcl/io/pcd_io.h>
#include "pcl_ros/transforms.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/transforms.h>
#include "g2o_copy_graph.h"
#include "../time/timestamp.h"

using namespace g2o; 
template<typename PointT>
void extractCloud(typename pcl::PointCloud<PointT>::Ptr& in, 
                typename pcl::PointCloud<PointT>::Ptr& out,
                pcl::PointIndices::Ptr& index, bool inv = false);

template<typename PointT>
void filterPointCloud(typename pcl::PointCloud<PointT>::Ptr& in,
    typename pcl::PointCloud<PointT>::Ptr& out, double _voxel_size = 0.01);

SparseOptimizer* createOptimizeClass(); 

int test1();  // offline optimization 
void test2(); // incrementally add planes
bool add_plane_node_incrementally(Plane3D* p, int id, SparseOptimizer* op);
bool loadPCAtNode(CloudPtr& pc, int nid);
map<int, Plane3D*> extractPlanes(SparseOptimizer* op);
void addPlaneNode(SparseOptimizer* op, map<int, Plane3D*>& p_set);
bool isFloor(Plane3D* p);

int main(int argc, char* argv[])
{
  // test1();
  test2(); 
  return 0; 
}

void test2()
{
  ofstream ouf("./subject_eit/sin_plane_time.log"); 

  // 1. load graph structure 
  SparseOptimizer * op = createOptimizeClass(); 
  if(op->load("./subject_eit/graph.g2o"))
  {
    cout<<"test_mul_plane_3d.cpp: succeed to load g2o_file!"<<endl; 
    ((g2o::VertexSE3*)op->vertices()[0])->setFixed(true);
  }else
  {
    cout<<"test_mul_plane_3d.cpp: failed to load g2o_file!"<<endl;
  }
  
  Eigen::Vector4d v;
  CPlane * p_plane = new CPlane;
  int i_thresh_number = 5000;   // at least 5000 points on this plane

  // 2. incrementally add node 
  SparseOptimizer * pg = createOptimizeClass(); 
  CloudPtr pc(new Cloud);    // each node's local point cloud 
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  VertexSE3 * last_op = 0; 
  VertexSE3 * last_pg = 0;
  for(HyperGraph::VertexIDMap::iterator it = op->vertices().begin(); it!= op->vertices().end(); ++it)
  {
    VertexSE3 * p1 = (VertexSE3*)(it->second); 
    VertexSE3 * p2 = copyNode<VertexSE3, VertexSE3>(p1);
    if(last_pg == 0) // first node 
    {
      pg->addVertex(p2); 
    }else
    {
      Eigen::Isometry3d inc_odo = last_op->estimate().inverse()*p1->estimate(); 
      p2->setEstimate(last_pg->estimate()*inc_odo);
      pg->addVertex(p2); 
      for(HyperGraph::EdgeSet::iterator it = p1->edges().begin(); it!= p1->edges().end(); ++it)
      {
        if((*it)->vertices()[0]->id() < p1->id()) // find the previous edge
        {
          EdgeSE3* e_in = dynamic_cast<EdgeSE3*>(*it); 
          EdgeSE3* e2 = copyEdge<EdgeSE3, EdgeSE3>(e_in, pg);
          pg->addEdge(e2);
        }
      }
    }
    last_op = p1; 
    last_pg = p2; 

    bool b_getPC = loadPCAtNode(pc, p1->id());
    TTimeStamp before_plane = getCurrentTime(); 
    
    if(b_getPC)
    {
      // 2.2 extract floor plane from this node 
      p_plane->computePCL(pc, inliers);
      bool b_getPlane = (inliers->indices.size() > i_thresh_number);
      if(b_getPlane)
      {
        // extract plane 
        Plane3D* tmp = new Plane3D(); 
        v << p_plane->nx_, p_plane->ny_, p_plane->nz_, p_plane->d1_;
        tmp->fromVector(v);
        add_plane_node_incrementally(tmp, p1->id(), pg);
      }  
    }else
    {
      cerr<<"test_plane_3d.cpp: what failed to get pcd at node =  "<<p1->id()<<endl;
    }
    TTimeStamp after_plane = getCurrentTime();
    ouf<<timeDifference(before_plane, after_plane)<<" ";

    // 3 optimize the graph 
    pg->initializeOptimization(); 
    pg->optimize(2); 
    TTimeStamp after_op = getCurrentTime(); 
    ouf<<timeDifference(after_plane, after_op)<<" "<<timeDifference(before_plane, after_op)<<endl;
  }
  pg->save("./subject_eit/single_incremental_plane3d_opt.g2o"); 
  ouf.close();

  return ; 
}



int test1()
{
  // 1. load graph structure 
  SparseOptimizer * op = createOptimizeClass(); 
  if(op->load("./subject_eit/graph.g2o"))
  {
    cout<<"test_plane_3d.cpp: succeed to load g2o_file!"<<endl; 
    ((g2o::VertexSE3*)op->vertices()[0])->setFixed(true);
  }else
  {
    cout<<"test_plane_3d.cpp: failed to load g2o_file!"<<endl;
  }

  // 2. extract plane for each node, and transform them into a single point cloud 
  map<int, Plane3D*> p_set = extractPlanes(op); 
  
  // 3. add plane node into the graph 
  addPlaneNode(op, p_set); 
  op->save("./subject_eit/gh_plane_before_opt.g2o"); 
  op->initializeOptimization();
  op->setVerbose(true); 
  op->optimize(10); 
  op->save("./subject_eit/gh_plane_after_opt.g2o"); 
  cout<<"test_plane_3d.cpp: complete test1()"<<endl;

  return 1;
}

bool add_plane_node_incrementally(Plane3D* p, int id, SparseOptimizer* op)
{
  static int offset_id = 5000; 
  static int floor_id = offset_id + 1; 
  
  static bool first_node = true; 
  Plane3D g_floor = ((g2o::VertexSE3*)op->vertices()[id])->estimate() * (*p);

  int num_edges = op->edges().size();
  int num_nodes = op->vertices().size();
  int lambda_f = num_edges / num_nodes; 
  lambda_f = lambda_f > 0 ? lambda_f : 1; 

  if(isFloor(&g_floor))
  {
    if(first_node)
    {
      // offset vertex 
      Eigen::Isometry3d offset3d = Eigen::Isometry3d::Identity();
      g2o::VertexSE3 * offsetVertex = new g2o::VertexSE3; 
      offsetVertex->setId(offset_id); 
      offsetVertex->setEstimate(offset3d); 
      offsetVertex->setFixed(true);
      op->addVertex(offsetVertex); 
         
      // add floor plane and prior initial wall planes
      g2o::VertexPlane* p = new g2o::VertexPlane; 
      p->setEstimate(g_floor); 
      p->setId(floor_id); 
      p->setFixed(true);
      op->addVertex(p);
      first_node = false;
    }else
    {
      g2o::EdgeSE3PlaneSensorCalib * e = new g2o::EdgeSE3PlaneSensorCalib; 
      e->vertices()[0] = op->vertices()[id]; 
      e->vertices()[1] = op->vertices()[floor_id];
      e->vertices()[2] = op->vertices()[offset_id]; 
      e->setMeasurement(*p); 
      Eigen::Matrix3d m = Eigen::Matrix3d::Zero(); 
      m(0,0) = 2 * lambda_f;   // aminth no contribution
      m(1,1) = 10 * lambda_f;   // elevation 
      m(2,2) = 20 * lambda_f;  // distance 
      e->setInformation(m); 
      op->addEdge(e);
    }
  }
  return true;
}


void addPlaneNode(SparseOptimizer* op, map<int, Plane3D*>& p_set)
{
  int offset_id = 5000; 
  int floor_id = offset_id + 1; 

  // offset vertex 
  Eigen::Isometry3d offset3d = Eigen::Isometry3d::Identity();
  g2o::VertexSE3 * offsetVertex = new g2o::VertexSE3; 
  offsetVertex->setId(offset_id); 
  offsetVertex->setEstimate(offset3d); 
  offsetVertex->setFixed(true);
  op->addVertex(offsetVertex); 

  // first plane set as floor plane 
  map<int, Plane3D*>::iterator it = p_set.begin(); 
  while(it!=p_set.end())
  {
    Plane3D g_floor = ((g2o::VertexSE3*)op->vertices()[it->first])->estimate() * (*(it->second));
    // if(isFloor(it->second))
    if(isFloor(&g_floor))
    {
      // first time add into as floor 
      g2o::VertexPlane* p = new g2o::VertexPlane; 
      // p->setEstimate(*(it->second));
      p->setEstimate(g_floor);
      p->setId(floor_id); 
      p->setFixed(true);
      op->addVertex(p);
      break; 
    }
    ++it;
  }
  
  // then add edges into the graph 
  while(it != p_set.end())
  {
    Plane3D g_floor = ((g2o::VertexSE3*)op->vertices()[it->first])->estimate() * (*(it->second));

    // if(isFloor(it->second))
    if(isFloor(&g_floor))
    {
      g2o::EdgeSE3PlaneSensorCalib * e = new g2o::EdgeSE3PlaneSensorCalib; 
      e->vertices()[0] = op->vertices()[it->first]; 
      e->vertices()[1] = op->vertices()[floor_id];
      e->vertices()[2] = op->vertices()[offset_id]; 
      e->setMeasurement(*(it->second)); 
      Eigen::Matrix3d m = Eigen::Matrix3d::Zero(); 
      m(0,0) = 20;   // aminth no contribution
      m(1,1) = 20;   // elevation 
      m(2,2) = 100;  // distance 
      e->setInformation(m); 
      op->addEdge(e);
    }
    ++it; 
  }
}

bool loadPCAtNode(CloudPtr& pc, int nid)
{
  bool ret = true;
  stringstream ss; 
  ss<<"./subject_eit/pcds/quicksave_"<<std::setfill('0')<<std::setw(4)<<nid<<".pcd";
  if(pcl::io::loadPCDFile(ss.str(), *pc) == 0)
  {
    // cout<<"test_mul_plane_3d.cpp: succeed to load node "<<nid<<" 's point cloud"<<endl;
  }else
  {
    cout<<"test_mul_plane_3d.cpp: failed to load node "<<nid<<" 's point cloud"<<endl;
    cout<<"ss = "<<ss.str()<<endl;
    ret = false;
    // return p_set; 
  }
  return ret;
}

map<int, Plane3D*> extractPlanes(SparseOptimizer* op)
{
  int nid; 
  int i_thresh_number = 5000;   // at least 5000 points on this plane
  // pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr g_pc(new pcl::PointCloud<pcl::PointXYZ>);
  CloudPtr pc(new Cloud);    // each node's local point cloud 
  CloudPtr g_pc(new Cloud);  // global point cloud 
  // CPlaneSet * p_set = new CPlaneSet; 
  CPlane *p_plane = new CPlane; 
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  map<int, Plane3D*> p_set; 
  ofstream ouf("plane_param.log");

  for(HyperGraph::VertexIDMap::iterator it = op->vertices().begin(); it!= op->vertices().end(); ++it)
  {
    VertexSE3 * pn = (VertexSE3*)(it->second); 
    nid = pn->id(); 
    stringstream ss; 
    ss<<"./subject_eit/pcds/quicksave_"<<std::setfill('0')<<std::setw(4)<<nid<<".pcd";
    if(pcl::io::loadPCDFile(ss.str(), *pc) == 0)
    {
      cout<<"test_plane_3d.cpp: succeed to load node "<<nid<<" 's point cloud"<<endl;
    }else
    {
      cout<<"test_plane_3d.cpp: failed to load node "<<nid<<" 's point cloud"<<endl;
      cout<<"ss = "<<ss.str()<<endl;
      return p_set; 
    }
    
    // extract plane 
    p_plane->computePCL(pc, inliers); 
    if(inliers->indices.size() < i_thresh_number)
    {
      continue;  // number of the points must large than i_thresh_number 
    }
  
    cout<<"test_plane_3d.cpp: node "<<it->first<<" has floor plane "<<endl;

    CloudPtr p_pc(new Cloud);
    extractCloud<Point>(pc, p_pc, inliers); 
    // transform this point cloud into global a
    Eigen::Isometry3d itrans = pn->estimate(); 
    // Eigen::Transform<double, 3, Eigen::Affine> tT = itrans; 
    // pcl::transformPointCloud(*p_pc, *p_pc, tT); 
    // *g_pc = *g_pc + *p_pc;

    // Plane3D 
    Eigen::Vector4d v; 
    v << p_plane->nx_, p_plane->ny_, p_plane->nz_, p_plane->d1_; 
    ouf<<v(0)<<" "<<v(1)<<" "<<v(2)<<" "<<v(3)<<" ";   // plane parameter in local reference 
    Plane3D* tmp_p = new Plane3D();
    tmp_p->fromVector(v); 
    // *tmp_p = itrans * (*tmp_p); 
    // v = tmp_p->toVector(); 
    ouf<<v(0)<<" "<<v(1)<<" "<<v(2)<<" "<<v(3)<<endl;   // plane parameter in local reference 
    p_set[nid] = tmp_p;  // extracted plane in node nid
  }
  
/// for display 
  if(0)
  {
    // voxel grid downsampling 
    CloudPtr dg_pc(new Cloud); 
    filterPointCloud<Point>(g_pc, dg_pc, 0.02);

    // 3. show the global point cloud 
    CVTKViewer<Point> viewer;
    viewer.getViewer()->addCoordinateSystem(0.2, 0,0,0);
    viewer.addPointCloud(dg_pc, "planes"); 
    while(!viewer.stopped())
    {
      viewer.runOnce(); 
      usleep(100000);
    }
  }
  return p_set;
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


template<typename PointT>
void extractCloud(typename pcl::PointCloud<PointT>::Ptr& in, 
                typename pcl::PointCloud<PointT>::Ptr& out,
                pcl::PointIndices::Ptr& index, bool inv)
{
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(in);
  extract.setIndices(index);
  extract.setNegative(inv);
  extract.filter(*out);
}

template<typename PointT>
void filterPointCloud(typename pcl::PointCloud<PointT>::Ptr& in,typename pcl::PointCloud<PointT>::Ptr& out, double _voxel_size)
{
    // typename pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
    // passthrough filter
    // pcl::PassThrough<PointT > pass;
    // pass.setInputCloud(in);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.0, _depth_limit);
    // pass.filter(*tmp);
    // g_passFilter<PointT>(in, tmp, 0.0, _depth_limit, "z");

    // voxelgrid filter
    pcl::VoxelGrid<PointT> vog;
    vog.setInputCloud(in); 
    vog.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
    vog.filter(*out);
}

bool isFloor(Plane3D* p)
{
  Eigen::Vector3d floor_normal(0, 0, 1); // so dot product is the n.dot(floor) = nz
  double COS15 = cos(15.*M_PI/180.); 
  double COS30 = cos(30.*M_PI/180.); 
  Eigen::Vector4d v; 
  v = p->toVector(); 
  // if(fabs(v(2)) > COS15)
  if(fabs(v(2)) > COS30)
    return true;
  return false;
}

void initTypes()
{
  VertexSE3 * v1 = new VertexSE3; 
  EdgeSE3 * e1 = new EdgeSE3;
}


