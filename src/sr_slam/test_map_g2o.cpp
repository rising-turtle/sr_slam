/*
 *  Feb. 2, 2016, David Z
 *
 *  Build a pcd and octomap map from g2o structure 
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

// octomap 
#include "ColorOctreeImpl.h"

using namespace g2o;

template<typename PointT>
void filterPointCloud(typename pcl::PointCloud<PointT>::Ptr& in,
    typename pcl::PointCloud<PointT>::Ptr& out, double _voxel_size = 0.01);

SparseOptimizer* createOptimizeClass(); 
ColorOctreeImpl* createOctreeImpl();

void toVectorQT(float p[7], Eigen::Isometry3d&);

int test1(); 

string input_g2o("./subject_eit/graph.g2o");
string output_pcd("map3d.pcd");
string output_ot("octomap3d.bt");
int main(int argc, char* argv[])
{
  if(argc >= 2)
    input_g2o = string(argv[1]);

  test1(); 
  return 0; 
}

// p[7], x,y,z, qw, qx, qy, qz
void toVectorQT(float p[7], Eigen::Isometry3d& m)
{
  Eigen::Quaterniond q(m.matrix().topLeftCorner<3,3>());
  q.normalize();
  p[3] = q.w(); p[4] = q.x(); p[5] = q.y(); p[6] = q.z(); 
  Eigen::Vector3d v = m.translation();
  p[0] = v(0); p[1] = v(1); p[2] = v(2);
  return ;
}

int test1()
{
  // 1. load graph structure 
  SparseOptimizer * op = createOptimizeClass(); 
  // if(op->load("./subject_eit/graph.g2o"))
  if(op->load(input_g2o.c_str()))
  {
    cout<<"test_plane_3d.cpp: succeed to load g2o_file!"<<endl; 
    ((g2o::VertexSE3*)op->vertices()[0])->setFixed(true);
  }else
  {
    cout<<"test_plane_3d.cpp: failed to load g2o_file!"<<endl;
  }

  // 2. load pcds 
  CloudPtr pc(new Cloud);    // each node's local point cloud 
  CloudPtr g_pc(new Cloud);  // global point cloud 
  CloudPtr p_pc(new Cloud);

  ColorOctreeImpl* ot_impl = createOctreeImpl();
  float pose[7]; // x, y, z, qw, qx, qy, qz

 for(HyperGraph::VertexIDMap::iterator it = op->vertices().begin(); it!= op->vertices().end(); ++it)
  {
    VertexSE3 * pn = (VertexSE3*)(it->second); 
    int nid = pn->id(); 
    stringstream ss; 
    ss<<"./subject_eit/pcds/quicksave_"<<std::setfill('0')<<std::setw(4)<<nid<<".pcd";
    if(pcl::io::loadPCDFile(ss.str(), *pc) == 0)
    {
      cout<<"test_map_g2o.cpp: succeed to load node "<<nid<<" 's point cloud"<<endl;
    }else
    {
      cout<<"test_map_g2o.cpp: failed to load node "<<nid<<" 's point cloud"<<endl;
      cout<<"ss = "<<ss.str()<<endl;
      continue; 
    }
    CloudPtr dg_pc(new Cloud); 
    filterPointCloud<Point>(pc, dg_pc, 0.05);

    Eigen::Isometry3d itrans = pn->estimate(); 
    Eigen::Transform<double, 3, Eigen::Affine> tT = itrans; 
    pcl::transformPointCloud(*dg_pc, *p_pc, tT); 
    *g_pc = *g_pc + *p_pc;
    
    // insert into octomap
    // 1. get the pose of this pc
    dg_pc->sensor_origin_ = Eigen::Vector4f::Zero(); 
    toVectorQT(pose, itrans); 
    ot_impl->insertPointCloud(*dg_pc, pose, 5); 
    ot_impl->updateInnerOccupancy();
  }

    // 3. show it 
    CloudPtr dg_pc(new Cloud); 
    filterPointCloud<Point>(g_pc, dg_pc, 0.1);

    pcl::io::savePCDFile(output_pcd, *dg_pc, true); 
    ot_impl->writeBinary(output_ot.c_str());

    // 3. show the global point cloud 
    CVTKViewer<Point> viewer;
    viewer.getViewer()->addCoordinateSystem(0.2, 0,0,0);
    viewer.addPointCloud(dg_pc, "planes"); 
    while(!viewer.stopped())
    {
      viewer.runOnce(); 
      usleep(100000);
    }
  return 1;
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

void initTypes()
{
  VertexSE3 * v1 = new VertexSE3; 
  EdgeSE3 * e1 = new EdgeSE3;
}



ColorOctreeImpl* createOctreeImpl()
{
  ColorOctreeImpl * ot_impl = new ColorOctreeImpl(0.05); 
  ot_impl->setClampingThresMin(0.001); 
  ot_impl->setClampingThresMax(0.999); 
  ot_impl->setOccupancyThres(0.8); 
  ot_impl->setProbHit(0.9); 
  ot_impl->setProbMiss(0.4); 
  return ot_impl;
}

