/*
 *  Jan. 31, 2016 David Z 
 *
 *  plane extraction and transform them into the same coordinate 
 *
 *  use not only floor plane, but also wall plane
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
void test3(string ); // check the behavior of graph optimization 
void test4(); // save the middle graph optimization results

bool add_plane_node_incrementally(CloudPtr& in, int id, SparseOptimizer* , HyperGraph::VertexSet&, HyperGraph::EdgeSet&); 
bool loadPCAtNode(CloudPtr& in, int id);
void inversePlane3D(Plane3D*);  // inverse itself 
bool needsInverse(Plane3D*); // whether it needs reverse

map<int, vector<Plane3D*> > extractPlanes(SparseOptimizer* op);
void add_plane_node_impl(SparseOptimizer* op, Plane3D*, int node_id, int plane_id, int offset_id);
// void addPlaneNode(SparseOptimizer* op, map<int, Plane3D*>& p_set); 
void addPlaneNode(SparseOptimizer* op, map<int, vector<Plane3D*> >& p_set); 
bool isFloor(Plane3D* p);
int isWall(Plane3D* p, map<int, Plane3D*>& w_ps, float cx = 0, float cy = 0);

void init_eit_wall_ps(map<int, Plane3D*>& , int ); 
void init_etas_wall_ps(map<int, Plane3D*>&, int);
void add_wall_ps(SparseOptimizer* op, map<int, Plane3D*>& p_set);

void display_plane(Plane3D* p, string pre="");

// TODO: this threshold stuff is not robust 
int large_bias_id = -100;
int floor_plane_id = -100;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mul_plane"); 
  ros::NodeHandle n; 

  // test1();
  // test2();
  string g2o_file("./g2o_increments/g_400_before.g2o");
  if(argc <= 1)
  {
  }else
  {
    g2o_file = string(argv[1]);
  }
  // test3(g2o_file);
  test4(); 
  return 0; 
}

void test4()
{
  int cnt = 0;
  int TMP_N = 10;
  bool first_round = true;

  HyperGraph::VertexSet verticesAdded; 
  HyperGraph::EdgeSet   edgesAdded; 

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
  
  // 2. incrementally add node 
  SparseOptimizer * pg = createOptimizeClass(); 
  
  CloudPtr pc(new Cloud);    // each node's local point cloud 
  VertexSE3 * last_op = 0; 
  VertexSE3 * last_pg = 0;
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
      continue;
    }else
    {
      Eigen::Isometry3d inc_odo = last_op->estimate().inverse()*p1->estimate(); 
      p2->setEstimate(last_pg->estimate()*inc_odo);
      pg->addVertex(p2); 
      verticesAdded.insert(p2);
      // copy edges in p1 to p2 
      for(HyperGraph::EdgeSet::iterator it = p1->edges().begin(); it!= p1->edges().end(); ++it)
      {
        // cout<<"vin->id = "<<vin->id()<<" plast->edge[0].id2 = "<<(*it)->vertices()[1]->id()<<endl;
        if((*it)->vertices()[0]->id() < p1->id()) // find the previous edge
        {
          EdgeSE3* e_in = dynamic_cast<EdgeSE3*>(*it); 
          EdgeSE3* e2 = copyEdge<EdgeSE3, EdgeSE3>(e_in, pg);
          pg->addEdge(e2);
          edgesAdded.insert(e2); 
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
    if(b_getPC && !add_plane_node_incrementally(pc, p1->id(), pg, verticesAdded, edgesAdded))
     // continue;
    {
      continue; 
    }

    // 3. optimize the graph
    // pg->setVerbose(true);

    int current_node_id = p2->id();
    ++ cnt; 
    stringstream s;
    s<<"./g2o_increments/g_"<<cnt<<"_before.g2o";
    if(cnt % TMP_N == 0)
    {
       printf("save g2o into %s\n", s.str().c_str());
       pg->save(s.str().c_str()); 
    }
    // pg->initializeOptimization(); 
    // int result_op = pg->optimize(5); 
    // printf(KRED "test_mul_plane_3d.cpp: optimize return %d, at cnt = %d\n", result_op, cnt);
    if(cnt % TMP_N == 0)
    {
      // printf("clear current g2o\n"); 
      // pg->clear();
      // printf("reload g2o from %s\n", s.str().c_str()); 
      // pg->load(s.str().c_str()); 

      // map<int, Plane3D*> wall_ps; 
      // init_eit_wall_ps(wall_ps, 5001);   // working in eit f5 
      // add_wall_ps(pg, wall_ps);


      // printf("try to optimize again!\n"); 
      if(first_round)
      {
        // pg->initializeOptimization(); 
      }else
      {
        // pg->updateInitialization(verticesAdded, edgesAdded);
      }
     
      // printf("after g2o optimization, try to save again!\n");
      // int result_op = pg->optimize(5, !first_round);
     
      pg->initializeOptimization(); 
      pg->optimize(5);

      verticesAdded.clear(); 
      edgesAdded.clear();

      first_round = false;
      stringstream s; 
      s<<"./g2o_increments/g_"<<cnt<<"_after.g2o";
      pg->save(s.str().c_str());
    }

    last_pg = (VertexSE3*)(pg->vertices()[current_node_id]);
  }
  pg->save("./subject_eit/new_incremental_plane3d_opt_before.g2o");
  // delete op; 
  pg->setVerbose(true);
  pg->initializeOptimization(); 
  int result_op = pg->optimize(5); 
  pg->save("./subject_eit/new_incremental_plane3d_opt.g2o");
  return ;

}

void test2()
{
  ofstream ouf("./subject_eit/mul_plane_time.log");

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
  
  // 2. incrementally add node 
  SparseOptimizer * pg = createOptimizeClass(); 
  
  CloudPtr pc(new Cloud);    // each node's local point cloud 
  VertexSE3 * last_op = 0; 
  VertexSE3 * last_pg = 0;
  for(HyperGraph::VertexIDMap::iterator it = op->vertices().begin(); it!= op->vertices().end(); ++it)
  {
    // 2.1 add vertex and its edges into pg 
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
      // copy edges in p1 to p2 
      for(HyperGraph::EdgeSet::iterator it = p1->edges().begin(); it!= p1->edges().end(); ++it)
      {
        // cout<<"vin->id = "<<vin->id()<<" plast->edge[0].id2 = "<<(*it)->vertices()[1]->id()<<endl;
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

    // 2.2 extract planes from this node 
    // load the point cloud of this node 
    bool b_getPC = loadPCAtNode(pc, p1->id());
    // if(!loadPCAtNode(pc, p1->id())) 
    //  continue; 
    
    // time for plane associate 
    // 2.3 add plane into the graph 
     TTimeStamp before_plane = getCurrentTime();  
     // TODO: recover this 
    // if(b_getPC && !add_plane_node_incrementally(pc, p1->id(), pg))
     // continue;
    {}
    TTimeStamp after_plane = getCurrentTime(); 
    ouf<<timeDifference(before_plane, after_plane)<<" ";

    // 3. optimize the graph
    // pg->setVerbose(true);
    TTimeStamp before_op = after_plane; //  getCurrentTime(); 
    pg->initializeOptimization(); 
    pg->optimize(5); 
    TTimeStamp after_op = getCurrentTime(); 
    ouf<<timeDifference(before_op, after_op)<<" "<<timeDifference(before_plane, after_op)<<endl;
  }
  
  pg->save("./subject_eit/incremental_plane3d_opt.g2o");
  ouf.close();
  return ;
}

void test3(string g2o_file)
{
  // 1. load graph structure 
  SparseOptimizer * op = createOptimizeClass(); 
  // if(op->load("./subject_eit/incremental_plane3d_opt.g2o"))
  if(op->load(g2o_file.c_str()))
  {
    cout<<"test_mul_plane_3d.cpp: succeed to load g2o_file!"<<endl; 
    // ((g2o::VertexSE3*)op->vertices()[0])->setFixed(true);
  }else
  {
    cout<<"test_mul_plane_3d.cpp: failed to load g2o_file!"<<endl;
  }
  op->setVerbose(true);
  op->initializeOptimization(); 
  op->optimize(5); 
  op->save("./subject_eit/check_incremental_plane3d_opt.g2o");
}

int test1()
{
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

  // 2. extract plane for each node, and transform them into a single point cloud 
  // map<int, Plane3D*> p_set = extractPlanes(op); 
  map<int, vector<Plane3D*> > p_set = extractPlanes(op);

  // 3. add plane node into the graph 
  addPlaneNode(op, p_set); 
  op->save("./subject_eit/m_plane_before_opt.g2o"); 
  op->initializeOptimization();
  op->setVerbose(true); 
  op->optimize(10); 
  op->save("./subject_eit/m_plane_after_opt.g2o"); 
  cout<<"test_mul_plane_3d.cpp: complete test1()"<<endl;

  return 1;
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

bool add_plane_node_incrementally(CloudPtr& pc, int id, SparseOptimizer* op, HyperGraph::VertexSet& verticesAdded, HyperGraph::EdgeSet& edgesAdded)
{
  CPlaneSet p_pSet; 
  int num_p = p_pSet.extractPlanes(pc); 
  if(num_p == 0) return false;

  vector<Plane3D*> pVec(num_p, NULL); 
  vector<float> pCenter_X(num_p, NULL); 
  vector<float> pCenter_Y(num_p, NULL); 
  Eigen::Vector4d v; 
  for(int i=0; i<num_p; ++i)
  {
    CPlane* p_plane = p_pSet.planeAt(i); 
    // Plane3D 
    v << p_plane->nx_, p_plane->ny_, p_plane->nz_, p_plane->d1_; 
    // ouf<<v(0)<<" "<<v(1)<<" "<<v(2)<<" "<<v(3)<<" ";   // plane parameter in local reference 
    Plane3D* tmp_p = new Plane3D();
    tmp_p->fromVector(v); 
    pVec[i] = tmp_p; 
    // ouf<<v(0)<<" "<<v(1)<<" "<<v(2)<<" "<<v(3)<<" ";// plane parameter in local reference 
    
    pCenter_X[i] = p_plane->cx_; 
    pCenter_Y[i] = p_plane->cy_;
  }
  
  static bool first_node = true; 
  static int offset_id = 5000; 
  static int floor_id = offset_id + 1;
  floor_plane_id = floor_id; 
  static map<int, Plane3D*> wall_ps; 

  bool b_add_new_plane = false;
  if(first_node) // first node set up floor plane and wall planes
  { 
    Plane3D g_floor = ((g2o::VertexSE3*)op->vertices()[id])->estimate() * (*pVec[0]);
    if(isFloor(&g_floor))
    {
      // offset vertex 
      Eigen::Isometry3d offset3d = Eigen::Isometry3d::Identity();
      g2o::VertexSE3 * offsetVertex = new g2o::VertexSE3; 
      offsetVertex->setId(offset_id); 
      offsetVertex->setEstimate(offset3d); 
      offsetVertex->setFixed(true);
      op->addVertex(offsetVertex); 
      verticesAdded.insert(offsetVertex); 

      // add floor plane and prior initial wall planes
      g2o::VertexPlane* p = new g2o::VertexPlane; 
      p->setEstimate(g_floor); 
      p->setId(floor_id); 
      p->setFixed(true);
      op->addVertex(p);
      verticesAdded.insert(p);

      // init wall planes, add them into graph 
      init_eit_wall_ps(wall_ps, floor_id); 
      // init_etas_wall_ps(wall_ps, floor_id);
      add_wall_ps(op, wall_ps);

      first_node = false; 
    }
  }else
  {
    // add these planes into the graph 
    for(int i=0; i<pVec.size(); i++)
    {
      Plane3D plane_in_w = ((g2o::VertexSE3*)op->vertices()[id])->estimate()*(*pVec[i]); 
      if(isFloor(&plane_in_w))
      {
        add_plane_node_impl(op, pVec[i], id, floor_id, offset_id);
        b_add_new_plane= true;
      }else
      {
        int wall_id = isWall(&plane_in_w, wall_ps, pCenter_X[i], pCenter_Y[i]);
        if(wall_id > 0)
        {
          add_plane_node_impl(op, pVec[i], id, wall_id, offset_id); 
          b_add_new_plane = true;
        }else // flip it, give it another chance 
        {
          inversePlane3D(&plane_in_w); 
          inversePlane3D(pVec[i]); 
          wall_id = isWall(&plane_in_w, wall_ps, pCenter_X[i], pCenter_Y[i]); 
          if(wall_id > 0)
          {
            add_plane_node_impl(op, pVec[i], id, wall_id, offset_id); 
            b_add_new_plane = true;
          }
        }

        if(wall_id > 0)
        {
          // printf(KYEL "node %d associate with wall %d with cx= %f \n", id, wall_id, pCenter_X[i]); 
        }
      }
    }
  }
  return b_add_new_plane;
}


void addPlaneNode(SparseOptimizer* op, map<int, vector<Plane3D*> >& p_set)
{
  int offset_id = 5000; 
  int floor_id = offset_id + 1; 

  ofstream ouf("wall_plane_param.log");

  // offset vertex 
  Eigen::Isometry3d offset3d = Eigen::Isometry3d::Identity();
  g2o::VertexSE3 * offsetVertex = new g2o::VertexSE3; 
  offsetVertex->setId(offset_id); 
  offsetVertex->setEstimate(offset3d); 
  offsetVertex->setFixed(true);
  op->addVertex(offsetVertex); 

  // first plane set as floor plane 
  // map<int, Plane3D*>::iterator it = p_set.begin(); 
  map<int, vector<Plane3D*> >::iterator it = p_set.begin();
  while(it!=p_set.end())
  {
    Plane3D* f_floor = it->second[0]; 
    // Plane3D g_floor = ((g2o::VertexSE3*)op->vertices()[it->first])->estimate() * (*(it->second));
    Plane3D g_floor = ((g2o::VertexSE3*)op->vertices()[it->first])->estimate() * (*f_floor);
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
  
  // init wall planes, add them into graph
  map<int, Plane3D*> wall_ps; 
  init_eit_wall_ps(wall_ps, floor_id);   // working in eit f5 
  // init_etas_wall_ps(wall_ps, floor_id);     // working in etas f5
  add_wall_ps(op, wall_ps);

 // then add edges into the graph 
  while(it != p_set.end())
  {
    // Plane3D g_floor = ((g2o::VertexSE3*)op->vertices()[it->first])->estimate() * (*(it->second));
    vector<Plane3D*>& pset = it->second;
    for(int i=0; i<pset.size(); i++)
    {
      Plane3D g_floor = ((g2o::VertexSE3*)op->vertices()[it->first])->estimate() * (*pset[i]);
      // if(isFloor(it->second))
      if(isFloor(&g_floor))
      {
        add_plane_node_impl(op, pset[i], it->first, floor_id, offset_id);
      }else
      {
          
          int wall_id = isWall(&g_floor, wall_ps); 
          if( wall_id >0 )
          {
            ROS_ERROR("test_mul_plane_3d.cpp: node %d has a wall id = %d ", it->first, wall_id);
            add_plane_node_impl(op, pset[i], it->first, wall_id, offset_id);
          }
          Eigen::Vector4d v; 
          v = g_floor.toVector(); 
          Eigen::Vector3d cv; cv(0)= v(0); cv(1) = v(1); cv(2) = v(2); // v<3,1>(0,0);
          ouf<<it->first<<" "<<R2D(g_floor.azimuth(cv))<<" "<<R2D(g_floor.elevation(cv))<<" "<<g_floor.distance()<<endl;
      }
    }
    ++it; 
  }
}

void add_wall_ps(SparseOptimizer* op, map<int, Plane3D*>& p_set)
{
  map<int, Plane3D*>::iterator it = p_set.begin(); 
  while(it != p_set.end())
  {
    g2o::VertexPlane* p = new g2o::VertexPlane; 
    p->setEstimate(*(it->second)); 
    p->setId(it->first);
    p->setFixed(true);
    if(op->vertices().find(it->first) == op->vertices().end())
      op->addVertex(p);

    ++it;
  }
  return ;
}

void add_plane_node_impl(SparseOptimizer* op, Plane3D* p, int node_id, int plane_id, int offset_id)
{
  g2o::EdgeSE3PlaneSensorCalib * e = new g2o::EdgeSE3PlaneSensorCalib; 
  e->vertices()[0] = op->vertices()[node_id]; // [it->first]; 
  e->vertices()[1] = op->vertices()[plane_id]; // [floor_id];
  e->vertices()[2] = op->vertices()[offset_id]; 
  // e->setMeasurement(*(it->second)); 
  e->setMeasurement(*p);
  Eigen::Matrix3d w_m = Eigen::Matrix3d::Zero(); 
  w_m(0,0) = 10;  // aminth contribution
  w_m(1,1) = 5;   // elevation 
  w_m(2,2) = 10; 

  Eigen::Matrix3d f_m = Eigen::Matrix3d::Zero(); 
  f_m(0,0) = 0; // aminth no contribution 
  f_m(1,1) = 100; // elevation 
  f_m(2,2) = 10000; 

  if(plane_id == floor_plane_id)
    e->setInformation(f_m);
  else
    e->setInformation(w_m); 
  op->addEdge(e);
  return ;
}

void init_etas_wall_ps(map<int, Plane3D*>& wall_ps, int floor_id)
{
  static int cur_id = floor_id + 2; 
  // add wall planes
  {
    Eigen::Vector4d ny;
    ny << 0, 1, 0, 0;
    Eigen::Vector4d nx; 
    nx << 1, 0, 0, 0;
    
    // start at [2080, 275], sx = 0.0204, sy = 0.0199
    // wall 1, y = 230, yw = 0.8955 
    Plane3D * pw1 = new Plane3D; 
    ny(3) = -0.8955; 
    pw1->fromVector(ny); 
    wall_ps[cur_id++] = pw1; 

    // wall 2, y = 304, yw = -0.5771
    Plane3D* pw2 = new Plane3D; 
    ny(3) = 0.5771; 
    pw2->fromVector(ny); 
    wall_ps[cur_id++] = pw2; 
  
    // wall 3, y = 1610, yw = -26.5665
    Plane3D* pw3 = new Plane3D; 
    ny(3) = 26.5665; 
    pw3->fromVector(ny); 
    wall_ps[cur_id++] = pw3; 
    large_bias_id = cur_id - 1;

    // wall 4, x = 2330, xw = 5.1
    Plane3D* pw4 = new Plane3D; 
    nx(3) = -5.1;
    pw4->fromVector(nx); 
    wall_ps[cur_id++] = pw4; 

    // wall 5, x = 2415, xw = 6.834
    Plane3D* pw5 = new Plane3D;
    nx(3) = -6.834;
    pw5->fromVector(nx); 
    wall_ps[cur_id++] = pw5;
  }
  return ;
}

void init_eit_wall_ps(map<int, Plane3D*>& wall_ps, int floor_id)
{
  static int cur_id = floor_id + 2; 
  // add wall planes 
  {
    Eigen::Vector4d ny;
    ny << 0, 1, 0, 0;
    Eigen::Vector4d nx; 
    nx << 1, 0, 0, 0;
    // start at [2680, 550], s = 0.0196  
    // wall 1, y = 505, yw = -0.882
    Plane3D *pw1 =  new Plane3D; 
    ny(3) = 0.882; 
    pw1->fromVector(ny); 
    wall_ps[cur_id++] = pw1; 

    // wall 2, y = 582, yw = 0.672
    Plane3D* pw2 = new Plane3D; 
    ny(3) = -0.672; 
    pw2->fromVector(ny); 
    wall_ps[cur_id++] = pw2; 

    // wall 3, x = 2335, xw = 6.762
    Plane3D* pw3 = new Plane3D; 
    nx(3) = -6.762; 
    pw3->fromVector(nx); 
    wall_ps[cur_id++] = pw3;

    // wall 4, y = 945, yw = 7.742
    Plane3D* pw4 = new Plane3D; 
    ny(3) = -7.742; 
    pw4->fromVector(ny*-1); 
    wall_ps[cur_id++] = pw4; 

    // wall 5, y = 1050, yw = 9.8
    Plane3D* pw5 = new Plane3D; 
    ny(3) = -9.8; 
    pw5->fromVector(ny); 
    wall_ps[cur_id++] = pw5;
  }

 return ;
}


map<int, vector<Plane3D*> > extractPlanes(SparseOptimizer* op)
{
  int nid; 
  int i_thresh_number = 5000;   // at least 5000 points on this plane
  // pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr g_pc(new pcl::PointCloud<pcl::PointXYZ>);
  CloudPtr pc(new Cloud);    // each node's local point cloud 
  CloudPtr g_pc(new Cloud);  // global point cloud 
  // CPlaneSet * p_set = new CPlaneSet; 
  // CPlane *p_plane = new CPlane; 
  CPlaneSet * p_pSet = new CPlaneSet; 
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // map<int, Plane3D*> p_set; 
  map<int, vector<Plane3D*> > p_set;
  // ofstream ouf("wall_plane_param.log");

  for(HyperGraph::VertexIDMap::iterator it = op->vertices().begin(); it!= op->vertices().end(); ++it)
  {
    VertexSE3 * pn = (VertexSE3*)(it->second); 
    nid = pn->id(); 
    stringstream ss; 
    ss<<"./subject_eit/pcds/quicksave_"<<std::setfill('0')<<std::setw(4)<<nid<<".pcd";
    if(pcl::io::loadPCDFile(ss.str(), *pc) == 0)
    {
      // cout<<"test_mul_plane_3d.cpp: succeed to load node "<<nid<<" 's point cloud"<<endl;
    }else
    {
      cout<<"test_mul_plane_3d.cpp: failed to load node "<<nid<<" 's point cloud"<<endl;
      cout<<"ss = "<<ss.str()<<endl;
      return p_set; 
    }
    
    // extract plane 
    // p_plane->computePCL(pc, inliers); 

    /*
    if(inliers->indices.size() < i_thresh_number)
    {
      continue;  // number of the points must large than i_thresh_number 
    }*/
    
    int num_p = p_pSet->extractPlanes(pc); 
    // cout<<"test_mul_plane_3d.cpp: node "<<it->first<<" has "<<num_p<<" planes"<<endl;
    if(num_p == 0) continue;
    vector<Plane3D*> pVec(num_p, NULL); 
    Eigen::Vector4d v; 
    for(int i=0; i<num_p; ++i)
    {
      CPlane* p_plane = p_pSet->planeAt(i); 
         // Plane3D 
      v << p_plane->nx_, p_plane->ny_, p_plane->nz_, p_plane->d1_; 
      // ouf<<v(0)<<" "<<v(1)<<" "<<v(2)<<" "<<v(3)<<" ";   // plane parameter in local reference 
      Plane3D* tmp_p = new Plane3D();
      tmp_p->fromVector(v); 
      pVec[i] = tmp_p; 
      // ouf<<v(0)<<" "<<v(1)<<" "<<v(2)<<" "<<v(3)<<" ";// plane parameter in local reference 
    }
    p_set[nid] = pVec;  // extracted plane in node nid
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


bool needsInverse(Plane3D* p) // whether it needs reverse
{
  static Plane3D wp; 
  Eigen::Vector4d wv,wu; 
  wv<< 0, 1, 0, 0; 
  wu<< 1, 0, 0, 0;
  wp.fromVector(wv); 
  Eigen::Vector3d e = wp.ominus(*p);   
  if(e(0) > M_PI/2. || e(0) < -M_PI/2.)
    return true; 
  wp.fromVector(wu); 
  e = wp.ominus(*p);
  if(e(0) > M_PI/2. || e(0) < -M_PI/2.)
    return true; 
  return false;
}

void inversePlane3D(Plane3D* p)  // inverse itself 
{
  Eigen::Vector4d v = p->toVector(); 
  v= v*-1;
  p->fromVector(v);
}

int isWall(Plane3D* p, map<int, Plane3D*>& w_ps, float cx, float cy)
{
  static const double BIG_N = 100000;
  static double T_Yaw = M_PI/10.; 
  static double T_Roll = M_PI/5.; 
  static double T_dis = 0.4; // 0.4;  // 0.1 for etas
  int ret_id = -1; 
  map<int, Plane3D*>::iterator it = w_ps.begin();
  double min_dis = BIG_N;
  double next_min_dis = BIG_N;
    
  // display_plane(p," Input Plane Parameters: ");
  

  while(it != w_ps.end())
  {
    Plane3D* pw = it->second; 
    Eigen::Vector3d e = pw->ominus(*p); 
    if(it->first == 5006)
    {
      // printf(KWHT "mul3d.cpp: match with 5006 e: %f %f %f\n", R2D(e(0)), R2D(e(1)), e(2));
    }
    
    // here to distinguish two wall planes  : 5006 RIGHT, 5007 LEFT
    if(it->first == 5006 && cx > 0) // RIGHT 
    {
      // printf(KCYN "test_mul_plane_3d.cpp: RIGHT wall but cx = %f < 0 \n", cx); 
      // ret_id = -1;
      ++it;
      continue; 
    }

    if(it->first == 5007 && cx < 0) // LEFT
    {
      // printf(KBLU "test_mul_plane_3d.cpp: LEFT wall but cx = %f > 0\n", cx); 
      // ret_id = -1;
      ++it;
      continue; 
    }

    if(fabs(e(0)) < T_Yaw && fabs(e(1)) < T_Roll)
    {
      if(min_dis > fabs(e(2)))
      {
        if(min_dis != BIG_N) 
        {
          next_min_dis = min_dis;
        }
        min_dis = fabs(e(2)); 
        ret_id = it->first; 
        // display_plane(pw, "check: ");
        // ROS_ERROR("test_mul_plane_3d.cpp: angle right dis: %f min_dis = %f, ret_id = %d", fabs(e(2)), min_dis, ret_id);
      }
    }
    ++it; 
  }
  if(ret_id == large_bias_id )
  {
    if( min_dis > 4)
    {
      printf(KYEL "test_mul_plane_3d.cpp: large_bias_id, but too large min_dis: %f\n", min_dis);
      ret_id = -1;
    }else{
      printf(KGRN "test_mul_plane_3d.cpp: large_bias_id, successfully min_dis: %f ret_id = %d\n", min_dis, ret_id);
    }
  }
  else {
    if(min_dis <0 || min_dis > T_dis) ret_id = -1; 
    if(ret_id != -1 && next_min_dis != BIG_N && next_min_dis < 1.67*min_dis)
    {
      printf(KCYN "test_mul_plane_3d.cpp: ambiguious match min_dis %f, next_min_dis %f, discard !\n", min_dis, next_min_dis); 
      ret_id = -1;
    }
    

    if(ret_id == 5006 || ret_id == 5007)
    {
      printf(KGRN "test_mul_plane_3d.cpp: associated with %d wall line cx = %f , dis = %f \n", ret_id, cx, min_dis);
    }
  }

  return ret_id;
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

void display_plane(Plane3D* p, string prefix)
{
  Eigen::Vector4d v; 
  v = p->toVector(); 
  Eigen::Vector3d cv; cv(0)= v(0); cv(1) = v(1); cv(2) = v(2); // v<3,1>(0,0);
  cout<<prefix<<R2D(p->azimuth(cv))<<" "<<R2D(p->elevation(cv))<<" "<<p->distance()<<endl;
}


