#include "graph_wrapper.h"
#include "misc.h"         // eigenTransf2TF
#include <cmath>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include "NodeWrapper.h"
#include "plane_extract.h"
#include "paramSrvMi.h"
#include "pcl_ros/transforms.h"
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include "global_def.h"
#include <string>
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h"

CGraphWrapper::CGraphWrapper()
{
  // declare robot pos publisher 
  ros::NodeHandle n; 
  r_pos_pub_ = n.advertise<std_msgs::Float32MultiArray>("/robot_pos", 10);
  
  // call back of save g2o graph 
  // graph_save_sub_ = nh.subscribe("/save_graph", 1, CGraphWrapper::);  // save graph structure
  r_pos2D_sub_ = new message_filters::Subscriber<std_msgs::Float32MultiArray>(n, "/robot_pos_2d_back", 1); 
  r_pos2D_sub_->registerCallback(boost::bind(&CGraphWrapper::robotPos2DCallBack, this, _1));

  // parameters for plane vertexes 
  floor_node_id_ = 60000; // this sets the upper limitation of the number of nodes in a graph 
  offset_sensor_id_ = floor_node_id_ + 1; 
  
  // obstacles publisher 
  cmd_pub = n.advertise<std_msgs::Int32>("/voice_cmd", 1);
  ParamSrvMi* ps = ParamSrvMi::instanceMi();
  z_min_threshold_ = ps->get<float>("obstacle_min_z"); 
  y_len_threshold_ = ps->get<float>("obstacle_len_y"); 
  x_max_threshold_ = ps->get<float>("obstacle_max_x");
  g_euclidean_threshold_ = ps->get<float>("obstacle_density_threshold"); 
  g_number_pt_threshold_ = ps->get<int>("obstacle_number_pt_threshold");
}

void CGraphWrapper::saveG2OGraph(QString filename)
{ 
   string pos2D_fname(qPrintable(filename)); 
   pos2D_fname += string("_pos2D"); 
   ofstream pos2D_fout(pos2D_fname); 
   ROS_ERROR("graph_wrapper.cpp: in saveG2OGraph(), pos2D_fname = %s, pos2D_fout has %d members", pos2D_fname.c_str(), pos2D_pool_.size());

   if(pos2D_fout.is_open())
   {
     map<int, vector< float> >::iterator it = pos2D_pool_.begin();
     while(it!= pos2D_pool_.end())
     {
       vector<float>& pos2D = it->second; 
       pos2D_fout<<it->first<<" "<<pos2D[0]<<" "<<pos2D[1]<<" "<<pos2D[2]<<endl;
       ++it;
     }
     pos2D_fout.close();
   }
  
   // transform the position of the vertex into global coordinate reference 
   map<int, Node*>::iterator it = graph_.begin(); 
   while(it != graph_.end())
   {
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(it->second->vertex_id_));
      tf::Transform T_o2c = eigenTransf2TF(v->estimate());
      tf::Transform T_g2c = T_g2o_ * T_o2c; 
      v->setEstimate(tf2G2O(T_g2c));
      ++it; 
   }

   return GraphManager::saveG2OGraph(filename);
}

CGraphWrapper::~CGraphWrapper()
{
  r_pos_pub_.shutdown();
}

int CGraphWrapper::graph_size()
{
  return graph_.size();
}

bool CGraphWrapper::nodeComparisons(Node* newNode, 
                         QMatrix4x4& curr_motion_estimate,
                         bool& edge_to_keyframe)
{
  return GraphManager::nodeComparisons(newNode, curr_motion_estimate, edge_to_keyframe);
} 

void CGraphWrapper::robotPos2DCallBack(const std_msgs::Float32MultiArray::ConstPtr& r_pos2d_ptr)
{
  const float * pf = r_pos2d_ptr->data.data(); 
  int node_id = (int)(*pf); 
  vector<float> pose2d(3);
  pose2d[0] = *(pf+1);  pose2d[1] = *(pf+2); pose2d[2] = *(pf+3); 
  
  // from 2D image coordinate reference to global coordinate 
  float& gx = pose2d[0]; 
  float& gy = pose2d[1]; 
  float& gth = pose2d[2];
  if(mapping_rule_select_ == 4)
  {
    //    image reference           rule 4 reference 
    //    o -------x               x ------ o
    //      |                             |
    //      |                             |
    //      y                             y
    gx = -gx;   gy = gy;  
    // ROS_ERROR("graph_wrapper.cpp: before gth = %f ", R2D(gth)); 
    gth = (gth * -1) - M_PI;
    // ROS_ERROR("graph_wrapper.cpp: after gth = %f", R2D(gth));
    gth = normAngle(gth, -M_PI);
  }// TODO 
  else if(mapping_rule_select_ == 2)
  {
    //    image reference           rule 2 reference 
    //    o -------x                      | y
    //      |                             |
    //      |                             |------ x
    //      y                             o
    gx = -gx; gy = gy;  // TODO: make it clearly
    gth = gth * -1; 
    gth = normAngle(gth, -M_PI);
  }else {
  
  }
  pos2D_pool_[node_id] = pose2d; 
  // ROS_ERROR("graph_wrapper.cpp: receive pose_id: %d, at %f %f %f", node_id, pose2d[0], pose2d[1], pose2d[2]);
}

bool CGraphWrapper::addNode(Node* new_node)
{
  bool ret = GraphManager::addNode(new_node);
  if(ret)
  {
    bool b_floor_detected = false; 
    ParamSrvMi* ps = ParamSrvMi::instanceMi();
    if(graph_size() == 1) // it's the first node 
    {
      firstNode(new_node);  // we will add extra stuff for the first node
    }else
    {
      if(ps->get<bool>("b_add_floor_into_graph"))  
       b_floor_detected = addFloorPlane(new_node); 
    }
    // ROS_WARN("graph_wrapper.cpp: succeed to addNode, then publish it!"); 
    if(ps->get<bool>("send_2_vis"))
      publish2D(new_node);
    else
    {
      // ROS_ERROR("graph_wrapper.cpp: what? send_2_vis is false?");
    }
    if(ps->get<bool>("save_added_node"))
      ((CNodeWrapper*)new_node)->write(ps->get<std::string>("save_node_path"));
    // ROS_WARN("graph_wrapper.cpp: succeed to addNode, finish publish it!"); 
    
    if(ps->get<bool>("b_front_obstacle_detect"))
    {     

      // if(detectObstacleInFront(new_node, b_floor_detected))
      if(detectObstacleInFront(new_node))
      {
        static int obs_id = 0;
        ROS_ERROR("graph_wrapper.cpp: detect obstacle %d in the front", ++obs_id);
        // notice client-end obstacle appear in the front
        std_msgs::Int32Ptr pmsg(new std_msgs::Int32); 
        pmsg->data = 30;     // 30 for obstacles
        cmd_pub.publish(pmsg);
      }
    }
  }
  return ret; 
}

// detect obstacles in the front 
bool CGraphWrapper::detectObstacleInFront(Node* new_node, bool floor_detected)
{
  Eigen::Isometry3d g_trans; 
  tfScalar g_yaw;
  getGlobalPose(new_node, g_trans, g_yaw); 
  Eigen::Matrix4f g_t = g_trans.matrix().cast<float>(); 
  
  // transform into global coordinate reference 
  pointcloud_type::Ptr global_pc(new pointcloud_type); 
  pointcloud_type::Ptr pc(new pointcloud_type);
  pcl::transformPointCloud(*(new_node->pc_col), *global_pc, g_t); 
  
  float tx = g_t(0,3); 
  float ty = g_t(1,3); 
  g_yaw += M_PI/2.;
  float cy = cos(g_yaw); 
  float sy = sin(g_yaw); 
  
  // inverse([R, t]) = [Rt, -Rt*t]
  tx =   cy * g_t(0,3) + sy * g_t(1,3); 
  ty = - sy * g_t(0,3) + cy * g_t(1,3); 

  // subtract camera's location (x, y, 0) and floor's z 
  // and filter out points not in the bounding box x ~ [0.4, 2], y ~ [-0.2, 0.2] z ~ [0.2, 2]
   // do not do this, just transform the coordinate reference into global, from camera to global 
    //            z                            z   x
    //           /                             |  /
    //          /                              | /
    //         /----- x                 y ---- |/
    //         |                             Global
    //         |                                     
    //         | y
    //       Camera 
  float mx, my; 
  for(int i=0; i<global_pc->points.size(); i++)
  {
    point_type& pt = global_pc->points[i]; 
    mx = pt.x; // - tx;  
    my = pt.y; // - ty; // pt.z -= floor_level_; 
    pt.x =   cy * mx + sy * my; 
    pt.y = - sy * mx + cy * my; 
    pt.x -= tx; 
    pt.y -= ty; 

    if(pt.x > 0.3 && pt.x < x_max_threshold_ && pt.y > -y_len_threshold_ && pt.y < y_len_threshold_ )
    {
      if(!floor_detected) // no floor
      {
        if(pt.z > z_min_threshold_ && pt.z < 0)
          pc->points.push_back(pt);  // this point in the bounding box 
      }else // there is floor 
      {
         // check the projection of this point on the floor 
         // 1. extract floor plane 
        static CPlaneExtract<point_type> floor_extracter; 
        Eigen::Vector4f coeffs; 
        floor_extracter.extract(new_node->pc_col, coeffs);
        
        // 2 check compute the distance from a point to this plane 
        Eigen::Vector4d pv; 
        pv << floor_extracter.nx_ , floor_extracter.ny_, floor_extracter.nz_, floor_extracter.d_; 
        point_type& pb = new_node->pc_col->points[i];
        double dis = pv(0) * pb.x + pv(1) * pb.y + pv(2) * pb.z + pv(3); 
        if( fabs(dis) > 0.15) 
            pc->points.push_back(pt);
      }
    }
  }
 
  // remove floor points 

/*
  if(pc->points.size() > 500)
  {
    {
      {
        pc->height = 1; 
        pc->width = pc->points.size(); 
        stringstream ss;
        ss<<"./pcds/b_"<<new_node->id_<<".pcd";
        pcl::io::savePCDFile(ss.str().c_str(), *pc); 
      }
      {
        stringstream ss;
        ss<<"./pcds/b_"<<new_node->id_<<"g.pcd"; 
        pcl::io::savePCDFile(ss.str().c_str(), *global_pc); 
      }
      ROS_ERROR("graph_wrapper.cpp: save %d bounding pcds",new_node->id_);
    }
  }
*/
  int pt_threshold = g_number_pt_threshold_;  
  float g_euclidean_threshold = g_euclidean_threshold_;  
 
  if(pc->points.size() < pt_threshold) // points in the bounding box is less 
  {
    // ROS_INFO("graph_wrapper.cpp: after bounding box filtering only %d pts left", pc->points.size());
    return false; 
  }

  // distance filter, get the largest cluster 
  typename pcl::search::KdTree<point_type>::Ptr tree(new pcl::search::KdTree<point_type>); 
  vector<pcl::PointIndices> cluster_indices;
    
  pcl::EuclideanClusterExtraction<point_type> ec; 
  ec.setClusterTolerance(g_euclidean_threshold);
  ec.setMinClusterSize(400);
  // ec.setMaxClusterSize(g_max_cluster_num);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pc);
  ec.extract(cluster_indices);

  int num_pts = 0;
  if(cluster_indices.size() > 0)
  { 
    // int num_pts = 0; 
    for(int j = 0; j<cluster_indices.size(); j++)
    {
      // num_pts += cluster_indices[j].indices.size(); 
      num_pts = cluster_indices[j].indices.size(); 
      if(num_pts > pt_threshold)
      {
        /*
        {
          {
            pc->height = 1; 
            pc->width = pc->points.size(); 
            stringstream ss;
            ss<<"./pcds/pc_"<<new_node->id_<<".pcd";
            pcl::io::savePCDFile(ss.str().c_str(), *pc); 
          }
          {
            stringstream ss;
            ss<<"./pcds/pc_"<<new_node->id_<<"g.pcd"; 
            pcl::io::savePCDFile(ss.str().c_str(), *global_pc); 
          }
          // ROS_ERROR("graph_wrapper.cpp: save obstacle pcds");
        }*/
        if(floor_detected)
        {
          // ROS_ERROR("graph_wrapper.cpp: floor %d is detected!", new_node->id_);
        }
        // ROS_ERROR("graph_wrapper.cpp: succeed! before euclidean filtering %d after %d pts left", pc->points.size(), num_pts);
        
        return true; 
      }
    }
  }
  ROS_ERROR("graph_wrapper.cpp: before euclidean filtering %d after only %d pts left", pc->points.size(), num_pts);
  return false;
}

bool CGraphWrapper::isFloor(void* p)
{
  g2o::Plane3D* pf = (g2o::Plane3D*)(p);
  Eigen::Vector3d floor_normal(0, 0, 1); // so dot product is the n.dot(floor) = nz
  double COS15 = cos(15.*M_PI/180.); 
  double COS30 = cos(30.*M_PI/180.);
  Eigen::Vector4d v; 
  v = pf->toVector(); 
  // if(fabs(v(2)) > COS15)
  if(fabs(v(2)) > COS30)
    return true;
  return false;
}

bool CGraphWrapper::getGlobalPose(Node* new_node, Eigen::Isometry3d& trans, tfScalar& yaw)
{
  // Eigen matrix rotation 
  CNodeWrapper* pn = dynamic_cast<CNodeWrapper*>(new_node);
  g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(new_node->vertex_id_));
  if(v == NULL)
  {
    ROS_ERROR("graph_wrapper.cpp: node id %d not exist", new_node->vertex_id_);
    return false;
  }
  tf::Transform T_o2c = eigenTransf2TF(v->estimate());
  tf::Transform T_g2c = T_g2o_ * T_o2c; 
  tfScalar pitch, roll; 
  T_g2c.getBasis().getEulerYPR(yaw, pitch, roll); 

  Eigen::Matrix4f T_g2c_eigen;      
  pcl_ros::transformAsMatrix(T_g2c, T_g2c_eigen);
  Eigen::Matrix4d T_g2c_eigen_d = T_g2c_eigen.cast<double>();
  // Eigen::Isometry3d T_g2c_isometry; 
  // T_g2c_isometry.matrix() = T_g2c_eigen_d.block<4,4>(0,0); 
  trans.matrix() = T_g2c_eigen_d.block<4,4>(0,0);
  return true;
}

bool CGraphWrapper::getGlobalPose(Node* new_node, Eigen::Isometry3d& trans)
{
  tfScalar yaw; 
  return getGlobalPose(new_node, trans, yaw);
}

bool CGraphWrapper::addFloorPlane(Node* new_node)
{
  // 1. extract floor plane 
  static CPlaneExtract<point_type> floor_extracter; 
  Eigen::Vector4f coeffs; 
  if(!floor_extracter.extract(new_node->pc_col, coeffs))
  {
    return false; 
  }
  // 1.2 check whether this is a floor plane 
  Eigen::Vector4d pv; 
  pv << floor_extracter.nx_ , floor_extracter.ny_, floor_extracter.nz_, floor_extracter.d_; 
  g2o::Plane3D pf3d; pf3d.fromVector(pv);

  // Eigen matrix rotation 
  // CNodeWrapper* pn = dynamic_cast<CNodeWrapper*>(new_node);
  // g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(new_node->vertex_id_));
  // tf::Transform T_o2c = eigenTransf2TF(v->estimate());
  // tf::Transform T_g2c = T_g2o_ * T_o2c; 
  // Eigen::Matrix4f T_g2c_eigen;      
  // pcl_ros::transformAsMatrix(T_g2c, T_g2c_eigen);
  // Eigen::Matrix4d T_g2c_eigen_d = T_g2c_eigen.cast<double>();
  Eigen::Isometry3d T_g2c_isometry; 
  getGlobalPose(new_node, T_g2c_isometry);
  // T_g2c_isometry.matrix() = T_g2c_eigen_d.block<4,4>(0,0); 

  g2o::Plane3D pf3d_g = T_g2c_isometry * pf3d ;
  if(!isFloor((void*)&pf3d_g))
  {
    return false; 
  }

  // 2. if success, add an edge between this node and the floor vertex 
  g2o::EdgeSE3PlaneSensorCalib * e = new g2o::EdgeSE3PlaneSensorCalib; 
  e->vertices()[0] = optimizer_->vertices()[new_node->vertex_id_]; 
  e->vertices()[1] = optimizer_->vertices()[floor_node_id_];
  e->vertices()[2] = optimizer_->vertices()[offset_sensor_id_]; 
  // e->setMeasurement(*(it->second)); 
  e->setMeasurement(pf3d);
  Eigen::Matrix3d m = Eigen::Matrix3d::Zero(); 
  m(0,0) = 20;   // aminth no contribution
  m(1,1) = 100;   // elevation 
  m(2,2) = 1000;  // distance 
  e->setInformation(m); 
  optimizer_->addEdge(e);
  
  // cam_cam_edges_ involves the nodes and the edges that are updated during graph optimization
  // add this will generate error, because cam_cam_edges_ is a set of EdgeSE3, not EdgeSE3PlaneSensorCalib
  // cam_cam_edges_.insert(e); 

  // ROS_ERROR("graph_wrapper.cpp: add plane edge at node %d", new_node->vertex_id_);
  return true;
}

void CGraphWrapper::publishG2O()
{
  // weather to send point data 
  ParamSrvMi* ps = ParamSrvMi::instanceMi();
  string g2o_file = ps->get<string>("g2o_file") ; 
  if(optimizer_->load(g2o_file.c_str()))
  {
    ROS_INFO("graph_wrapper.cpp: succeed to load g2ofile: %s", g2o_file.c_str()) ;    
  }else
  {
    ROS_ERROR("graph_wrapper.cpp: failed to load g2ofile: %s", g2o_file.c_str()) ;
    return ;
  }
  
  std_msgs::Float32MultiArray pos_msg; 
  pos_msg.data.resize(4);     // x, y, theta, node_id there is coordinate translation  
  ros::Rate loop_rate(10);  // publish speed 

  // publish each node 
  int seq_id = 0; 
  for(g2o::HyperGraph::VertexIDMap::iterator it = optimizer_->vertices().begin(); it!= optimizer_->vertices().end(); ++it)
  {
    g2o::VertexSE3* vin = dynamic_cast<g2o::VertexSE3*>(it->second); 
    if(vin == NULL)
      continue;
    if(vin->id() != seq_id) // sequencial 
    {
      continue; 
    }
    
    ROS_INFO("graph_wrapper.cpp: pub vertex id: %d" , seq_id++); 
    tf::Transform T_g2c = eigenTransf2TF(vin->estimate()); 

    int mapping_rule_select = ps->get<int>("mapping_rule");
    mapping_rule_select_ = mapping_rule_select; 
      
    float x1, y1, z1; 
    float x2, y2, z2; 
    tf::Vector3 t = T_g2c.getOrigin(); 
    x1 = t.getX(); y1 = t.getY(); z1 = t.getZ(); 
 
    tf::Quaternion q = T_g2c.getRotation();    
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // float yaw = asin(2*(q.w()*q.y() - q.x()*q.z())); 
    float yaw = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1-2*(SQ(q.y()) + SQ(q.z())));
    if(mapping_rule_select ==1 )
    {
      //     O ----- y
      //      |
      //      |
      //      x
      // x2 = x1;
      // z2 = z1*cp - y1*sp; 
      // z2 = sqrt(z1*z1 + y1*y1);

      // x2 = y1 ;
      // y2 = x1*cp - z1*sp; 

      x2 = y1; 
      y2 = x1; 
    }else if(mapping_rule_select == 2)
    {
      //        y
      //        |
      //        |
      //        |
      //       O ----- x 
      // x2 = z1*cp - y1*sp; 
      // z2 = -x1;

      // x2 = x1*cp - z1*sp; 
      // y2 = -y1;
      
      x2 = x1;
      y2 = -y1;
    }else if(mapping_rule_select == 3)
    {
      //        x
      //        |
      //        |
      //        |  
      // y ----- O
      //
      // x2 = -x1; 
      // z2 = -(z1*cp - y1*sp);
      
      // x2 = -y1; 
      // y2 = -(x1*cp - z1*sp);

      x2 = -y1; 
      y2 = -x1;
    }else if(mapping_rule_select == 4)
    {
      // x ----- O     
      //        |
      //        |
      //        | 
      //        y
      x2 = -x1; 
      y2 = y1;
    }
    /* change initial heading  */
    // pos_msg.data[0] = z2; //t.getZ(); 
    // pos_msg.data[1] = -x2; //-t.getX();
    pos_msg.data[0] = x2; 
    // pos_msg.data[1] = z2;  
    pos_msg.data[1] = y2;
    // pos_msg.data[2] = yaw
    // July 16, 2015, this is a bug, retrieve the reference transformation 
    pos_msg.data[2] = -1*(yaw+M_PI);  

    // ROS_ERROR("GraphWrapper, succeed to add node, broadcast result!"); 
    r_pos_pub_.publish(pos_msg);   
    ROS_INFO("graph_wrapper.cpp: publish 2d pos: %f %f %f ", x2, y2, pos_msg.data[2]);
    ros::spinOnce(); 
    loop_rate.sleep(); 
  }
  return ;
}

void CGraphWrapper::publish2D(Node* new_node)
{
    static const int nn = 4;  // x,z,theta,node_id
    std_msgs::Float32MultiArray pos_msg; 
    pos_msg.data.resize(4);     // x, y, theta, node_id there is coordinate translation  

    CNodeWrapper* pn = dynamic_cast<CNodeWrapper*>(new_node);

    // retrieve the position of current node 
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(new_node->vertex_id_));
    tf::Transform tf_trans = eigenTransf2TF(v->estimate());
 
    static tf::Transform T_o2c;                  // original to current camera 
    static tf::Transform T_g2c;                  // global to current camera T_g2c = T_g2b * T_b2o * T_o2c; 

    T_o2c = tf_trans;                            // camera position in the original reference

    static float floor_level = floor_level_;
    static float floor_range = 0.1; // points above floor +floor_range be considered as floor 

    // weather to send point data 
    ParamSrvMi* ps = ParamSrvMi::instanceMi();

    // from global to camera 
    // T_g2c = T_g2b * T_b2o * T_o2c; 
    T_g2c = T_g2o_ * T_o2c; 

    // mapping robot reference to 2D map reference z->x, x->-y, pitch-> theta
    // 2D pixel coordinate reference 
    //   
    //  O ----- x
    //    |
    //    |
    //    y
    tf::Vector3 t = T_g2c.getOrigin(); 
 
    // the rule map from 3D to 2D 
    int mapping_rule_select = ps->get<int>("mapping_rule");
    mapping_rule_select_ = mapping_rule_select; 
      
    float x1, y1, z1; 
    float x2, y2, z2; 
    x1 = t.getX(); y1 = t.getY(); z1 = t.getZ(); 
    
    // quaternion q w,x,y,z -> q0, q1, q2, q3
    // tf::Quaternion q = tf_trans.getRotation();    
    tf::Quaternion q = T_g2c.getRotation();    
    
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // float yaw = asin(2*(q.w()*q.y() - q.x()*q.z())); 
    float yaw = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1-2*(SQ(q.y()) + SQ(q.z())));
    // cerr<<"graph_wrapper.cpp: compute yaw: "<<R2D(yaw)<<endl;

    // 2D map reference 
    //
    //    O ------- x2
    //      |
    //      |
    //      |
    //      y2 
    //

    if(mapping_rule_select ==1 )
    {
      //     O ----- y
      //      |
      //      |
      //      x
      // x2 = x1;
      // z2 = z1*cp - y1*sp; 
      // z2 = sqrt(z1*z1 + y1*y1);

      // x2 = y1 ;
      // y2 = x1*cp - z1*sp; 

      x2 = y1; 
      y2 = x1; 
    }else if(mapping_rule_select == 2)
    {
      //        y
      //        |
      //        |
      //        |
      //       O ----- x 
      // x2 = z1*cp - y1*sp; 
      // z2 = -x1;

      // x2 = x1*cp - z1*sp; 
      // y2 = -y1;
      
      x2 = x1;
      y2 = -y1;
      // ROS_ERROR("graph_wrapper.cpp: in mapping_rule_ 2");
    }else if(mapping_rule_select == 3)
    {
      //        x
      //        |
      //        |
      //        |  
      // y ----- O
      //
      // x2 = -x1; 
      // z2 = -(z1*cp - y1*sp);
      
      // x2 = -y1; 
      // y2 = -(x1*cp - z1*sp);

      x2 = -y1; 
      y2 = -x1;
    }else if(mapping_rule_select == 4)
    {
      // x ----- O     
      //        |
      //        |
      //        | 
      //        y
      x2 = -x1; 
      y2 = y1;
    }
    /* change initial heading  */
    // pos_msg.data[0] = z2; //t.getZ(); 
    // pos_msg.data[1] = -x2; //-t.getX();
    pos_msg.data[0] = x2; 
    // pos_msg.data[1] = z2;  
    pos_msg.data[1] = y2;
    // pos_msg.data[2] = yaw
    // July 16, 2015, this is a bug, retrieve the reference transformation 
    pos_msg.data[2] = -1*(yaw+M_PI);  

    // cerr<<"graph_wrapper.cpp: yaw: "<<R2D(yaw)<<" pos_msg.yaw: "<<R2D(pos_msg.data[2])<<endl;

    // add vertex id in msg, send it to the 2D localization program 
    pos_msg.data[3] = new_node->vertex_id_;

    if(ps->get<bool>("send_pc2_vis") && pn->pc_col->points.size() > 0)
    {
      // 1, transform point cloud to global reference 
      pointcloud_type::Ptr global_pc(new pointcloud_type); 
      pointcloud_type::Ptr distant_pc(new pointcloud_type); 

      // distance threshold 
      float min_distance_threshold = ps->get<float>("min_distance_thre"); 

      // delete points too close
      for(int i=0; i<pn->pc_col->points.size(); i++)
      {
        point_type& pt = pn->pc_col->points[i]; 
        if(SQ(pt.x) + SQ(pt.y) + SQ(pt.z) > min_distance_threshold)  // TODO: this distance threshold should be a parameter 
          distant_pc->points.push_back(pt);
      }
    
      // first transform to the first coordinate reference 
      // tf::Transform pose = eigenTransf2TF(v->estimate());
      Eigen::Matrix4f eigen_transform ;      
      // pcl_ros::transformAsMatrix(pose, eigen_transform);
      pcl_ros::transformAsMatrix(T_g2c, eigen_transform);

      // then, transform to parallel with floor
      // eigen_transform = R_original*eigen_transform;

      // Eigen::Matrix4f tmp1 = R_original*eigen_transform;
      // pcl::transformPointCloud(*(pn->pc_col), *global_pc, eigen_transform);
      pcl::transformPointCloud(*distant_pc, *global_pc, eigen_transform);

      // 2, add the points (x, z) to the msg patch, y is orthogonal to the ground 
      int N = global_pc->points.size(); // pn->pc_col->points.size();
      int CNT = 0; 
      pos_msg.data.resize(nn + N*2); 
      vector<bool> flag(N, false);

      for(int i=0; i<N; i++)
      {
        point_type& pt = global_pc->points[i]; //pn->pc_col->points[i]; 
        // if(pt.y < floor_level) floor_level = pt.y;
        if(pt.z < floor_level) floor_level = pt.z;
        // if(pt.z - z2 < 0.6) flag[i] = true; // all points within 0.6 will be deleted
      }
      
      // ROS_ERROR("graph_wrapper.cpp: floor_level = %f", floor_level);
        
      for(int i=0; i<N; i++)
      {
        if(flag[i]) continue;
        point_type& pt = global_pc->points[i]; //pn->pc_col->points[i]; 
        // if(floor_level != -10 && pt.y < floor_level + floor_range) 
        if(floor_level != -10 && pt.z < floor_level + floor_range) 
          continue;
        
        if(mapping_rule_select == 1)
        {
          //     O ----- x
          //      |
          //      |
          //      z
          // pos_msg.data[nn + 2*CNT] = pt.x; 
          // pos_msg.data[nn + 2*CNT+1] = pt.z;
          //     O ----- y
          //      |
          //      |
          //      x
          pos_msg.data[nn + 2*CNT] = pt.y; 
          pos_msg.data[nn + 2*CNT+1] = pt.x;

        }else if(mapping_rule_select == 2)
        {
          //        x
          //        |
          //        |
          //        |
          //       O ----- z
          // pos_msg.data[nn + 2*CNT] = pt.z; 
          // pos_msg.data[nn + 2*CNT+1] = -pt.x;

          //        y
          //        |
          //        |
          //        |
          //       O ----- x
          pos_msg.data[nn + 2*CNT] = pt.x; 
          pos_msg.data[nn + 2*CNT+1] = -pt.y;

        }else if(mapping_rule_select == 3)
        {
          //        z
          //        |
          //        |
          //        |  
          // x ----- O
          //
          // pos_msg.data[nn + 2*CNT] = -pt.x; 
          // pos_msg.data[nn + 2*CNT+1] = -pt.z;

          //        x
          //        |
          //        |
          //        |  
          // y ----- O
          //
          pos_msg.data[nn + 2*CNT] = -pt.y; 
          pos_msg.data[nn + 2*CNT+1] = -pt.x;

        }else if(mapping_rule_select == 4)
        {
          // x ----- O     
          //        |
          //        |
          //        | 
          //        y
          pos_msg.data[nn + 2*CNT] = -pt.x;  
          pos_msg.data[nn + 2*CNT+1] = pt.y;
        }
        else
        {
          ROS_ERROR("graph_wrapper.cpp: what? there is no such rule right now!");
        }
        ++CNT;
        // pos_msg.data[3 + 2*i+1] = sqrt(SQ(pt.y) + SQ(pt.z));
        // pos_msg.data[3 + 2*i+1] = pt.z*cp - pt.y*sp;
      }
      // cerr<<"graph_wrapper.cpp: send camera data size: "<<CNT<<endl;
      pos_msg.data.resize(nn + CNT*2);
      // ROS_ERROR("graph_wrapper.cpp: publish 2D info CNT = %d", CNT);
    }

    // ROS_ERROR("GraphWrapper, succeed to add node, broadcast result!"); 
    r_pos_pub_.publish(pos_msg);   
}

double CGraphWrapper::normAngle(double angle, double base) {
    double pi2 = 2*M_PI;
    double min2pi = base + pi2;
    while(angle>=min2pi) angle -= pi2;
    while(angle<base) angle += pi2;
    return angle;
}


bool CGraphWrapper::firstNode(Node* new_node)
{
    // with IMU 
    // tf::StampedTransform tf_trans = computeFixedToBaseTransform(new_node, false);
    
    // do not do this, just transform the coordinate reference into global, from camera to global 
    //            z                            z   x
    //           /                             |  /
    //          /                              | /
    //         /----- x                 y ---- |/
    //         |                             Global
    //         |                                     
    //         | y
    //       Camera 

    static tf::Transform T_g2b;                  // global to base 
    static tf::Transform T_b2o;                  // base to original 
    // static tf::Transform T_o2c;                  // original to current camera 
    // static tf::Transform T_g2c;                  // global to current camera T_g2c = T_g2b * T_b2o * T_o2c; 

    // T_o2c = tf_trans;                            // camera position in the original reference

    // Eigen matrix rotation 
    CNodeWrapper* pn = dynamic_cast<CNodeWrapper*>(new_node);
    static float pitch = 0; // pn->getPitch();
    // static Eigen::Matrix4f R_original = Eigen::Matrix4f::Identity();
    static float cp = cos(pitch); 
    static float sp = sin(pitch);

    // the coordinate of the floor, to delete points that belong to the floor
    static float floor_y = 0; 
    static float floor_z = 0;
    static float floor_x = 0;
    static tf::Vector3 floor_center; 
    static float floor_level = -10;
    static float floor_range = 0.1; // points above floor +floor_range be considered as floor 

    // if(graph_.size() == 1) // the first node, figure out the ground and the pitch 
    {
      // set T_g2b, from global to base 
      tf::Matrix3x3 R_y;    // first, rotate along y 90'
      R_y.setValue( 0, 0, 1, 
                    0, 1, 0, 
                    -1, 0, 0); 
      tf::Matrix3x3 R_z; // second, rotate along z -90' 
      R_z.setValue( 0, 1, 0, 
                    -1, 0,  0,
                    0, 0, 1);
      T_g2b = tf::Transform(R_y*R_z);
      /*
      {
      // for debug

      tf::Vector3 t1 = T_g2b.getBasis()[0]; 
      cerr<<"graph_wrapper.cpp row 1 : "<<t1.x()<<" "<<t1.y()<<" "<<t1.z()<<endl;
      t1 = T_g2b.getBasis()[1];
      cerr<<"graph_wrapper.cpp roa 2 : "<<t1.x()<<" "<<t1.y()<<" "<<t1.z()<<endl;
      t1 = T_g2b.getBasis()[2];
      cerr<<"graph_wrapper.cpp row 3 : "<<t1.x()<<" "<<t1.y()<<" "<<t1.z()<<endl;

      t1 = tf::Vector3(1,2,3); 
      t1 = T_g2b*t1; 
      cerr<<"graph_wrapper.cpp t1 : "<<t1.x()<<" "<<t1.y()<<" "<<t1.z()<<endl;
      }
      */

      // ROS_ERROR("graph_wrapper.cpp: start to extract floor!");
      if(pn->pc_col->points.size() > 0) // store points 
      {
         CPlaneExtract<point_type> floor_extracter;
         float tmp = floor_extracter.getPitchByGround(pn->pc_col);
         if(tmp != 0) // succeed to set the pitch angle 
         {
            pitch = tmp; 
            ROS_ERROR("graph_wrapper.cpp: compute the initial pitch angle: %f", R2D(tmp));
            cp = cos(pitch); 
            sp = sin(pitch); 
            // cerr<<"cp: "<<cp<<" sp: "<<sp<<endl;
            // cerr<<"R_original: "<<R_original<<endl;
            
            //   Swiss Ranger coordinate reference 
            //          y   x
            //          |  /
            //          | /
            //          |/____ z  

            // this rotate around x-axis 
            // R_original(1,1) = cp;   R_original(1,2) = -sp;
            // R_original(2,1) = sp;   R_original(2,2) = cp;
            // floor_y = floor_extracter.floor_y_; 
            // floor_z = floor_extracter.floor_z_;
            // floor_level = floor_z*sp + floor_y*cp; 
            // ROS_ERROR("graph_wrapper.cpp: floor_y: %f floor_z: %f floor level: %f", floor_y, floor_z, floor_level);

            //   Smart cane coordinate reference 
            //          z   y
            //          |  /
            //          | /
            //          |/____ x  

            // this rotate around y-axis 
            // R_original(2,2) = cp;   R_original(0,2) = -sp;
            // R_original(2,0) = sp;   R_original(0,0) = cp;
            // R_original = R_original.inverse().eval();
            // cerr<<"R_inverse: "<<R_original<<endl;
            // floor_z = floor_extracter.floor_z_; 
            // floor_x = floor_extracter.floor_x_; 
            // floor_level = floor_x*sp + floor_z*cp; 
            // ROS_ERROR("graph_wrapper.cpp: floor_y: %f floor_z: %f floor level: %f", floor_y, floor_z, floor_level);

            //   camera coordinate reference 
            //             x
            //            /
            //           /
            //          /____ z  
            //          |
            //          |
            //          | y
            // this rotate around x-axis 
            tf::Matrix3x3 R_b2o;
            // R_original(1,1) = cp;   R_original(1,2) = -sp;
            // R_original(2,1) = sp;   R_original(2,2) = cp;
            R_b2o.setValue(1, 0, 0, 
                           0, cp, -sp, 
                           0, sp, cp);

            // T_b2o = eigenTransf2TF(R_original); // get transformation from base to origin 
            T_b2o = tf::Transform(R_b2o);

            // set floor center 
            floor_z = floor_extracter.floor_z_; 
            floor_y = floor_extracter.floor_y_;
            floor_x = floor_extracter.floor_x_; 
            floor_center.setValue(floor_x, floor_y, floor_z);
            
            floor_center = T_g2b*T_b2o*floor_center;
            floor_level = floor_center.z();
            ROS_ERROR("graph_wrapper.cpp: floor_z: %f floor level: %f", floor_center.z(), floor_level);

            // reset min_z for obstacle detection,  
            if(floor_level > -1.2 && floor_level < -0.6)
            {
              z_min_threshold_ = floor_level + 0.15; 
            }

            // R_original(1,1) = cp;   R_original(1,2) = sp;
            // R_original(1,2) = -sp;   R_original(2,2) = cp;
         }else
         {
            ROS_ERROR("graph_wrapper.cpp: failed to detect floor, set T_b2o = Identity!");
            T_b2o = tf::Transform();
         }
        
         T_g2o_ = T_g2b * T_b2o; 

         // add plane vertex, first offset, and then floor node 
         Eigen::Isometry3d offset3d = Eigen::Isometry3d::Identity();
         g2o::VertexSE3 * offsetVertex = new g2o::VertexSE3; 
         offsetVertex->setId(offset_sensor_id_); 
         offsetVertex->setEstimate(offset3d); 
         offsetVertex->setFixed(true);
         optimizer_->addVertex(offsetVertex); 
         
         // construct Plane3D 
         g2o::Plane3D pFloor;
         Eigen::Vector4d v; 
         v << floor_extracter.nx_ , floor_extracter.ny_, floor_extracter.nz_, floor_extracter.d_; 
         pFloor.fromVector(v); 
         
         // something is wrong here 
         // cerr<<"graph_wrapper.cpp: initial floor parameter: "<<v<<endl;
         // transform into global coordinate 
         // Eigen::Matrix4f T_g2c_eigen;      
         // pcl_ros::transformAsMatrix(T_g2o_, T_g2c_eigen);
         // Eigen::Matrix4d T_g2c_eigen_d = T_g2c_eigen.cast<double>();
         // Eigen::Isometry3d T_g2c_isometry; 
         // T_g2c_isometry.matrix() = T_g2c_eigen_d.block<4,4>(0,0); 
         // g2o::Plane3D pf3d_g = T_g2c_isometry * pFloor ;
         // v = pf3d_g.toVector(); 
         // cerr<<"graph_wrapper.cpp: final floor parameter: "<<v<<endl;

         // because it is the first node, so it stands for the base coordinate reference 
         g2o::VertexPlane * p = new g2o::VertexPlane; 
         p->setEstimate(pFloor);
         // p->setEstimate(pf3d_g); 
         p->setId(floor_node_id_); 
         p->setFixed(true); 
         optimizer_->addVertex(p); 
      }else{
        ROS_ERROR("graph_wrapper.cpp: no point cloud avaliable, set T_b2o = Identity!");
        T_b2o = tf::Transform();  
      }
      // ROS_ERROR("graph_wrapper.cpp: finish floor extraction!");
      // now T_g2o_ is able to be determined 
      T_g2o_ = T_g2b * T_b2o; 
    }
/*
    int reproject_step = 150;
    if(graph_.size() % reproject_step == 0)
    {
      ROS_ERROR("graph_wrapper.cpp: reproject step, recompute the Tb2o");
      if(pn->pc_col->points.size() > 0) // store points 
      {
         CPlaneExtract<point_type> floor_extracter;
         float tmp = floor_extracter.getPitchByGround(pn->pc_col);
         float rad30 = D2R(30);
         if(tmp != 0 && fabs(tmp) < rad30)  // succeed to set the pitch angle 
         {
            float pitch = tmp; 
            ROS_ERROR("graph_wrapper.cpp: compute the initial pitch angle: %f", R2D(tmp));
            float cp = cos(pitch); 
            float sp = sin(pitch); 
                 //   camera coordinate reference 
            //             x
            //            /
            //           /
            //          /____ z  
            //          |
            //          |
            //          | y
            // this rotate around x-axis 
            tf::Matrix3x3 R_b2o;
            // R_original(1,1) = cp;   R_original(1,2) = -sp;
            // R_original(2,1) = sp;   R_original(2,2) = cp;
            R_b2o.setValue(1, 0, 0, 
                           0, cp, -sp, 
                           0, sp, cp);

            // T_b2o = eigenTransf2TF(R_original); // get transformation from base to origin 
            T_b2o = tf::Transform(R_b2o);

            // set floor center 
            floor_z = floor_extracter.floor_z_; 
            floor_y = floor_extracter.floor_y_;
            floor_x = floor_extracter.floor_x_; 
            floor_center.setValue(floor_x, floor_y, floor_z);

            floor_center = T_g2b*T_b2o*floor_center;
            floor_level = floor_center.z();
            ROS_ERROR("graph_wrapper.cpp: floor_z: %f floor level: %f", floor_center.z(), floor_level);

            // R_original(1,1) = cp;   R_original(1,2) = sp;
            // R_original(1,2) = -sp;   R_original(2,2) = cp;
         } 
      }
    }
    */
  floor_level_ = floor_level; 
  return true;
}


tf::Transform CGraphWrapper::getLastNodePose()
{
    // if(!m_bHasOptimized)
    {
    //    optimizeGraph();
    //    m_bHasOptimized = true;
    }
    optimizer_mutex_.lock();
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[graph_.size()-1]->vertex_id_));
    tf::Transform previous = eigenTransf2TF(v->estimate());
    optimizer_mutex_.unlock();
    return previous;
}


