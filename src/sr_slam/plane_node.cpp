#include "plane_node.h"
#include "pcl_ros/transforms.h"
#include <pcl/filters/voxel_grid.h>
#include "../sr_plane/plane.h"
#include "../sr_plane/plane_set.h"
#include "../sr_plane/cam_observation.h"
#include <pcl/common/transformation_from_correspondences.h>
#include "misc2.h" // weather this will cause conflict
#include "opencv2/highgui/highgui.hpp"
#include "../sr_plane/glob_def.h"
#include "paramSrvMi.h"
#include <limits>
#include "icp.h"
#include "cam_model.h"

#define COST(number, mean_error) ((mean_error)/(double)(number))

extern std::vector<cv::DMatch> sample_matches_prefer_by_distance (unsigned int sample_size, std::vector<cv::DMatch>& matches_with_depth);

extern Eigen::Matrix4f getTransformFromMatches(const Node* newer_node,
    const Node* earlier_node,
    const std::vector<cv::DMatch>& matches,
    bool& valid, 
    const float max_dist_m);

double errorFunction2(const Eigen::Vector4f& x1,
                      const Eigen::Vector4f& x2,
                      const Eigen::Matrix4d& transformation);

extern tf::Transform getTranRPYt(double r, double p, double y, double tx, double ty, double tz);

// extern tf::Transform eigenTransf2TF(const Eigen::Matrix4f& tf);
namespace {
void display_eigen(Eigen::Matrix4f& tmp_trans)
{
  tf::Transform tfT = eigenTransf2TF(tmp_trans);  
  tfScalar r, p, y, tx, ty, tz;
  tfT.getBasis().getEulerYPR(y, p, r); 
  tf::Vector3 t = tfT.getOrigin(); 
  tx = t.getX(); ty = t.getY(); tz = t.getZ();
  ROS_WARN("plane_node.cpp: sample transform rpy xyz : %f %f %f %f %f %f", r, p, y, tx, ty, tz);
}

inline int uv2index(int u, int v)
{
  return (u + v*176);
}

}
CPlaneNode::CPlaneNode(const cv::Mat& visual, 
          const cv::Mat& depth, 
          const cv::Mat& detection_mask, 
          std_msgs::Header depth_header, 
          cv::Ptr<cv::FeatureDetector> detector, 
          cv::Ptr<cv::DescriptorExtractor> extractor)
  : CNodeWrapper(visual, depth, detection_mask, depth_header, detector, extractor),
  b_has_been_projected_(false)
{
  intensity_img_ = visual.clone();
  init_node();
}

CPlaneNode::CPlaneNode(const cv::Mat& visual, 
          const cv::Mat& depth, 
          const cv::Mat& detection_mask, 
          const CamModel& cam_info,
          std_msgs::Header depth_header, 
          cv::Ptr<cv::FeatureDetector> detector, 
          cv::Ptr<cv::DescriptorExtractor> extractor)
  : CNodeWrapper(visual, depth, detection_mask, cam_info, depth_header, detector, extractor),
  b_has_been_projected_(false)
{
  intensity_img_ = visual.clone();
  init_node();
}



CPlaneNode::CPlaneNode(const cv::Mat& visual, 
          const cv::Mat& depth, 
          const cv::Mat& detection_mask, 
          std_msgs::Header depth_header, 
          vector<float>& px, vector<float>& py, vector<float>& pz,
          cv::Ptr<cv::FeatureDetector> detector, 
          cv::Ptr<cv::DescriptorExtractor> extractor)
  : CNodeWrapper(visual, depth, detection_mask, depth_header, px, py, pz, detector, extractor),
  b_has_been_projected_(false)
{
  intensity_img_ = visual.clone();
  init_node();
}

CPlaneNode::CPlaneNode(string path, int id): 
  CNodeWrapper(path, id), 
  b_has_been_projected_(false)
{
  init_node();
}

CPlaneNode::~CPlaneNode()
{
  // ROS_WARN("plane_node.cpp: in ~CPlaneNode()");
}

void CPlaneNode::init_node()
{
  b_valid_feature_projec_.resize(feature_locations_3d_.size(), false);
  string vro_strategy = ParamSrvMi::instanceMi()->get<std::string>("vro_strategy");
  if(vro_strategy == "vro_plane" || vro_strategy == "vro_plane_em") // use vro_plane for VO 
  {
     intersect_to_planes();
  }
  filter_pc();
}

void CPlaneNode::filter_pc()
{
  if(pc_col->points.size() <=0)
    return; 
  
  ros::NodeHandle nh("~"); 
  bool b_downsample_pc;
  nh.param("b_downsample_pc", b_downsample_pc, false);
   
  if(b_downsample_pc)
  {
    // voxelgrid filter
    pointcloud_type::Ptr out(new pointcloud_type);
    double voxel_size; 
    nh.param("filter_voxel_size", voxel_size, 0.02);
    pcl::VoxelGrid<point_type> vog;
    vog.setInputCloud(pc_col); 
    vog.setLeafSize(voxel_size, voxel_size, voxel_size);
    vog.filter(*out);
    pc_col.swap(out);
  }
}

/*
bool CPlaneNode::intersect_to_plane()
{
  CPlane* p = new CPlane();
  if(pc_col->points.size() <= 0)
  {
    delete p;
    return false;
  }
  p->computeParameters<point_type>(pc_col); 
  
  // TODO: check the plane's valibility 
  
  // compute the intersection point of the features 
  int M = feature_locations_2d_.size(); 
  // VFPair index(M);
  // cv::Point2f p2d;
  // for(int i=0; i<M; i++)
  // {
  //  p2d = feature_locations_2d_[i].pt;
  //  index[i].first = p2d.x; 
  //  index[i].second = p2d.y;
  // }
  
  VectorPF3 feature_loc(M);
  for(int i=0; i<M; i++)
  {
    Eigen::Vector4f& pt = feature_locations_3d_[i]; 
    feature_loc[i][0] = pt(0); feature_loc[i][1] = pt(1); feature_loc[i][2] = pt(2);
  }

  // VectorPF3 f_locs = genSR4k_set(p, index);
  VectorPF3 f_locs = genSR4k_set(p, feature_loc);
  assert(f_locs.size() == M);

  // do not use the points whose' projection distance is so small 
  // double sigma = p->computeSigma(); 

  feature_locations_3d_on_plane_.resize(M);
  b_valid_feature_projec_.resize(M);
  for(int i=0; i<M; i++)
  {
    CPointF3& p1 = feature_loc[i];
    CPointF3& p2 = f_locs[i];
    double sq_dis = SQ(p2[0] - p1[0]) + SQ(p2[1] - p1[1]) + SQ(p2[2] - p1[2]);
  
    if(sq_dis > SQ(0.02)) // not necessary to project it 
    {
      feature_locations_3d_on_plane_[i](0) = NAN; // std::numeric_limits<float>::signaling_NAN(); 
      feature_locations_3d_on_plane_[i](1) = NAN; // std::numeric_limits<float>::signaling_NAN();
      feature_locations_3d_on_plane_[i](2) = NAN; // std::numeric_limits<float>::signaling_NAN(); 
      feature_locations_3d_on_plane_[i](3) = 1.0;
      b_valid_feature_projec_[i] = false;
    }
   else
    {
      feature_locations_3d_on_plane_[i](0) = f_locs[i][0]; 
      feature_locations_3d_on_plane_[i](1) = f_locs[i][1];
      feature_locations_3d_on_plane_[i](2) = f_locs[i][2]; 
      feature_locations_3d_on_plane_[i](3) = 1.0;
      b_valid_feature_projec_[i] = true;
    }
  }

  delete p;
  b_has_been_projected_ = true;
  return true;
}*/

void CPlaneNode::points2Plane(CPlane* p, vector<int>& index) // index indicates for each point belong to which plane
{
  int N = pc_col->points.size(); 
  index.resize(N, -1);
  int boarder = 10;  
  int i, which_plane;
  
  // int M = pset->v_plane_set_.size();
  double min_dis;
  CPointF3 pt;
  for(int row = boarder; row < sr_rows - boarder; row++ )
    for(int col = boarder; col < sr_cols - boarder; col++)
    {
      i = col * sr_rows + row; 
      pcl::PointXYZRGB& pcl_pt = pc_col->points[i];
      pt = CPointF3(pcl_pt.x, pcl_pt.y, pcl_pt.z);
      min_dis = p->dis2plane(pt); 
    
      if(min_dis < 0.02)  // TODO: make it as a parameter, this is the min_dis for plane fitting
      {
        index[i] = 0;
      }
    }
  return ;
}

void CPlaneNode::points2Plane(CPlaneSet* pset, vector<int>& index) // index indicates for each point belong to which plane
{
  int N = pc_col->points.size(); 
  index.resize(N, -1);
  int boarder = 5;  
  int i, which_plane;
  
  int M = pset->v_plane_set_.size();
  double min_dis;
  CPointF3 pt;
  
  // ROS_ERROR("plane_node.cpp: N = %d, while sr_n = %d", N, sr_rows*sr_cols);

  for(int row = boarder; row < sr_rows - boarder; row++ )
    for(int col = boarder; col < sr_cols - boarder; col++)
    {
      i = col * sr_rows + row; 
      pcl::PointXYZRGB& pcl_pt = pc_col->points[i];
      pt = CPointF3(pcl_pt.x, pcl_pt.y, pcl_pt.z);
      // ROS_INFO("plane_node.cpp: point (%d,%d) at (%f, %f, %f) ", row, col, pt[0], pt[1], pt[2]);
      min_dis = pset->v_plane_set_[0]->dis2plane(pt); 
      // ROS_INFO("plane_node.cpp: distance to plane 0 %f", min_dis);
      which_plane = 0;
      for(int j=1; j<M; j++)
      {
        double tmp_dis = pset->v_plane_set_[j]->dis2plane(pt); 
        if(tmp_dis < min_dis)
        {
          min_dis = tmp_dis; 
          which_plane = j;
        }
      } 
      
      if(min_dis < 0.02)  // TODO: make it as a parameter, this is the min_dis for plane fitting
      {
        index[i] = which_plane;
      }
    }
}

void CPlaneNode::handle_with_plane(int M, VectorPF3& feature_loc, VectorPF3& f_locs, vector<bool>& b_belong_to_plane)
{
  feature_locations_3d_on_plane_.resize(M);
  b_valid_feature_projec_.resize(M);
  for(int i=0; i<M; i++)
  {
    if(b_belong_to_plane[i])
    {
      {
        feature_locations_3d_on_plane_[i](0) = f_locs[i][0]; 
        feature_locations_3d_on_plane_[i](1) = f_locs[i][1];
        feature_locations_3d_on_plane_[i](2) = f_locs[i][2]; 
        feature_locations_3d_on_plane_[i](3) = 1.0;
        b_valid_feature_projec_[i] = true; // true
      }
    }else // this feature is not on any plane
    {
        feature_locations_3d_on_plane_[i](0) = NAN; // std::numeric_limits<float>::signaling_NAN(); 
        feature_locations_3d_on_plane_[i](1) = NAN; // std::numeric_limits<float>::signaling_NAN();
        feature_locations_3d_on_plane_[i](2) = NAN; // std::numeric_limits<float>::signaling_NAN(); 
        feature_locations_3d_on_plane_[i](3) = 1.0;
        b_valid_feature_projec_[i] = false; // true;
    }
  }
}

/*
void CPlaneNode::handle_with_plane(int M, VectorPF3& feature_loc, VectorPF3& f_locs, vector<bool>& b_belong_to_plane)
{
  vector<int> index; // index the item to be kept
  feature_locations_3d_on_plane_.resize(M);
  b_valid_feature_projec_.resize(M);
  for(int i=0; i<M; i++)
  {
    if(b_belong_to_plane[i])
    {
      CPointF3& p1 = feature_loc[i];
      CPointF3& p2 = f_locs[i];
      double sq_dis = SQ(p2[0] - p1[0]) + SQ(p2[1] - p1[1]) + SQ(p2[2] - p1[2]);

      if(sq_dis > SQ(0.05)) // not necessary to project it, delete 
      {
        feature_locations_3d_on_plane_[i](0) = NAN; // std::numeric_limits<float>::signaling_NAN(); 
        feature_locations_3d_on_plane_[i](1) = NAN; // std::numeric_limits<float>::signaling_NAN();
        feature_locations_3d_on_plane_[i](2) = NAN; // std::numeric_limits<float>::signaling_NAN(); 
        feature_locations_3d_on_plane_[i](3) = 1.0;
        b_valid_feature_projec_[i] = false;
      }
      else
      {
         feature_locations_3d_on_plane_[i](0) = f_locs[i][0]; 
         feature_locations_3d_on_plane_[i](1) = f_locs[i][1];
         feature_locations_3d_on_plane_[i](2) = f_locs[i][2]; 

        // feature_locations_3d_[i](0) = f_locs[i][0]; 
        // feature_locations_3d_[i](1) = f_locs[i][1];
        // feature_locations_3d_[i](2) = f_locs[i][2];

        // feature_locations_3d_on_plane_[i](0) = NAN; // std::numeric_limits<float>::signaling_NAN(); 
        // feature_locations_3d_on_plane_[i](1) = NAN; // std::numeric_limits<float>::signaling_NAN();
        // feature_locations_3d_on_plane_[i](2) = NAN; // std::numeric_limits<float>::signaling_NAN(); 

        feature_locations_3d_on_plane_[i](3) = 1.0;
        index.push_back(i);
        b_valid_feature_projec_[i] = true; // true
      }
    }else // this feature is not on any plane
    {
        feature_locations_3d_on_plane_[i](0) = NAN; // std::numeric_limits<float>::signaling_NAN(); 
        feature_locations_3d_on_plane_[i](1) = NAN; // std::numeric_limits<float>::signaling_NAN();
        feature_locations_3d_on_plane_[i](2) = NAN; // std::numeric_limits<float>::signaling_NAN(); 
        feature_locations_3d_on_plane_[i](3) = 1.0;
        index.push_back(i);
        b_valid_feature_projec_[i] = false; // true;
    }
  }

  // copy the rest 
  // copy buf

  M = index.size();
  std::vector<cv::KeyPoint> tmp_feature_2d(M);
  cv::Mat tmp_feature_descriptors(M, feature_descriptors_.cols, CV_32FC1);
  std_vector_of_eigen_vector4f tmp_feature_3d(M); 
  std_vector_of_eigen_vector4f tmp_feature_3d_plane(M);
  vector<bool> tmp_valid_feature_proj(M);
 
  for(int i=0; i<M; i++)
  {
    int k = index[i];
    tmp_feature_2d[i] = feature_locations_2d_[k]; 
    tmp_feature_3d[i] = feature_locations_3d_[k];
    tmp_feature_3d_plane[i] = feature_locations_3d_on_plane_[k];
    tmp_valid_feature_proj[i] = b_valid_feature_projec_[k];
    for(int j=0; j<feature_descriptors_.cols; j++)
    {
      feature_descriptors_.row(k).copyTo(tmp_feature_descriptors.row(i));
    }
  }

  {
    feature_locations_2d_.swap(tmp_feature_2d); 
    feature_locations_3d_.swap(tmp_feature_3d);
    feature_locations_3d_on_plane_.swap(tmp_feature_3d_plane);
    b_valid_feature_projec_.swap(tmp_valid_feature_proj);
    feature_descriptors_ = tmp_feature_descriptors;

    if(flannIndex != 0) 
    {
      delete flannIndex; 
      flannIndex = 0;
    }
  }

  return;
}
*/

bool CPlaneNode::intersect_to_plane(CPlane* p)
{
  // compute the intersection point of the features 
  int M = feature_locations_2d_.size(); 
  VectorPF3 feature_loc(M);
  VPair feature_loc_2d(M); 
  for(int i=0; i<M; i++)
  {
    Eigen::Vector4f& pt = feature_locations_3d_[i]; 
    feature_loc[i][0] = pt(0); feature_loc[i][1] = pt(1); feature_loc[i][2] = pt(2);
    cv::KeyPoint& p2d = feature_locations_2d_[i]; 
    feature_loc_2d[i].first = round(p2d.pt.x ); 
    feature_loc_2d[i].second = round(p2d.pt.y );
  }
  
  // VectorPF3 f_locs = genSR4k_set(p, index);
  vector<int> p_index;
  points2Plane(p, p_index); 
  vector<bool> b_belong_to_plane;
  
  // ROS_INFO("before genSR4k_set!");
  VectorPF3 f_locs = genSR4k_set(p, feature_loc, feature_loc_2d, p_index, b_belong_to_plane);
  // VectorPF3 f_locs = genSR4k_set(p, feature_loc);
  
  ROS_INFO("plane_node.cpp: after genSR4k_set()");
  assert(f_locs.size() == M);
  handle_with_plane(M, feature_loc, f_locs, b_belong_to_plane);
  
  return true;
}

bool CPlaneNode::intersect_to_plane2()
{
  if(pc_col->points.size() <= 0)
  {
    return false;
  }
  CPlane* p = new CPlane();
  p->computeParameters<point_type>(pc_col); 
  intersect_to_plane(p);

  delete p;
  b_has_been_projected_ = true;
  ROS_WARN("finish intersect_with planes");
  return true;
}

bool CPlaneNode::intersect_to_floor()
{
  if(pc_col->points.size() <= 0)
  {
    return false;
  }
  CPlaneSet *pset = new CPlaneSet(); 
  int number_of_planes = pset->extractPlanes(pc_col); 
  // ROS_WARN("plane_node.cpp: retrive %d planes", number_of_planes); 
  if(number_of_planes <= 0)
  {
    ROS_ERROR("plane_node.cpp: failed to extract any plane!");
    delete pset; 
    return false; 
  }
  
  // compute the intersection point of the features 
  int M = feature_locations_2d_.size(); 
  VectorPF3 feature_loc(M);
  VPair feature_loc_2d(M); 
  for(int i=0; i<M; i++)
  {
    Eigen::Vector4f& pt = feature_locations_3d_[i]; 
    feature_loc[i][0] = pt(0); feature_loc[i][1] = pt(1); feature_loc[i][2] = pt(2);
    cv::KeyPoint& p2d = feature_locations_2d_[i]; 
    feature_loc_2d[i].first = round(p2d.pt.x ); 
    feature_loc_2d[i].second = round(p2d.pt.y );
  }

  CPlane* p = pset->selectFloor(); 
  if(p == NULL)
  {
    ROS_ERROR("plane_node.cpp: fail to select floor from frame");
    delete pset; 
    return false;
  }
 
  intersect_to_plane(p);

  delete pset; 
  return true;
}


void CPlaneNode::remove_sift_points(pointcloud_type::Ptr& pOut)
{
  pcl::PointIndices::Ptr index_set(new pcl::PointIndices);
  
  // add all the index of points around sift points 
  int row,col;
  int M = feature_locations_2d_.size();
  for(int i=0; i<M; i++)
  {
      cv::KeyPoint& p2d = feature_locations_2d_[i]; 
      col = round(p2d.pt.x ); 
      row = round(p2d.pt.y );
      for(int v=row - 2; v<=row +2; v++)
      {
        for(int u=col - 2; u<=col + 2; u++)
        {
          if(u < 0 || u >= 176 || v <0 || v >= 144 )
          {
            continue; 
          }
          int index = uv2index(u,v); 
          index_set->indices.push_back(index); 
        }
      }
  }
  ((CPlaneSet*)0)->extractOutlier<point_type>(pc_col, pOut, index_set);
  
  return ;
}

bool CPlaneNode::intersect_to_planes() // 
{
  if(pc_col->points.size() <= 0)
  {
    return false;
  }
  if(b_has_been_projected_)
  {
    return true;
  }

  CPlaneSet *pset = new CPlaneSet(); 

  // extract plane without sift points
  pointcloud_type::Ptr tmp_pc(new pointcloud_type); 
  remove_sift_points(tmp_pc);

  // int number_of_planes = pset->extractPlanes(pc_col); 
  int number_of_planes = pset->extractPlanes(tmp_pc);
  // ROS_WARN("plane_node.cpp: retrive %d planes", number_of_planes); 
  if(number_of_planes <= 0)
  {
    ROS_ERROR("plane_node.cpp: failed to extract any plane!");
    delete pset; 
    return false; 
  }
  
  // compute the intersection point of the features 
  int M = feature_locations_2d_.size(); 
  VectorPF3 feature_loc(M);
  VPair feature_loc_2d(M); 
  for(int i=0; i<M; i++)
  {
    Eigen::Vector4f& pt = feature_locations_3d_[i]; 
    feature_loc[i][0] = pt(0); feature_loc[i][1] = pt(1); feature_loc[i][2] = pt(2);
    cv::KeyPoint& p2d = feature_locations_2d_[i]; 
    feature_loc_2d[i].first = round(p2d.pt.x ); 
    feature_loc_2d[i].second = round(p2d.pt.y );
  }

  // ROS_WARN("plane_node.cpp: before points2plane");
  // VectorPF3 f_locs = genSR4k_set(p, index);
  vector<int> p_index;
  points2Plane(pset, p_index); 
  // ROS_WARN("plane_node.cpp: after points2plane");

  // ROS_WARN("plane_node.cpp: before genSR4k_set");
  vector<bool> b_belong_to_plane;
  VectorPF3 f_locs = genSR4k_set(pset, feature_loc, feature_loc_2d, p_index, b_belong_to_plane);
  // ROS_WARN("plane_node.cpp: after genSR4k_set");

  // VectorPF3 f_locs = genSR4k_set(p, feature_loc);
  assert(f_locs.size() == M);

  // do not use the points whose' projection distance is so small 
  // double sigma = p->computeSigma(); 
  
  handle_with_plane(M, feature_loc, f_locs, b_belong_to_plane);

  delete pset;
  b_has_been_projected_ = true;
  // ROS_WARN("finish intersect_with planes");
  return true;
}

bool CPlaneNode::intersect_to_plane() // delete all the points not close to the fitted plane
{
  if(pc_col->points.size() <= 0)
  {
    return false;
  }
  CPlane* p = new CPlane();
  p->computeParameters<point_type>(pc_col); 
  
  // TODO: check the plane's valibility 
  
  // compute the intersection point of the features 
  
  int M = feature_locations_2d_.size(); 
  
  VectorPF3 feature_loc(M);
  for(int i=0; i<M; i++)
  {
    Eigen::Vector4f& pt = feature_locations_3d_[i]; 
    feature_loc[i][0] = pt(0); feature_loc[i][1] = pt(1); feature_loc[i][2] = pt(2);
  }

  // VectorPF3 f_locs = genSR4k_set(p, index);
  VectorPF3 f_locs = genSR4k_set(p, feature_loc);
  
  // ROS_INFO("plane_node.cpp: after genSR4k_set()");
  assert(f_locs.size() == M);

  // do not use the points whose' projection distance is so small 
  // double sigma = p->computeSigma(); 

  // ROS_INFO("plane_node.cpp: after assert()");
  vector<int> index; // index the item to be kept
  feature_locations_3d_on_plane_.resize(M);
  b_valid_feature_projec_.resize(M);
  for(int i=0; i<M; i++)
  {
    CPointF3& p1 = feature_loc[i];
    CPointF3& p2 = f_locs[i];
    double sq_dis = SQ(p2[0] - p1[0]) + SQ(p2[1] - p1[1]) + SQ(p2[2] - p1[2]);
  
    if(sq_dis > SQ(0.05)) // not necessary to project it, delete 
    {
      feature_locations_3d_on_plane_[i](0) = NAN; // std::numeric_limits<float>::signaling_NAN(); 
      feature_locations_3d_on_plane_[i](1) = NAN; // std::numeric_limits<float>::signaling_NAN();
      feature_locations_3d_on_plane_[i](2) = NAN; // std::numeric_limits<float>::signaling_NAN(); 
      feature_locations_3d_on_plane_[i](3) = 1.0;
      b_valid_feature_projec_[i] = false;
    }
   else
    {
      feature_locations_3d_on_plane_[i](0) = f_locs[i][0]; 
      feature_locations_3d_on_plane_[i](1) = f_locs[i][1];
      feature_locations_3d_on_plane_[i](2) = f_locs[i][2]; 
      feature_locations_3d_on_plane_[i](3) = 1.0;
      index.push_back(i);
      b_valid_feature_projec_[i] = true;
    }
  }

  // copy the rest 
  // copy buf

  M = index.size();
  std::vector<cv::KeyPoint> tmp_feature_2d(M);
  cv::Mat tmp_feature_descriptors(M, feature_descriptors_.cols, CV_32FC1);
  std_vector_of_eigen_vector4f tmp_feature_3d(M); 
  std_vector_of_eigen_vector4f tmp_feature_3d_plane(M);
  vector<bool> tmp_valid_feature_proj(M);
 
  for(int i=0; i<M; i++)
  {
    int k = index[i];
    tmp_feature_2d[i] = feature_locations_2d_[k]; 
    tmp_feature_3d[i] = feature_locations_3d_[k];
    tmp_feature_3d_plane[i] = feature_locations_3d_on_plane_[k];
    tmp_valid_feature_proj[i] = b_valid_feature_projec_[k];
    for(int j=0; j<feature_descriptors_.cols; j++)
    {
      feature_descriptors_.row(k).copyTo(tmp_feature_descriptors.row(i));
    }
  }

  {
    feature_locations_2d_.swap(tmp_feature_2d); 
    feature_locations_3d_.swap(tmp_feature_3d);
    feature_locations_3d_on_plane_.swap(tmp_feature_3d_plane);
    b_valid_feature_projec_.swap(tmp_valid_feature_proj);
    feature_descriptors_ = tmp_feature_descriptors;

    if(flannIndex != 0) 
    {
      delete flannIndex; 
      flannIndex = 0;
    }
  }

  delete p;
  b_has_been_projected_ = true;
  return true;
}

bool CPlaneNode::isValid(std_vector_of_eigen_vector3f& pts)
{
  for(int i=0; i<pts.size(); i++)
  {
    if(!isValid(pts[i]))
      return false;
  }
  return true;
}

bool CPlaneNode::isValid(Eigen::Vector3f& p) // check the validity of a point
{
  if(isnan(p(0)) || isnan(p(1)) || isnan(p(2)))
    return false; 
  // if(p(2) <= 0) // p(2) 
    // return false;
  return true;
}

Eigen::Vector3f CPlaneNode::selectPos(int select_code, int idx) const
{
  if(select_code == 0)
    return feature_locations_3d_[idx].head<3>();
 // else if(select_code == 1)
  else
    return feature_locations_3d_on_plane_[idx].head<3>();
  
  ROS_ERROR("plane_node.cpp: should never arrive here select_code = %d", select_code);
  return Eigen::Vector3f::Identity();
}

void CPlaneNode::selectPosGivenCode(Node* older_node, int code, std::vector<cv::DMatch>& matches, std_vector_of_eigen_vector3f& fp, std_vector_of_eigen_vector3f& tp)
{
  CPlaneNode* p_older_node = dynamic_cast<CPlaneNode*>(older_node);
  int d_code;
  int sample_size = matches.size();
  for(int i=0; i<sample_size; i++)
  {
    d_code = 1<<(2*i);
    // ROS_INFO("plane_node.cpp: i = %d code %d, d_code = %d and result %d", i, code, d_code, code& d_code);
    fp[i] = selectPos(code & d_code, matches[i].queryIdx); 
    d_code = 1<<(2*i+1);
    // ROS_INFO("plane_node.cpp: i = %d code %d, d_code = %d and result %d", i, code, d_code, code & d_code);
    tp[i] = p_older_node->selectPos(code & d_code, matches[i].trainIdx);
  }
  return ;
}

/*
void CPlaneNode::selectPosGivenCode(Node* older_node, int code, std::vector<cv::DMatch>& matches, Eigen::Vector3f& fp1, Eigen::Vector3f& fp2, Eigen::Vector3f& fp3, Eigen::Vector3f& tp1, Eigen::Vector3f& tp2, Eigen::Vector3f& tp3)
{
    assert(i>=0 && i<64);
    CPlaneNode* p_older_node = dynamic_cast<CPlaneNode*>(older_node);
    fp1 = selectPos(code & 0x00100000, matches[0].queryIdx);
    tp1 = p_older_node->selectPos(code & 0x00010000, matches[0].trainIdx); 
    fp2 = selectPos(code & 0x00001000, matches[1].queryIdx); 
    tp2 = p_older_node->selectPos(code & 0x00000100, matches[1].trainIdx); 
    fp3 = selectPos(code & 0x00000010, matches[2].queryIdx);
    tp3 = p_older_node->selectPos(code & 0x00000001, matches[2].trainIdx);
    return ;
}
*/

vector<int> CPlaneNode::computeConsistentSet(Node* older_node, std::vector<cv::DMatch>& matches, float max_dist_m)
{
  vector<int> ret; 
  CPlaneNode* p_older_node = dynamic_cast<CPlaneNode*>(older_node);

  int sample_size = matches.size(); 
  assert(sample_size <= 5);
  int N_combination = 1<<(2*sample_size);
  vector<float> delta_f(sample_size-1); 
  vector<float> delta_t(sample_size-1);
  std_vector_of_eigen_vector3f fp(sample_size);
  std_vector_of_eigen_vector3f tp(sample_size);

  for(int i=0; i<N_combination; i++)
  {
    selectPosGivenCode(older_node, i, matches, fp, tp); 
    if(!isValid(fp) || !isValid(tp))
      continue; 
    
    bool valid = true; 
    // compute delta 
    for(int j=0; j<sample_size-1; j++)
    {
      delta_f[j] = (fp[j+1]-fp[j]).squaredNorm();
      delta_t[j] = (tp[j+1]-tp[j]).squaredNorm(); 
      if(SQ(delta_f[j] - delta_t[j]) > SQ(max_dist_m)) // not a good match
      { 
        valid = false; 
        break;
        // continue;
      }
    }
    if(valid)
      ret.push_back(i);
  }
  // ROS_INFO("plane_node.cpp: computeConsistentSet finished, set size %u", ret.size());
  return ret;
}

/*
vector<int> CPlaneNode::computeConsistentSet(Node* older_node, std::vector<cv::DMatch>& matches)
{
  vector<int> ret; 
  CPlaneNode* p_older_node = dynamic_cast<CPlaneNode*>(older_node);

  // all possible combinations 4*4*4  
  float delta_f1, delta_f2, delta_t1, delta_t2;
  Eigen::Vector3f fp1, fp2, fp3; 
  Eigen::Vector3f tp1, tp2, tp3;
  for(int i=0; i<64; i++)
  {
    // 64  = 0x01000000
    //
    selectPosGivenCode(old_node, i, fp1, fp2, fp3, tp1, tp2, tp3);
    
    if(!isValid(fp1) || !isValid(fp2) || !isValid(fp3) \
        !isValid(tp1) || !isValid(tp2) || !isValid(tp3))
      continue;

    delta_f1 = (fp2 - fp1).squaredNorm(); // distance between points p1  and  p2
    delta_t1 = (tp2 - tp1).squaredNorm(); // distance between points p1' and  p2' 
    
    if(fabs(delta_f1 - delta_t1) > SQ(max_dist_m)) // not a good match
      continue;
    
    delta_f2 = (fp3 - fp2).squaredNorm(); // distance between points p2  and  p3
    delta_t2 = (tp3 - tp2).squaredNorm(); // distance between points p2' and  p3'
    
    if(fabs(delta_f2 - delta_t2) > SQ(max_dist_m)) // not a good match 
      continue; 
    
    ret.push_back(i);  // i is a good match
  }
  return ret;
}*/

Eigen::Matrix4f CPlaneNode::getTransfromFromMatchesWithProject(const CPlaneNode* old_node, std::vector<cv::DMatch>& matches, vector<bool>& code_src, vector<bool>& code_tar, bool& valid, float max_dist_m) const
{
  pcl::TransformationFromCorrespondences tfc;
  valid = true;
  std::vector<Eigen::Vector3f> t, f;
  
  for(int i=0; i<matches.size(); i++)
  {
    cv::DMatch& m = matches[i]; 
    Eigen::Vector3f from = selectPos(code_src[i]?1:0, m.queryIdx); 
    Eigen::Vector3f to = old_node->selectPos(code_tar[i]?1:0, m.trainIdx); 
    if(!isValid(from) || !isValid(to))
      continue;
    float weight = 1.; //1./(from(2) + to(2));
    
    if(max_dist_m > 0)
    {
      if(f.size() >= 1)
      {
        float delta_f = (from - f.back()).squaredNorm(); 
        float delta_t = (to - t.back()).squaredNorm(); 
        
        // double abs_dis = fabs(delta_f - delta_t); 
        // Eigen::Vector3f s_f = from - f.back(); 
        // Eigen::Vector3f t_f = to - t.back();
        // cout<<"delta_sf: "<<s_f(0)<<" "<<s_f(1)<<" "<<s_f(2)<< 
        // " delta_tf: "<<t_f(0)<<" "<<t_f(1)<<" "<<t_f(2)<<endl;
        // cout<<"length_delta_sf: "<<s_f.squaredNorm()<<" length_delta_tf: "<<t_f.squaredNorm()<<endl;
        // ROS_WARN("plane_node.cpp: what's the distance %f", abs_dis);

        // changed at Oct.12 by David Zhang
        if(SQ(delta_f - delta_t) > SQ(max_dist_m))
        // if(fabs(delta_f - delta_t) > SQ(max_dist_m))  // this cause the problem!, not thee fabs but the square distance
        {
          valid = false; 
          return Eigen::Matrix4f(); 
        }
      }
      f.push_back(from); 
      t.push_back(to);
    }
    tfc.add(from, to, weight); 
  }
  
  return tfc.getTransformation().matrix();

}

std_vector_of_eigen_matrix4f CPlaneNode::getTransfromFromMatchesCombination(Node* old_node, std::vector<cv::DMatch>& matches,  bool& valid, float max_dist_m)
{
  CPlaneNode* p_old_node = dynamic_cast<CPlaneNode*>(old_node);
  std_vector_of_eigen_matrix4f ret;
  if(!b_has_been_projected_ || !p_old_node->b_has_been_projected_)
  {
    // ROS_WARN("plane_node.cpp: plane points are not avaliable, use the original matches");
    int sample_size = matches.size();
    static vector<bool> c_src(sample_size, false);
    static vector<bool> c_tar(sample_size, false);
    Eigen::Matrix4f T = getTransfromFromMatchesWithProject(old_node, matches, c_src, c_tar, valid, max_dist_m*2.5);
    // Eigen::Matrix4f T = getTransformFromMatches(this, old_node, matches, valid, max_dist_m*4); 
    ret.push_back(T);
    return ret; 
  }

  vector<int> good_code = computeConsistentSet(old_node, matches, max_dist_m*4);
  if(good_code.size() <=0 )
  {
    // ROS_ERROR("plane_node.cpp: good_code.size() <= 0");
    valid = false; 
    return ret;
  }

  int sample_size = matches.size();
  std_vector_of_eigen_vector3f fp(sample_size);
  std_vector_of_eigen_vector3f tp(sample_size);
 
  vector<float> w(sample_size); 

  // std::vector<Eigen::Vector3f> t(sample_size);
  // std::vector<Eigen::Vector3f> f(sample_size);

  for(int i=0; i<good_code.size(); i++)
  {
    pcl::TransformationFromCorrespondences tfc; 
    selectPosGivenCode(old_node, good_code[i], matches, fp, tp);
    for(int j=0; j<sample_size; j++)
    {
      // w[j] = 1./(fp[j](2) + tp[j](2));
      w[j] = 1.;
      tfc.add(fp[j], tp[j], w[j]);
    }
    Eigen::Matrix4f TR = tfc.getTransformation().matrix(); 
    ret.push_back(TR);
  }
  return ret;
}


bool CPlaneNode::VRO_ori(const Node* older_node, Eigen::Matrix4f& trans) 
{
  MatchingResult mr = Node::matchNodePair(older_node);
  // result 
  trans = mr.final_trafo;

  return true;
}

void CPlaneNode::setRPY(float* rpy)
{
  rpy_[0] = (*rpy++); rpy_[1] = (*rpy++); rpy_[2] = *rpy;
}

void CPlaneNode::diffRPY(CPlaneNode* earlier_node, double& r, double& p, double& y)
{
  // pose of earlier node 
  tf::Matrix3x3 old_r; old_r.setEulerZYX(earlier_node->rpy_[2], earlier_node->rpy_[1], earlier_node->rpy_[0]); 
  tf::Transform odo_tf_old; odo_tf_old.setBasis(old_r); 
  
  // pose of current node 
  tf::Matrix3x3 new_r; new_r.setEulerZYX(rpy_[2], rpy_[1], rpy_[0]); 
  tf::Transform odo_tf_new; odo_tf_new.setBasis(new_r); 

  // transformation from the earlier pose to the current pose 
  tf::Transform inc_t = odo_tf_old.inverse() * odo_tf_new; 

  // compute the euler transformation 
  inc_t.getBasis().getEulerZYX(y, p, r); 
  return ;
}

/*
MatchingResult CPlaneNode::matchNodePair(const Node* older_node)
{
  ROS_ERROR("plane_node.cpp: for debug, in CPlaneNode::matchNodePair()");
  // return VRO(older_node);
  return Node::matchNodePair(older_node);
}*/

MatchingResult CPlaneNode::VRO(const Node* older_node)
{
  MatchingResult mr; 
  // ROS_WARN("plane_node.cpp: in PlaneNode VRO!");
  try{
    bool found_transformation = false;
    const unsigned int min_matches = (unsigned int) ParameterServer::instance()->get<int>("min_matches");
    this->featureMatching(older_node, &mr.all_matches); 
    if (mr.all_matches.size() < min_matches)
    {
      ROS_INFO("Too few inliers between %i and %i for RANSAC method. Only %i correspondences to begin with.",
          older_node->id_,this->id_,(int)mr.all_matches.size());
    } 
    else 
    {
      /*{
        // for debug 
        ifstream inf_match("/home/davidz/work/data/SwissRanger4000/dat/initial_match.log");
        int M = 251; 
        mr.all_matches.resize(M); 
        int trainIdx, queryIdx; 
        for(int j=0; j<251; j++)
        { 
          inf_match>>trainIdx>>queryIdx; 
          mr.all_matches[j].trainIdx = trainIdx -1;   // different index from matlab 
          mr.all_matches[j].queryIdx = queryIdx -1;
          // ROS_INFO("j = %d trainIdx %d queryIdx %d", j+1, trainIdx, queryIdx);
        }
      }*/

      // ROS_WARN("plane_node.cpp: featureMatching, sift matched points %d !", mr.all_matches.size());
      //All good for feature based transformation estimation
      found_transformation = getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches);
      // found_transformation = getRelativeTransformationToNew(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches); 
      // found_transformation = getRelativeTransformationToNew_ori(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches); 

      // ROS_WARN("plane_node.cpp: after Transformation!");

      //Statistics
      float nn_ratio = 0.0;
      if(found_transformation)
      {
        for(unsigned int i = 0; i < mr.inlier_matches.size(); i++){
          nn_ratio += mr.inlier_matches[i].distance;
        }
        nn_ratio /= mr.inlier_matches.size();
        //ParameterServer::instance()->set("nn_distance_ratio", static_cast<double>(nn_ratio + 0.2));
        mr.final_trafo = mr.ransac_trafo;
        mr.edge.informationMatrix =   Eigen::Matrix<double,6,6>::Identity()*(mr.inlier_matches.size()/(mr.rmse*mr.rmse)); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)

        mr.edge.id1 = older_node->id_;//and we have a valid transformation
        mr.edge.id2 = this->id_; //since there are enough matching features,
        mr.edge.transform = mr.final_trafo.cast<double>();//we insert an edge between the frames

      found_transformation = true; 

      ROS_INFO("RANSAC found a %s transformation with %d inliers matches with average ratio %f", found_transformation? "valid" : "invalid", (int) mr.inlier_matches.size(), nn_ratio);
    }else{
      for(unsigned int i = 0; i < mr.all_matches.size(); i++){
        nn_ratio += mr.all_matches[i].distance;
      }
      nn_ratio /= mr.all_matches.size();
      ROS_INFO("RANSAC found no valid trafo, but had initially %d feature matches with average ratio %f",(int) mr.all_matches.size(), nn_ratio);

    } 
    if(found_transformation) {
      // ROS_INFO("Returning Valid Edge");
      // ++initial_node_matches_; //trafo is accepted
    }
    else {
      mr.edge.id1 = -1;
      mr.edge.id2 = -1;
    }
    }
  }
  catch (std::exception e){
    ROS_ERROR("Caught Exception in comparison of Nodes %i and %i: %s", this->id_, older_node->id_, e.what());
  }

  // for debug 
      ros::NodeHandle nh("~");
      bool b_dis_match_point; 
      bool b_dis_initial_match_point; 
      nh.param<bool>("vro_display_matches", b_dis_match_point, false);
      nh.param<bool>("vro_display_initial_matches", b_dis_initial_match_point, false);
      if(b_dis_match_point)
      {
        // display the matched features 
        int M = mr.inlier_matches.size(); 
        // vector<cv::KeyPoint> kp1(M); 
        // vector<cv::KeyPoint> kp2(M);  
        // for(int i=0; i<M; i++)
        {
        //  kp1[i] = older_node->feature_locations_2d_[mr.inlier_matches[i].trainIdx];
        //  kp2[i] = feature_locations_2d_[mr.inlier_matches[i].queryIdx];
        }
        /*
        // record the matched points 
        ofstream inlier_f("./vro/inlier_pts.log"); 
        for(int i=0; i<M; i++)
        {
          Eigen::Vector4f src_pt = feature_locations_3d_[mr.inlier_matches[i].queryIdx];
          Eigen::Vector4f tar_pt = older_node->feature_locations_3d_[mr.inlier_matches[i].trainIdx];
          inlier_f<<src_pt(0)<<" "<<src_pt(1)<<" "<<src_pt(2)<<"; "<<tar_pt(0)<<" "<<tar_pt(1)<<" "<<tar_pt(2)<<endl;
        }
        inlier_f.close();

        // print result 
        ROS_INFO("plane_node.cpp: return trans: ");
        display_eigen(mr.final_trafo);
        */
        // cv::Mat img1(d_rows_, d_cols_, CV_8UC1);
        // cv::Mat img2(d_rows_, d_cols_, CV_8UC1);
        cv::Mat img1 = getIntensity_img();
        cv::Mat img2 = ((const CPlaneNode*)older_node)->getIntensity_img();
        cv::Mat outImg, outImgIni;
        
        if(b_dis_initial_match_point)
        {
          cv::drawMatches(img1, feature_locations_2d_, img2, older_node->feature_locations_2d_, mr.all_matches, outImgIni);
          cv::namedWindow("debug_initial_matches",2); 
          cv::imshow("debug_initial_matches", outImgIni); 
          cv::waitKey(0);
        }

        cv::drawMatches(img1, feature_locations_2d_, img2, older_node->feature_locations_2d_, mr.inlier_matches, outImg );
        cv::namedWindow("debug_matches", 1);
        cv::imshow("debug_matches", outImg);
        cv::waitKey(0);

      }
  return mr; 
}

bool CPlaneNode::VRO( const Node* older_node, Eigen::Matrix4f& trans) 
{ 
  MatchingResult mr = VRO(older_node); 
  if(mr.inlier_matches.size() > 0)
  {
    // result 
    trans = mr.final_trafo;
    return true;
  }
  return false;
}

cv::Mat CPlaneNode::getIntensity_img()
{
  if(!intensity_img_.empty() && intensity_img_.rows > 0)
    return intensity_img_; 
  cv::Mat img(d_rows_, d_cols_, CV_8UC1);
  return img;
}

// do not incorporate covariance in compute the distance 
double CPlaneNode::errorFunctionSR4kNew(Eigen::Vector3f& x1, Eigen::Vector3f& x2, Eigen::Matrix4d& transformation)
{
  // NAN CHECK 
  bool nan1 = isnan(x1(2));
  bool nan2 = isnan(x2(2));
  if(nan1 || nan2)
  {
    //FIXME: Handle Features with NaN, by reporting the reprojection error
    return std::numeric_limits<double>::max();
  }

  // transformation 
  Eigen::Matrix4d tf_12 = transformation;
  Eigen::Vector4d x_1, x_2; 
  x_1.head<3>() = x1.cast<double>(); x_1(3) = 1.; 
  x_2.head<3>() = x2.cast<double>(); x_2(3) = 1.;
  Eigen::Vector3d mu_1 = x_1.head<3>();
  Eigen::Vector3d mu_2 = x_2.head<3>();
  Eigen::Vector3d mu_1_in_frame_2 = (tf_12 * x_1).head<3>(); // 
  double delta_sq_norm = (mu_1_in_frame_2 - mu_2).squaredNorm();

  // ROS_INFO_STREAM("plane_node.cpp: delta_sq_norm: "<<delta_sq_norm<<" mu_2: "<<mu_2(0)<<" "<<mu_2(1)<<" "<<mu_2(2)<<
  //	" mu_1_in_frame_2: "<<mu_1_in_frame_2(0)<<" "<<mu_1_in_frame_2(1)<<" "<<mu_1_in_frame_2(2));
  return delta_sq_norm;
}

double CPlaneNode::errorFunctionSR4k(Eigen::Vector3f& x1, Eigen::Vector3f& x2, Eigen::Matrix4d& transformation)
{
  // one type 44 * 35, another type 69 * 55
  static const double cam_angle_x = D2R(44.); // D2R(44.);  // 58.0/180.0*M_PI;/*{{{*/
  static const double cam_angle_y = D2R(35.); // D2R(35.);  //45.0/180.0*M_PI;
  static const double cam_resol_x = 176; // 176; // 640;
  static const double cam_resol_y = 144; // 144; // 480;
  static const double raster_stddev_x = 3*tan(cam_angle_x/cam_resol_x);  //5pix stddev in x
  static const double raster_stddev_y = 3*tan(cam_angle_y/cam_resol_y);  //5pix stddev in y
  static const double raster_cov_x = raster_stddev_x * raster_stddev_x;
  static const double raster_cov_y = raster_stddev_y * raster_stddev_y;/*}}}*/
  static const bool use_error_shortcut = true;//ParameterServer::instance()->get<bool>("use_error_shortcut");

  // NAN CHECK 
  bool nan1 = isnan(x1(2));
  bool nan2 = isnan(x2(2));
  if(nan1 || nan2)
  {
    //FIXME: Handle Features with NaN, by reporting the reprojection error
    return std::numeric_limits<double>::max();
  }

  // transformation 
  Eigen::Matrix4d tf_12 = transformation;
  Eigen::Vector4d x_1, x_2; 
  x_1.head<3>() = x1.cast<double>(); x_1(3) = 1.; 
  x_2.head<3>() = x2.cast<double>(); x_2(3) = 1.;
  Eigen::Vector3d mu_1 = x_1.head<3>();
  Eigen::Vector3d mu_2 = x_2.head<3>();
  Eigen::Vector3d mu_1_in_frame_2 = (tf_12 * x_1).head<3>(); // 
  if(use_error_shortcut)
  {
    double delta_sq_norm = (mu_1_in_frame_2 - mu_2).squaredNorm();
    double sigma_max_1 = std::max(raster_cov_x, depth_covariance(mu_1(2)));//Assuming raster_cov_x and _y to be approx. equal
    double sigma_max_2 = std::max(raster_cov_x, depth_covariance(mu_2(2)));//Assuming raster_cov_x and _y to be approx. equal
    if(delta_sq_norm > 2.0 * (sigma_max_1+sigma_max_2)) //FIXME: Factor 3 for mahal dist should be gotten from caller
    {
      return std::numeric_limits<double>::max();
    }
  } 

  Eigen::Matrix3d rotation_mat = tf_12.block(0,0,3,3);

  // Point 1
  Eigen::Matrix3d cov1 = Eigen::Matrix3d::Zero();
  cov1(0,0) = 1 * raster_cov_x * mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(1,1) = 1 * raster_cov_y * mu_1(2); //how big is 1px std dev in meter, depends on depth
  if(nan1) cov1(2,2) = 1e9; //stddev for unknown: should be within 100m
  else     cov1(2,2) = depth_covariance(mu_1(2));

  // Point 2
  Eigen::Matrix3d cov2 = Eigen::Matrix3d::Zero();
  cov2(0,0) = 1 * raster_cov_x* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov2(1,1) = 1 * raster_cov_y* mu_2(2); //how big is 1px std dev in meter, depends on depth
  if(nan2) cov2(2,2) = 1e9; //stddev for unknown: should be within 100m
  else     cov2(2,2) = depth_covariance(mu_2(2));

  Eigen::Matrix3d cov1_in_frame_2 = rotation_mat.transpose() * cov1 * rotation_mat;//Works since the cov is diagonal => Eig-Vec-Matrix is Identity
  Eigen::Vector3d delta_mu_in_frame_2 = mu_1_in_frame_2 - mu_2;
  // double delta_mu_z = delta_mu_in_frame_2(2);
  if(isnan((float)delta_mu_in_frame_2(2)))
  {
    ROS_WARN("Unexpected NaN");
    delta_mu_in_frame_2(2) = 0.0;//FIXME: Hack: set depth error to 0 if NaN 
  }

  Eigen::Matrix3d cov_mat_sum_in_frame_2 = cov1_in_frame_2 + cov2;     
  double sqrd_mahalanobis_distance = delta_mu_in_frame_2.transpose() *cov_mat_sum_in_frame_2.ldlt().solve(delta_mu_in_frame_2);

  if(!(sqrd_mahalanobis_distance >= 0.0))
  {
    return std::numeric_limits<double>::max();
  }
  return sqrd_mahalanobis_distance;
} 

bool CPlaneNode::selectMinDis(CPlaneNode* old_node, cv::DMatch& m, Eigen::Matrix4d& transformation4d,
    int code_src, int code_tar, bool& min_code_src, bool& min_code_tar, double& min_dis)
{
  if((code_src == 1 && !b_valid_feature_projec_[m.queryIdx]) || (code_tar == 1 && !old_node->b_valid_feature_projec_[m.trainIdx]))
  {
    return false;
  }
  Eigen::Vector3f src_pt, tar_pt;
  src_pt = selectPos(code_src, m.queryIdx); 
  tar_pt = old_node->selectPos(code_tar, m.trainIdx); 
  if(!isValid(src_pt) || !isValid(tar_pt))
  {
    // ROS_ERROR("plane_node.cpp: something error here invalid points src_pt: %f %f %f and tar_pt %f %f %f", src_pt(0), src_pt(1), src_pt(2), tar_pt(0), tar_pt(1), tar_pt(2));
    return false;
  }

   Eigen::Vector4f origin ;
   Eigen::Vector4f target ; 
   origin.head<3>() = src_pt; origin(3) = 1.; 
   target.head<3>() = tar_pt; target(3) = 1.;
  // double m_dis = errorFunction2(origin, target, transformation4d); 
  // double m_dis = errorFunctionSR4k(src_pt, tar_pt, transformation4d);
  double m_dis = errorFunctionSR4kNew(src_pt, tar_pt, transformation4d);

  if(m_dis < min_dis)
  {
    min_dis = m_dis; 
    // ROS_WARN("plane_node.cpp: min_dis: %f, m_dis: %f", min_dis, m_dis);
    min_code_src = (code_src==1); 
    min_code_tar = (code_tar==1);
  }
  return true;
}

void CPlaneNode::computeInliersAndErrorWithProj(const CPlaneNode* old_node, std::vector<cv::DMatch>& all_matches, 
    Eigen::Matrix4f& transformation4f, vector<bool>& code_src, vector<bool>& code_tar, 
    std::vector<cv::DMatch>& inliers, double& mean_inlier_error, double dis_threhsold ) const
{
    inliers.clear(); 
    assert(all_matches.size() > 0); 
    const size_t all_matches_size = all_matches.size();

    // 
    inliers.reserve(all_matches_size);
    code_src.reserve(all_matches_size); 
    code_tar.reserve(all_matches_size);

    double mean_error = 0.0;
    Eigen::Matrix4d transformation4d = transformation4f.cast<double>();

    // ofstream dis_of("point_dis.log");

    for(int i=0; i<all_matches_size; i++)
    {
       cv::DMatch& m = all_matches[i];
       bool code_src_i = 0; 
       bool code_tar_i = 0; 
       double min_dis = 1e9; 

       // Eigen::Vector4f& origin = feature_locations_3d_[m.queryIdx]; 
       // Eigen::Vector4f& target = old_node->feature_locations_3d_[m.trainIdx]; 
       // min_dis = errorFunction2(origin, target, transformation4d);

       // 4 combinations : 00, 01, 10, 11
       selectMinDis(old_node, m, transformation4d, 0, 0, code_src_i, code_tar_i, min_dis); 
       // dis_of<<min_dis<<endl;

       if(b_has_been_projected_ && old_node->b_has_been_projected_)
       {
         // if(!b_valid_feature_projec_[m.queryIdx] || !old_node->b_valid_feature_projec_[m.trainIdx])
         {
            // do not use invlid points 
            // min_dis = dis_threhsold + 1;
         }
         // else
         {
           // selectMinDis(old_node, m, transformation4d, 0, 1, code_src_i, code_tar_i, min_dis); 
           // selectMinDis(old_node, m, transformation4d, 1, 0, code_src_i, code_tar_i, min_dis); 
           selectMinDis(old_node, m, transformation4d, 1, 1, code_src_i, code_tar_i, min_dis); 
         }
       } 
      /*
       static bool once = true;
       if(m.queryIdx == 280 && once)
       {
        once = false; 
        ROS_ERROR("queryIdx = %d trainIdx = %d error: %f dis_threshold = %f", m.queryIdx, m.trainIdx, min_dis, dis_threhsold);
        cout<<" v_ori: "<<selectPos(0, m.queryIdx)<<" v_tar: "<<old_node->selectPos(0, m.trainIdx)<<endl;
       }
*/
       // check weather it is a good match 
       if(min_dis > dis_threhsold)
       {
         // ROS_WARN("plane_node.cpp: min_dis: %f, dis_treshold: %f", min_dis, dis_threhsold);
        continue;  // not a good match 
       }
      
       if(min_dis < 0)
       {
        ROS_ERROR("plane_node.cpp: M_dis < 0 : %lf", min_dis); 
        continue;
       }
      // ROS_INFO("plane_node.cpp: find a inlier min_dis: %f, dis_threshold: %f", min_dis, dis_threhsold);
      mean_error += min_dis; 
      inliers.push_back(m);
      code_src.push_back(code_src_i); 
      code_tar.push_back(code_tar_i);
    }
    
    if(inliers.size() < 3)
    {
      mean_inlier_error = 1e9;
    }else
    {
      mean_error /= inliers.size();
      mean_inlier_error = sqrt(mean_error);
    }
  return;
}

bool CPlaneNode::testTrans2(CPlaneNode* p_earlier_node, Eigen::Matrix4f& transformation)
{
  const float max_dist_m = ParameterServer::instance()->get<double>("max_dist_for_inliers");

  bool valid_tf;
  int M = 248;
  int queryIdx, trainIdx; 
  vector<bool> code_src(M, false);   // src pc for selection position of a feature point 
  vector<bool> code_tar(M, false);   // tar pc for selection position of a feature point
  vector<cv::DMatch> inlier;
  inlier.resize(M);
  p_earlier_node->feature_locations_3d_.resize(M); 
  feature_locations_3d_.resize(M); 

  ifstream src_inf("/home/davidz/work/data/SwissRanger4000/dat/src_pts.log"); 
  ifstream tar_inf("/home/davidz/work/data/SwissRanger4000/dat/tar_pts.log"); 
  
  for(int i=0; i<M; i++)
  {
    Eigen::Vector4f src_pt, tar_pt; 
    src_inf>>src_pt(0)>>src_pt(1)>>src_pt(2); src_pt(3) = 1.; 
    tar_inf>>tar_pt(0)>>tar_pt(1)>>tar_pt(2); tar_pt(3) = 1.; 
    inlier[i].queryIdx = inlier[i].trainIdx = i;
    feature_locations_3d_[i] = src_pt; 
    p_earlier_node->feature_locations_3d_[i] = tar_pt;
    if(i<10)
      cout<<tar_pt(0)<<" "<<tar_pt(1)<<" "<<tar_pt(2)<<endl;
  }

  transformation = getTransfromFromMatchesWithProject(p_earlier_node, inlier, code_src, code_tar, valid_tf, max_dist_m);
  // transformation = getTransformFromMatches(this, (Node*)p_earlier_node, inlier,valid_tf,max_dist_m);
  return true;
}

bool CPlaneNode::testTrans(CPlaneNode* p_earlier_node, Eigen::Matrix4f& transformation)
{
  const float max_dist_m = ParameterServer::instance()->get<double>("max_dist_for_inliers");
  ROS_INFO("max_dist_m: %f", max_dist_m);
  // debug, using the outer file containing matched pair 
  ifstream inf("/home/davidz/work/data/SwissRanger4000/dat/inlier_match.log"); 
  bool valid_tf;
  int M = 248;
  int queryIdx, trainIdx; 
  vector<bool> code_src;   // src pc for selection position of a feature point 
  vector<bool> code_tar;   // tar pc for selection position of a feature point
  vector<cv::DMatch> inlier;
  inlier.resize(M);

  for(int i=0; i<M; i++)
  {
    inf>>trainIdx>>queryIdx; 
    inlier[i].trainIdx = trainIdx-1; // this is the where the bug exist
    inlier[i].queryIdx = queryIdx-1;
    
    // ROS_INFO("plane_node.cpp: trainIdx: %f , queryIdx: %f", trainIdx, queryIdx);
  }
  // fake 
  for(int k=0; k<inlier.size(); k++)
  {
    code_src.push_back(false);
    code_tar.push_back(false);
  }
  
  ofstream of1("tar_pts.log"); 
  ofstream of2("src_pts.log"); 
  for(int k=0; k<inlier.size(); k++)
  {
    Eigen::Vector4f& tpt = p_earlier_node->feature_locations_3d_[inlier[k].trainIdx]; 
    Eigen::Vector4f& spt = feature_locations_3d_[inlier[k].queryIdx]; 
    of1<<inlier[k].trainIdx<<" "<<tpt(0)<<" "<<tpt(1)<<" "<<tpt(2)<<endl;
    of2<<spt(0)<<" "<<spt(1)<<" "<<spt(2)<<endl;
  }

  ofstream of3("tar_pts_all.log");
  for(int k=0; k<p_earlier_node->feature_locations_3d_.size(); k++)
  {
    Eigen::Vector4f& tpt = p_earlier_node->feature_locations_3d_[k];
    of3<<tpt(0)<<" "<<tpt(1)<<" "<<tpt(2)<<endl;
  }

  transformation = getTransfromFromMatchesWithProject(p_earlier_node, inlier, code_src, code_tar, valid_tf, max_dist_m);
  // transformation = getTransformFromMatches(this, (Node*)p_earlier_node, inlier,valid_tf,max_dist_m);
  return true;
}

bool CPlaneNode::avaliable_trans(Eigen::Matrix4f& T)
{
  static int max_rotate_angle = ParameterServer::instance()->get<int>("max_rotation_degree"); 
  static double max_rotate_angle_r = D2R(max_rotate_angle);
  static double max_translation = ParameterServer::instance()->get<double>("max_translation_meter"); 
  tf::Transform tfT = eigenTransf2TF(T);  
  tfScalar r, p, y, tx, ty, tz;
  tfT.getBasis().getEulerYPR(y, p, r); 
  tf::Vector3 t = tfT.getOrigin(); 
  tx = fabs(t.getX()); ty = fabs(t.getY()); tz = fabs(t.getZ());
  if(tx >= max_translation || ty >= max_translation || tz >= max_translation)
  {
    return false; 
  }
  if(fabs(y) >= max_rotate_angle_r || fabs(p) >= max_rotate_angle_r || fabs(r) >= max_rotate_angle_r)
  {
    return false;
  }

  return true;
}


// try to improve transformation using EM method 
bool CPlaneNode::EMRefineTrans(const Node* earlier_node, std::vector<cv::DMatch>& inliers, double ini_rmse, 
            Eigen::Matrix4f& ini_T, double & final_rmse, Eigen::Matrix4f& final_T)
{
  bool ret = false;
  
  final_rmse = ini_rmse;
  final_T = ini_T; 
  Eigen::Matrix4f inter_T = ini_T;

  int N_Pair = inliers.size();
  vector<bool> index_zero(N_Pair, false);
  vector< vector<bool> > index_map; 
  index_map.push_back(index_zero);
  const CPlaneNode* old_node = dynamic_cast<const CPlaneNode*>(earlier_node);

  int trainIdx, queryIdx; 
  for(int n_iter=0; n_iter<20; n_iter++)
  {
    vector<bool> index(N_Pair, false);
    pcl::TransformationFromCorrespondences tfc;

    // E_step, find Z given T 
    for(int k = 0; k<N_Pair; k++)
    {
        trainIdx = inliers[k].trainIdx; 
        queryIdx = inliers[k].queryIdx;
        Eigen::Vector4f p1 = feature_locations_3d_[queryIdx]; 
        Eigen::Vector3f p1f = p1.head<3>();
        Eigen::Vector4f q1 = old_node->feature_locations_3d_[trainIdx]; 
        Eigen::Vector3f q1f = q1.head<3>();
        // ROS_ERROR("plane_node.cpp: p1f: %f %f %f  q1f: %f %f %f ", p1f(0), p1f(1), p1f(2), q1f(0), q1f(1), q1f(2));

        if(b_valid_feature_projec_[queryIdx] && old_node->b_valid_feature_projec_[trainIdx]) 
            // figure out weather the extrapolate point is more consistent 
        {
          Eigen::Vector4f p2 = feature_locations_3d_on_plane_[queryIdx]; 
          Eigen::Vector3f p2f = p2.head<3>();
          Eigen::Vector4f q2 = old_node->feature_locations_3d_on_plane_[trainIdx]; 
          Eigen::Vector3f q2f = q2.head<3>();
          //ROS_ERROR("plane_node.cpp: q2f: %f %f %f  p2f: %f %f %f ", q2f(0), q2f(1), q2f(2), p2f(0), p2f(1), p2f(2));

          Eigen::Vector4f delta1 = inter_T * p1; // - q1; // raw point pair
          Eigen::Vector4f delta2 = inter_T * p2; // - q2; // extrapolated point pair
          // ROS_ERROR("plane_node.cpp: p1': %f %f %f %f p2': %f %f %f %f ", delta1(0), delta1(1), delta1(2), delta1(3), 
          //      delta2(0), delta2(1), delta2(2), delta2(3));

          delta1 = delta1 - q1; 
          delta2 = delta2 - q2;

          if(delta1.squaredNorm() > delta2.squaredNorm()) // the extrapolated point pair is better 
          {
            index[k] = true;
            // tfc.add(p2.head<3>(), q2.head<3>(), 1.);
            tfc.add(p2f, q2f, 1.);
            // ROS_WARN("plane_node.cpp: delta1: %f %f %f %f delta2: %f %f %f %f ", delta1(0), delta1(1), delta1(2), delta1(3), 
            //    delta2(0), delta2(1), delta2(2), delta2(3));

            // ROS_WARN("plane_node.cpp: match pair %d raw dis: %lf, > extrapolated dis: %lf", k, delta1.squaredNorm(), delta2.squaredNorm());
          }else
          {
            // ROS_WARN("plane_node.cpp: delta1: %f %f %f %f delta2: %f %f %f %f ", delta1(0), delta1(1), delta1(2), delta1(3), 
            //    delta2(0), delta2(1), delta2(2), delta2(3));
            // ROS_WARN("plane_node.cpp: match pair %d raw dis: %lf, < extrapolated dis: %lf", k, delta1.squaredNorm(), delta2.squaredNorm());
            // tfc.add(p1.head<3>(), q1.head<3>(), 1.);
            tfc.add(p1f, q1f, 1.);
          }
        }else
        {
          // ROS_WARN("plane_node.cpp: match pair %d is not on the plane!", k);
          // tfc.add(p1.head<3>(), q1.head<3>(), 1.);
          tfc.add(p1f, q1f, 1.);
        }   
    } 

    // if allready exist in previous iteration, break
    bool b_existed = false; 
    for(int j = 0; j<index_map.size(); j++)
    {
      if(index_map[j] == index) // exist before 
      {
        // ROS_WARN("plane_node.cpp: repeated case occur when iteration = %d", n_iter);
        b_existed = true;
        break; 
      }
    }
    if(b_existed)
    {
      break;
    }
    index_map.push_back(index);
      
    // M_step, find T given Z 
    inter_T = tfc.getTransformation().matrix();   
    // cout<<"plane_node.cpp: after "<<n_iter<<" iterantion: trans: "<<endl<<inter_T<<endl;

    // figure out weather this transformation is better than the best
    // compute RMSE given new Transformation 
    Eigen::Matrix4d inter_Td = inter_T.cast<double>();
    double tmp_rmse = 0;
    for(int k = 0; k<N_Pair; k++)
    {
      trainIdx = inliers[k].trainIdx; 
      queryIdx = inliers[k].queryIdx;
      
      Eigen::Vector4d p, q;

      if(index[k])  // use extrapolated point pair
      {
        p = feature_locations_3d_on_plane_[queryIdx].cast<double>();
        q = old_node->feature_locations_3d_on_plane_[trainIdx].cast<double>();
      }else         // use raw point pair
      {
        p = feature_locations_3d_[queryIdx].cast<double>();
        q = old_node->feature_locations_3d_[trainIdx].cast<double>();
      }
      
      // Eigen::Vector3d delta = (inter_T * p - q).head<3>();
      Eigen::Vector4d delta = inter_Td * p; // - q).head<3>();
      delta = delta - q;
      tmp_rmse += delta.squaredNorm();
    }
    tmp_rmse = sqrt(tmp_rmse/(double)(N_Pair));
    
    // better than the best? 
    if(tmp_rmse < final_rmse)
    {
      // ROS_WARN("plane_node.cpp: the transformation is updated with pre_rmse %lf, now_rmse %lf", final_rmse, tmp_rmse);
      final_rmse = tmp_rmse;
      final_T = inter_T;
      ret = true;
    }
  }

  // for debug 
  if(ret)
  {
      // ROS_ERROR("plane_node.cpp: refinement, before rmse: %lf, after rmse: %lf ", ini_rmse, final_rmse); 
      // ROS_ERROR("plane_node.cpp: before refinement: ");
      // display_eigen(ini_T); 
      // ROS_ERROR("plane_node.cpp: after refinement: ");
      // display_eigen(final_T);
  }

  return ret;
}

bool CPlaneNode::getRelativeTransformationTo(const Node* target_node, 
        std::vector<cv::DMatch>* initial_matches,
        Eigen::Matrix4f& resulting_transformation, 
        float& rmse,
        std::vector<cv::DMatch>& matches) const  // override the Node' function 
{
  bool ret; 
  static string vro_strategy = ParamSrvMi::instanceMi()->get<std::string>("vro_strategy");
  // ROS_ERROR("plane_node.cpp: vro_strategy %s ", vro_strategy.c_str());
  // if(vro_strategy != "vro_my")
  {
    // intersect_to_planes(); 
    // ((const CPlaneNode*)target_node)->intersect_to_planes();
  }

  if(vro_strategy == "vro_plane" || vro_strategy == "vro_my")
    ret = getRelativeTransformationToNew(target_node, initial_matches, resulting_transformation, rmse, matches);
  else if(vro_strategy == "vro_plane_em")
    ret = getRelativeTransformationEM(target_node, initial_matches, resulting_transformation, rmse, matches); 
  else if(vro_strategy == "vro_ori")
  {
    ret = getRelativeTransformationToNew_ori(target_node, initial_matches, resulting_transformation, rmse, matches);
  }
  else
  {
    ROS_ERROR("plane_node.cpp: no vro_strategy %s matched", vro_strategy.c_str());
    return false;
  }
  return ret;
}

bool CPlaneNode::getRelativeTransformationEM(const Node* earlier_node, 
            std::vector<cv::DMatch>* initial_matches, Eigen::Matrix4f& resulting_transformation,
            float & rmse, std::vector<cv::DMatch>& matches)
{
  if(!b_has_been_projected_ || !((const CPlaneNode*)earlier_node)->b_has_been_projected_)
  {
    return getRelativeTransformationToNew(earlier_node, initial_matches, resulting_transformation, rmse, matches);
  }

  ROS_WARN("plane_node.cpp: in CPlaneNode getRelativeTransformationEM!");
  // VALIDATION 
  assert(initial_matches != NULL); 
  if(initial_matches->size() <= (unsigned int) ParameterServer::instance()->get<int>("min_matches"))
  {
    ROS_INFO("Only %d feature matches between %d and %d (minimal: %i)", (int)initial_matches->size() , this->id_, earlier_node->id_, ParameterServer::instance()->get<int>("min_matches"));
    return false;
  }

  //PREPARATION
  //unsigned int min_inlier_threshold = int(initial_matches->size()*0.2);
  unsigned int min_inlier_threshold = (unsigned int) ParameterServer::instance()->get<int>("min_matches");
  if(min_inlier_threshold > 0.75 * initial_matches->size())
  {
    //FIXME: Evaluate whether beneficial
    ROS_INFO("Lowering min_inlier_threshold from %d to %d, because there are only %d matches to begin with", min_inlier_threshold, (int) (0.75 * initial_matches->size()), (int)initial_matches->size());
    min_inlier_threshold = 0.75 * initial_matches->size();
  }
  
  double inlier_error; //all squared errors
  srand((long)std::clock());
  
  // a point is an inlier if it's no more than max_dist_m m from its partner apart
  const float max_dist_m = ParameterServer::instance()->get<double>("max_dist_for_inliers");
  int ransac_iterations = ParameterServer::instance()->get<int>("ransac_iterations");

  bool b_fixed_ransac_iteration = ParamSrvMi::instanceMi()->get<bool>("b_ransac_fixed_iter");
  int N = initial_matches->size();
  double one_over_indices = 1./(double)(N);
  double probability_ = 0.99;
  double log_probability  = log (1.0 - probability_);

  // initialize result values of all iterations 
  matches.clear();
  resulting_transformation = Eigen::Matrix4f::Identity();
  rmse = 1e6;
  unsigned int valid_iterations = 0;//, best_inlier_cnt = 0;
  const unsigned int sample_size = 4;// chose this many randomly from the correspondences:
  bool valid_tf = false; // valid is false iff the sampled points clearly aren't inliers themself 
  
  std::vector<cv::DMatch>* matches_with_depth = initial_matches; 
  std::sort(matches_with_depth->begin(), matches_with_depth->end()); //sort by distance, which is the nn_ratio
  
  int real_iterations = 0;
  const CPlaneNode* p_earlier_node = dynamic_cast<const CPlaneNode*>(earlier_node);
  double final_cost = -1;

  int invalid_ransac_numebr = 0;

  for(int n = 0; (n < ransac_iterations && matches_with_depth->size() >= sample_size); n++) 
  {
      std::vector<cv::DMatch> inlier = sample_matches_prefer_by_distance(sample_size, *matches_with_depth); //initialization with random samples 

      real_iterations++;   
      bool valid_refinement = false;
      
      {
        // improve the transformation of the randomly selected samples 
        Eigen::Matrix4f tmp_trans;
        static vector<bool> c_src(sample_size, false);
        static vector<bool> c_tar(sample_size, false);

        // randomly select in a complete space 
        c_src.resize(sample_size, false); 
        c_tar.resize(sample_size, false); 
        for(int si=0; si<sample_size; si++)
        {
          if(b_valid_feature_projec_[inlier[si].queryIdx] && p_earlier_node->b_valid_feature_projec_[inlier[si].trainIdx])
          {
            c_src[si] = c_tar[si] = (rand()%2)?true:false;
          }
        }
        
        tmp_trans = getTransfromFromMatchesWithProject(p_earlier_node, inlier, c_src, c_tar, valid_tf, max_dist_m*2.5);
        if(!valid_tf) 
        {
          ++invalid_ransac_numebr;
          continue;
        }
        
        if(!avaliable_trans(tmp_trans)) // if it's not in the valid movement bound
        {
          ++invalid_ransac_numebr; 
          continue;
        }

        //Initialize Results of refinement
        double refined_error = 1e6;
        std::vector<cv::DMatch> refined_matches; 
        Eigen::Matrix4f refined_transformation = Eigen::Matrix4f::Identity();
        
        vector<bool> code_src;   // src pc for selection position of a feature point 
        vector<bool> code_tar;   // tar pc for selection position of a feature point
        Eigen::Matrix4f transformation = tmp_trans; 
        double refined_cost = -1;
        // compute inliers under random initial transformation
        computeInliersAndErrorWithProj( p_earlier_node, *initial_matches, transformation, 
            code_src, code_tar, inlier, inlier_error, SQ(max_dist_m));
        
        if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m)
        {
          // ROS_ERROR("plane_node.cpp: iteration %d inlier.size() %u, inlier_error: %f", n, inlier.size(), inlier_error);
          continue; //break; 
        }

        // EM refinement 
        double EM_refine_error; 
        Eigen::Matrix4f EM_refine_T;
        bool EM_refined = EMRefineTrans(earlier_node, inlier, inlier_error, transformation, EM_refine_error, EM_refine_T);
        if(EM_refined)
        {
          transformation = EM_refine_T;
          inlier_error = EM_refine_error;
        }
        
        // get the result of the refined transformation and error
        refined_cost = COST(inlier.size(), inlier_error);
        refined_transformation = transformation;
        refined_matches = inlier; 
        refined_error = inlier_error; 

        // whether it is better than the best right now
        if(final_cost == -1 || (refined_cost < final_cost && refined_matches.size() > matches.size()*0.5))
        {
          final_cost = refined_cost; 
          rmse = refined_error; 
          resulting_transformation = refined_transformation; 
          matches.assign(refined_matches.begin(), refined_matches.end());
          valid_refinement = true; 
        }
      }
      if(valid_refinement) // this is a valid iteration 
      {
        valid_iterations ++; 
        if(!b_fixed_ransac_iteration) // update iteration time
        {
          // 2, update iteration number 
          int M = matches.size();
          int m1 = 0; 
          for(int m=0; m<M; m++)
          {
            if(!b_valid_feature_projec_[matches[m].queryIdx]) ++m1;
          }
          
          // double w = (double)M*one_over_indices;
          double w1 = (double)m1*one_over_indices; 
          double w2 = (double)(M-m1)*one_over_indices; 
          
          // double p_at_least_one_outlier = 1. - pow(w, (double)(sample_size)); 
          double p_at_least_one_outlier = 1. - pow(w1 + 0.5*w2, (double)(sample_size)); 

          p_at_least_one_outlier = (std::max) (std::numeric_limits<double>::epsilon (), 
              p_at_least_one_outlier);       // Avoid division by -Inf
          p_at_least_one_outlier = (std::min) (1.0 - std::numeric_limits<double>::epsilon (),
              p_at_least_one_outlier);   // Avoid division by 0.
          int tmp = log_probability / log(p_at_least_one_outlier); 
          if(tmp < ransac_iterations) ransac_iterations = tmp;
          // ROS_WARN("plane_node.cpp: iteration at %d, max_iteration = %d", n, ransac_iterations);

        }else
        {
          //Performance hacks:
          if (matches.size() > initial_matches->size()*0.5) n+=10;///Iterations with more than half of the initial_matches inlying, count tenfold
          if (matches.size() > initial_matches->size()*0.75) n+=20;///Iterations with more than 3/4 of the initial_matches inlying, count twentyfold
          if (matches.size() > initial_matches->size()*0.9) break; ///Can this get better anyhow?
          // if (matches.size() > initial_matches->size()*0.8) break; ///Can this get better anyhow?
          // ROS_ERROR("plane_node.cpp: valid iteration: %d, n is: %d", valid_iterations, n);
        }
      }
  }
  ROS_INFO("%i good iterations (from %i), inlier pct %i, inlier cnt: %i, error (MHD): %.2f",valid_iterations, ransac_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size(),rmse);
  
  bool enough_absolute = (matches.size() >= min_inlier_threshold);
  return enough_absolute;
}

bool CPlaneNode::getRelativeTransformationToNew(const Node* earlier_node, 
            std::vector<cv::DMatch>* initial_matches, Eigen::Matrix4f& resulting_transformation,
            float & rmse, std::vector<cv::DMatch>& matches) const 
{
  ROS_WARN("plane_node.cpp: in CPlaneNode getRelativeTransformationToNew!");
  // VALIDATION 
  assert(initial_matches != NULL); 
  
  if(initial_matches->size() <= (unsigned int) ParameterServer::instance()->get<int>("min_matches"))
  {
    ROS_INFO("Only %d feature matches between %d and %d (minimal: %i)", (int)initial_matches->size() , this->id_, earlier_node->id_, ParameterServer::instance()->get<int>("min_matches"));
    return false;
  }
  
  //PREPARATION
  //unsigned int min_inlier_threshold = int(initial_matches->size()*0.2);
  unsigned int min_inlier_threshold = (unsigned int) ParameterServer::instance()->get<int>("min_matches");
  if(min_inlier_threshold > 0.75 * initial_matches->size())
  {
    //FIXME: Evaluate whether beneficial
    ROS_INFO("Lowering min_inlier_threshold from %d to %d, because there are only %d matches to begin with", min_inlier_threshold, (int) (0.75 * initial_matches->size()), (int)initial_matches->size());
    min_inlier_threshold = 0.75 * initial_matches->size();
  }
  
  double inlier_error; //all squared errors
  srand((long)std::clock());
  
   // a point is an inlier if it's no more than max_dist_m m from its partner apart
  const float max_dist_m = ParameterServer::instance()->get<double>("max_dist_for_inliers");
  int ransac_iterations = ParameterServer::instance()->get<int>("ransac_iterations");

  bool b_fixed_ransac_iteration = ParamSrvMi::instanceMi()->get<bool>("b_ransac_fixed_iter");
  int N = initial_matches->size();
  double one_over_indices = 1./(double)(N);
  double probability_ = 0.99;
  double log_probability  = log (1.0 - probability_);

  // ROS_ERROR("plane_node.cpp: max_dist_m: %f ransac_iterations: %d", max_dist_m, ransac_iterations);

  // initialize result values of all iterations 
  matches.clear();
  resulting_transformation = Eigen::Matrix4f::Identity();
  rmse = 1e6;
  unsigned int valid_iterations = 0;//, best_inlier_cnt = 0;
  const unsigned int sample_size = 4;// chose this many randomly from the correspondences:
  bool valid_tf = false; // valid is false iff the sampled points clearly aren't inliers themself 
  
  std::vector<cv::DMatch>* matches_with_depth = initial_matches; 
  std::sort(matches_with_depth->begin(), matches_with_depth->end()); //sort by distance, which is the nn_ratio
  
  int real_iterations = 0;
  const CPlaneNode* p_earlier_node = dynamic_cast<const CPlaneNode*>(earlier_node);
  double final_cost = -1;

  int invalid_ransac_numebr = 0;

  // compute the inlier ratio match
  /*int invalid_initial_matches = 0; 
  for(int i=0; i<matches_with_depth->size(); i++)
  {
    if(!b_valid_feature_projec_[(*matches_with_depth)[i].queryIdx] || 
        !p_earlier_node->b_valid_feature_projec_[(*matches_with_depth)[i].trainIdx])
      ++invalid_initial_matches;
  }*/

  for(int n = 0; (n < ransac_iterations && matches_with_depth->size() >= sample_size); n++) //Without the minimum number of matches, the transformation can not be computed as usual TODO: implement monocular motion est
  {
    // ROS_INFO("plane_node.cpp: iteration %d process:", n);
    std::vector<cv::DMatch> inlier = sample_matches_prefer_by_distance(sample_size, *matches_with_depth); //initialization with random samples 
    /*
    bool b_next_try = false;
    for(int l=0; l<inlier.size(); l++)
    {
      if(!b_valid_feature_projec_[inlier[l].queryIdx] || !p_earlier_node->b_valid_feature_projec_[inlier[l].trainIdx])
      {
        // do not ransac with invalid points
        ++invalid_ransac_numebr;
        b_next_try = true;
        break;
      }
    }
    if(b_next_try) continue;
  */

  // std::vector<cv::DMatch> inlier = sample_matches(sample_size, *matches_with_depth); //initialization with random samples 
  /*std::vector<cv::DMatch> inlier;
  // for debug
  {
    int samples[5][4] = {{171, 17, 247, 124}, {186, 166, 174, 96}, {18, 154, 49, 105}, {44, 18, 35, 187}, {83, 163, 121, 150}}; 
    if(n>=5) inlier = sample_matches_prefer_by_distance(sample_size, *matches_with_depth); //initialization with random samples 
    else
    {
      std::vector<cv::DMatch>& mm = *initial_matches;
      inlier.push_back(mm[samples[n][0]-1]);
      inlier.push_back(mm[samples[n][1]-1]);
      inlier.push_back(mm[samples[n][2]-1]);
      inlier.push_back(mm[samples[n][3]-1]);
    }
  }*/
  
    real_iterations++;
    
    bool valid_refinement = false;

    // all the possible initilized transformations, refine them 
    // std_vector_of_eigen_matrix4f transformation_set = getTransfromFromMatchesCombination(earlier_node, inlier, valid_tf, max_dist_m); 
      
    std_vector_of_eigen_matrix4f transformation_set;
    // transformation_set = getTransfromFromMatchesCombination(earlier_node, inlier, valid_tf, max_dist_m);   

    // Eigen::Matrix4f tmp_trans = getTransformFromMatches(this, earlier_node, inlier,valid_tf,max_dist_m);
    // Eigen::Matrix4f tmp_trans = getTransfromFromMatchesWithProject(p_earlier_node, inlier, c_src, c_tar, valid_tf, max_dist_m);

    {
      Eigen::Matrix4f tmp_trans;
      {
        static vector<bool> c_src(sample_size, false);
        static vector<bool> c_tar(sample_size, false);
        
        // randomly select in a complete space 
        c_src.resize(sample_size, false); 
        c_tar.resize(sample_size, false); 
        for(int si=0; si<sample_size; si++)
        {
          if(b_valid_feature_projec_[inlier[si].queryIdx] && p_earlier_node->b_valid_feature_projec_[inlier[si].trainIdx])
          {
            c_src[si] = c_tar[si] = (rand()%2)?true:false;
          }
        }

        tmp_trans = getTransfromFromMatchesWithProject(p_earlier_node, inlier, c_src, c_tar, valid_tf, max_dist_m*2.5);
        // tmp_trans = getTransfromFromMatchesWithProject(p_earlier_node, inlier, c_src, c_tar, valid_tf, 0);
        if(!valid_tf) 
        {
          ++invalid_ransac_numebr;
          continue;
        }
        
        if(!avaliable_trans(tmp_trans)) // if it's not in the valid movement bound
        {
          ++invalid_ransac_numebr; 
          continue;
        }

        //  for(int l=0; l<sample_size; l++)
        //  {
        //   int t_i = inlier[l].trainIdx;
        //   int s_i = inlier[l].queryIdx;
        //   Eigen::Vector4f& t_pt = p_earlier_node->feature_locations_3d_[t_i]; 
        //   Eigen::Vector4f& s_pt = feature_locations_3d_[s_i];
        //   ROS_INFO("%d tar_pt_id %d %f %f %f, src_pt_id %d %f %f %f", l, t_i, t_pt(0), t_pt(1), t_pt(2), s_i, s_pt(0), s_pt(1), s_pt(2));
        // }
        // ROS_ERROR("plane_node.cpp: initial guess transformation:");
        // display_eigen(tmp_trans);
      }
      transformation_set.push_back(tmp_trans);
    }
  

    // for debug
    /*
    if(valid_iterations == 0)
    {
      Eigen::Matrix4f tmp_trans; 
     transformation_set.clear(); 
     tf::Transform tmptf = getTranRPYt(D2R(0.12), D2R(-0.968), D2R(0.399), -0.002, -0.01, 0.0434);
     pcl_ros::transformAsMatrix(tmptf, tmp_trans);
     transformation_set.push_back(tmp_trans);
    }*/

    // ROS_WARN("plane_node.cpp: transformation_set.size() = %d", transformation_set.size());
     for(int i=0; i<transformation_set.size(); i++)
    {
      //Initialize Results of refinement
      double refined_error = 1e6;
      std::vector<cv::DMatch> refined_matches; 
      Eigen::Matrix4f refined_transformation = Eigen::Matrix4f::Identity();

      vector<bool> code_src;   // src pc for selection position of a feature point 
      vector<bool> code_tar;   // tar pc for selection position of a feature point
      Eigen::Matrix4f transformation = transformation_set[i]; 
      double refined_cost = -1;
      
      // ROS_INFO("transformation_set[%d] is: ", i);
      // display_eigen(transformation);

      // compute inliers under random initial transformation
      computeInliersAndErrorWithProj( p_earlier_node, *initial_matches, transformation, 
          code_src, code_tar, inlier, inlier_error, SQ(max_dist_m));
 
      if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m)
      {
          // ROS_ERROR("plane_node.cpp: iteration %d inlier.size() %u, inlier_error: %f", n, inlier.size(), inlier_error);
          continue; //break; 
      }

      // ROS_INFO("plane_node.cpp: inlier.size() = %u  inlier_error = %f", inlier.size(), inlier_error);
      for(int refinements = 1; refinements < 20; refinements++)
      {
        if(transformation != transformation)   // contain NaN 
          break; 
        // computeInliersAndErrorWithProj( p_earlier_node, *initial_matches, transformation, 
        //  code_src, code_tar, inlier, inlier_error, SQ(max_dist_m));
        /*
        if(valid_iterations == 0 && refinements == 1)
        {
          cout <<" query 280 pt: "<<this->feature_locations_2d_[280].pt<<" train 259 pt: "<<p_earlier_node->feature_locations_2d_[259].pt<<endl;
          ofstream ouf("inlier_match.log"); 
          // cout << "refinements: "<<refinements<<" transformation : "<<transformation<<endl;
          for(int i=0; i<inlier.size(); i++)
          {
            cv::DMatch& m = inlier[i]; 
            ouf << m.queryIdx<<"\t"<<m.trainIdx<<"\t"<<endl;// this->feature_locations_3d_[m.queryIdx]<<"\t"<<p_earlier_node->feature_locations_3d_[m.trainIdx]<<endl;
          }
        }
      */
        transformation = getTransfromFromMatchesWithProject(p_earlier_node, inlier, code_src, code_tar, valid_tf, 0);
        /*test which features are inliers 
        computeInliersAndError(*initial_matches, transformation, 
            this->feature_locations_3d_, //this->feature_depth_stats_, 
            earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
            refined_matches.size(), //break if no chance to reach this amount of inliers
            inlier, inlier_error, max_dist_m*max_dist_m); 
        
        // fake 
        for(int k=0; k<inlier.size(); k++)
        {
          code_src.push_back(false);
          code_tar.push_back(false);
        }
        */

        // hopeless case 
        if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m)
        {
          // ROS_ERROR("plane_node.cpp: inlier.size() %u, min_inlier_threshold %d inlier_error: %f max_dist_m: %f", inlier.size(), min_inlier_threshold, inlier_error, max_dist_m);
          break; 
        }

        // superior to before   
        /*if ( (inlier.size() >= refined_matches.size() && inlier_error <= refined_error) || 
            ( (inlier.size() >= matches.size() && inlier_error <= rmse) && 
            !(refined_matches.size() >= matches.size() && refined_error <= rmse)))*/ 
        if(refined_cost == -1 || (COST(inlier.size(), inlier_error) < refined_cost &&  inlier.size() > refined_matches.size()*0.5))
        {
          refined_cost = COST(inlier.size(), inlier_error);
          assert(inlier_error >= 0); 
          refined_transformation = transformation; 
          refined_matches = inlier; 
          refined_error = inlier_error; 
          // ROS_WARN("plane_node.cpp: in loop %d match is refined at %d with inlier.size: %d, inlier_error: %f", n , refinements, inlier.size(), inlier_error);
          // ROS_ERROR("plane_node.cpp: refined transformation!");
          // display_eigen(transformation);

          // under new transformation, compute the inliers
         size_t prev_num_inliers = refined_matches.size(); 
         computeInliersAndErrorWithProj( p_earlier_node, *initial_matches, transformation, 
            code_src, code_tar, inlier, inlier_error, SQ(max_dist_m));

          if(inlier.size() == prev_num_inliers) break; // no more features are added 
        }
        else 
          break;
        
        // transformation = getTransfromFromMatchesWithProject(p_earlier_node, inlier, code_src, code_tar, valid_tf, max_dist_m);
        // transformation = getTransformFromMatches(this, earlier_node, inlier,valid_tf,max_dist_m);

        if(!valid_tf) 
        {
          ROS_ERROR("plane_node.cpp: should not arrive here!"); 
          break;
        }

      } // END REFINEMENTS 
      
      // Successful Iteration? 
      if(refined_matches.size() > 0) // Valid ?
      {
        //Acceptable && superior to previous iterations?
        //if (refined_error <= rmse &&  
        //    refined_matches.size() >= matches.size() && 
        //    refined_matches.size() >= min_inlier_threshold)
        if(final_cost == -1 || ( COST(refined_matches.size(), refined_error) < final_cost && refined_matches.size() > matches.size()*0.5 ))
        {
          // ROS_DEBUG("%s: Improvment in iteration %d: inliers: %i (min %i), inlier_error: %.2f (max %.2f)",nodesstring.c_str(), real_iterations, (int)refined_matches.size(), (int) min_inlier_threshold,  refined_error, max_dist_m);
          // ROS_ERROR("plane_node.cpp: cost comparison, previous %f now: %f", final_cost,  COST(refined_matches.size(), refined_error));
          final_cost = COST(refined_matches.size(), refined_error);
          rmse = refined_error;
          resulting_transformation = refined_transformation;
          matches.assign(refined_matches.begin(), refined_matches.end());
          // ROS_WARN("plane_node.cpp: valid match found at loop %d with matches.size: %d, rmse: %f", n, matches.size(), rmse);

          valid_refinement = true; // this a valid iteration 
          
        }
      }
    }
    if(valid_refinement)
    {
      valid_iterations++;

      if(!b_fixed_ransac_iteration) // update iteration time
      {
          // 2, update iteration number 
          int M = matches.size();
          int m1 = 0; 
          for(int m=0; m<M; m++)
          {
            if(!b_valid_feature_projec_[matches[m].queryIdx]) ++m1;
          }
          
          // double w = (double)M*one_over_indices;
          double w1 = (double)m1*one_over_indices; 
          double w2 = (double)(M-m1)*one_over_indices; 
        
        // double w = (double)M*one_over_indices;
        // double p_at_least_one_outlier = 1. - pow(w, (double)(sample_size)); 
        double p_at_least_one_outlier = 1. - pow(w1 + 0.5*w2, (double)(sample_size)); 
        p_at_least_one_outlier = (std::max) (std::numeric_limits<double>::epsilon (), 
            p_at_least_one_outlier);       // Avoid division by -Inf
        p_at_least_one_outlier = (std::min) (1.0 - std::numeric_limits<double>::epsilon (),
            p_at_least_one_outlier);   // Avoid division by 0.
        int tmp = log_probability / log(p_at_least_one_outlier); 
        if(tmp < ransac_iterations) ransac_iterations = tmp;
        // ROS_INFO("plane_node.cpp: iteration at %d, max_iteration = %d", n, ransac_iterations);

      }else
      {
        //Performance hacks:
        if (matches.size() > initial_matches->size()*0.5) n+=10;///Iterations with more than half of the initial_matches inlying, count tenfold
        if (matches.size() > initial_matches->size()*0.75) n+=20;///Iterations with more than 3/4 of the initial_matches inlying, count twentyfold
        if (matches.size() > initial_matches->size()*0.9) break; ///Can this get better anyhow?
        // if (matches.size() > initial_matches->size()*0.8) break; ///Can this get better anyhow?
        // ROS_INFO("plane_node.cpp: valid iteration: %d, n is: %d", valid_iterations, n);
      }
    }
  }

#ifdef USE_PCL_ICP
    // for debug 
    if(ParamSrvMi::instanceMi()->get<bool>("use_icp_refinement_find_inlier"))
    {
      ROS_INFO("plane_node.cpp: hello in icp_refinement");
      Eigen::Matrix4f icp_trafo = resulting_transformation; 
      int max_count = ParameterServer::instance()->get<int>("gicp_max_cloud_size"); 
      pointcloud_type::Ptr tmp1(new pointcloud_type());
      pointcloud_type::Ptr tmp2(new pointcloud_type());
      filterCloud(*pc_col, *tmp1, max_count);
      filterCloud(*(earlier_node->pc_col), *tmp2, max_count);
      resulting_transformation = icpAlignment(tmp2, tmp1, icp_trafo);
     
      int inlier = 0; 
      // get inliers and outliers 
      for(int i=0; i<matches.size(); i++)
      {
        cv::DMatch& m = matches[i]; 
        Eigen::Vector4f origin = feature_locations_3d_[m.queryIdx]; 
        Eigen::Vector4f target = earlier_node->feature_locations_3d_[m.trainIdx]; 
        Eigen::Vector3f p = target.head<3>(); 
        Eigen::Vector3f q = (icp_trafo * origin).head<3>(); 
        Eigen::Vector3f e = p -q; 
        if(e.norm() <= 0.05)
        {
          ++inlier;
        }
      }
      static ofstream inlier_inf("./rgbd_icp_inliers.log");
      inlier_inf<<id_<<" "<<inlier<<" "<<matches.size()-inlier<<endl;
      
    }

#endif
  
    // calculate outlier and inliers given ground truth transformation 
  if(ParamSrvMi::instanceMi()->get<bool>("compute_inlier_outlier_gt"))
  {
      if(!gt_has_been_set_ || !(((const CNodeWrapper*)earlier_node)->gt_has_been_set_))
      {
        // this should never happen 
        ROS_ERROR("plane_node.cpp: what? node %d or earlier node %d gt not been set ", id_, earlier_node->id_);
      }

      static Eigen::Matrix4f vo_gT = Eigen::Matrix4f::Identity();

      // tf::Transform compare gt and vo 
        tf::Transform gt_t = ((const CNodeWrapper*)earlier_node)->gt_T_.inverse()*gt_T_; 
        Eigen::Matrix4f eigen_gt;
        pcl_ros::transformAsMatrix(gt_t, eigen_gt);
        
        tf::Transform vo_t = eigenTransf2TF(resulting_transformation); 
        resulting_transformation = eigen_gt;
        // tf::Transform vo_gTT = eigenTransf2TF(vo_gT); 

        // printTransform("plane_node.cpp: groundtruth: ", gt_t); 
        // printTransform("plane_node.cpp: rgbdmatch: ", );
        /* tf::Transform t = gt_t; 
        ROS_INFO_STREAM("gt_t: " << ": Translation " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z());
        ROS_INFO_STREAM("gt_t: " << ": Rotation " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW());

         t = ((const CNodeWrapper*)earlier_node)->gt_T_;
         ROS_INFO_STREAM("earlier_node_t: " << ": Translation " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z());
         ROS_INFO_STREAM("earlier_node_t: " << ": Rotation " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW());

         // t = gt_T_; 
         t = vo_gTT;
         ROS_INFO_STREAM("vo_node_t: " << ": Translation " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z());
         ROS_INFO_STREAM("vo_node_t: " << ": Rotation " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW());

        t = vo_t; 
        ROS_INFO_STREAM("vo_t: " << ": Translation " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z());
        ROS_INFO_STREAM("vo_t: " << ": Rotation " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW());
        */
      int inlier = 0; 
      // get inliers and outliers 
      for(int i=0; i<matches.size(); i++)
      {
        cv::DMatch& m = matches[i]; 
        Eigen::Vector4f origin = feature_locations_3d_[m.queryIdx]; 
        Eigen::Vector4f target = earlier_node->feature_locations_3d_[m.trainIdx]; 
        Eigen::Vector3f p = target.head<3>(); 
        
          // compute inliers and outliers 
        Eigen::Vector3f q = (eigen_gt * origin).head<3>(); 
        // Eigen::Vector3f q = (resulting_transformation * origin).head<3>(); 

        Eigen::Vector3f e = p -q; 
        if(e.norm() <= 0.05)
        {
          ++inlier;
        }
      }
      static ofstream inlier_inf("./rgbd_slam/rgbd_gt_inliers.log");
      inlier_inf<<id_<<" "<<inlier<<" "<<matches.size()-inlier<<" "<<earlier_node->header_.stamp.toSec()<<" "<< header_.stamp.toSec() <<endl;
      ROS_WARN("plane_node.cpp: inlier: %d outlier: %d", inlier, matches.size()-inlier);
      // update vo_gT 
      vo_gT = vo_gT * resulting_transformation; 
  }

  ROS_INFO("%i good iterations (from %i), inlier pct %i, inlier cnt: %i, error (MHD): %.2f",valid_iterations, ransac_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size(),rmse);

 // ROS_ERROR("plane_node.cpp: invalid ransac number %d", invalid_ransac_numebr);

  // ROS_ERROR("plane_node.cpp: invalid ransac number %d initial inlier number: %d, ratio: %f", invalid_ransac_numebr, invalid_initial_matches, (float)invalid_initial_matches/(float)matches_with_depth->size());

  bool enough_absolute = (matches.size() >= min_inlier_threshold);
  
  /*
  // for strategy plane based EM 
  static string vro_strategy = ParamSrvMi::instanceMi()->get<std::string>("vro_strategy");
  if(vro_strategy == string("vo_plane_em") && enough_absolute)
  {
    intersect_to_planes(); 
    ((CPlaneNode*)earlier_node)->intersect_to_planes();
    double final_rmse;
    Eigen::Matrix4f final_T;
    bool refined = EMRefineTrans(earlier_node, matches, rmse, resulting_transformation, final_rmse, final_T); 
    if(refined)
    {
      // ROS_ERROR("plane_node.cpp: refined transformation!");
      rmse = final_rmse; 
      resulting_transformation = final_T;
    }
  }*/

  return enough_absolute;
}


bool CPlaneNode::getRelativeTransformationToNew_ori(const Node* earlier_node, 
            std::vector<cv::DMatch>* initial_matches, Eigen::Matrix4f& resulting_transformation,
            float & rmse, std::vector<cv::DMatch>& matches) const
{
  ROS_WARN("plane_node.cpp: in CPlaneNode getRelativeTransformationToNew_ori!");
  // VALIDATION 
  assert(initial_matches != NULL); 
  
  if(initial_matches->size() <= (unsigned int) ParameterServer::instance()->get<int>("min_matches"))
  {
    ROS_INFO("Only %d feature matches between %d and %d (minimal: %i)", (int)initial_matches->size() , this->id_, earlier_node->id_, ParameterServer::instance()->get<int>("min_matches"));
    return false;
  }
  
  //PREPARATION
  //unsigned int min_inlier_threshold = int(initial_matches->size()*0.2);
  unsigned int min_inlier_threshold = (unsigned int) ParameterServer::instance()->get<int>("min_matches");
  if(min_inlier_threshold > 0.75 * initial_matches->size())
  {
    //FIXME: Evaluate whether beneficial
    ROS_INFO("Lowering min_inlier_threshold from %d to %d, because there are only %d matches to begin with", min_inlier_threshold, (int) (0.75 * initial_matches->size()), (int)initial_matches->size());
    min_inlier_threshold = 0.75 * initial_matches->size();
  }
  
  double inlier_error; //all squared errors
  srand((long)std::clock());
  
   // a point is an inlier if it's no more than max_dist_m m from its partner apart
  const float max_dist_m = ParameterServer::instance()->get<double>("max_dist_for_inliers");
  const int ransac_iterations = ParameterServer::instance()->get<int>("ransac_iterations");
 
  // initialize result values of all iterations 
  matches.clear();
  resulting_transformation = Eigen::Matrix4f::Identity();
  rmse = 1e6;
  unsigned int valid_iterations = 0;//, best_inlier_cnt = 0;
  const unsigned int sample_size = 4;// chose this many randomly from the correspondences:
  bool valid_tf = false; // valid is false iff the sampled points clearly aren't inliers themself 
  
  std::vector<cv::DMatch>* matches_with_depth = initial_matches; 
  std::sort(matches_with_depth->begin(), matches_with_depth->end()); //sort by distance, which is the nn_ratio
  
  int real_iterations = 0;
  for(int n = 0; (n < ransac_iterations && matches_with_depth->size() >= sample_size); n++) //Without the minimum number of matches, the transformation can not be computed as usual TODO: implement monocular motion est
  {
    //Initialize Results of refinement
    double refined_error = 1e6;
    std::vector<cv::DMatch> refined_matches; 
    std::vector<cv::DMatch> inlier = sample_matches_prefer_by_distance(sample_size, *matches_with_depth); //initialization with random samples 
     //std::vector<cv::DMatch> inlier = sample_matches(sample_size, *matches_with_depth); //initialization with random samples 
    Eigen::Matrix4f refined_transformation = Eigen::Matrix4f::Identity();

    real_iterations++;
    for(int refinements = 1; refinements < 20 ; refinements++) 
    {
      Eigen::Matrix4f transformation = getTransformFromMatches(this, earlier_node, inlier,valid_tf,max_dist_m);
      //Eigen::Matrix4f transformation = getTransformFromMatchesUmeyama(this, earlier_node, inlier,valid_tf);
      if (!valid_tf || transformation!=transformation)  //Trafo Contains NaN?
        break; // valid_tf is false iff the sampled points aren't inliers themself 

      //test which features are inliers 
      computeInliersAndError(*initial_matches, transformation, 
          this->feature_locations_3d_, //this->feature_depth_stats_, 
          earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
          refined_matches.size(), //break if no chance to reach this amount of inliers
          inlier, inlier_error, max_dist_m*max_dist_m); 

      if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
        ROS_DEBUG("Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
        break; //hopeless case
      }   //superior to before?
        if (inlier.size() >= refined_matches.size() && inlier_error <= refined_error) {
          size_t prev_num_inliers = refined_matches.size();
          assert(inlier_error>=0);
          refined_transformation = transformation;
          refined_matches = inlier;
          refined_error = inlier_error;
          if(inlier.size() == prev_num_inliers) break; //only error improved -> no change would happen next iteration
        }
        else break;
    }  //END REFINEMENTS
        //Successful Iteration?
    if(refined_matches.size() > 0){ //Valid?
        valid_iterations++;
        ROS_DEBUG("Valid iteration: inliers/matches: %lu/%lu (min %u), refined error: %.2f (max %.2f), global error: %.2f", 
                refined_matches.size(), matches.size(), min_inlier_threshold,  refined_error, max_dist_m, rmse);

        //Acceptable && superior to previous iterations?
        if (refined_error <= rmse &&  
            refined_matches.size() >= matches.size() && 
            refined_matches.size() >= min_inlier_threshold)
        {
          // ROS_DEBUG("%s: Improvment in iteration %d: inliers: %i (min %i), inlier_error: %.2f (max %.2f)",nodesstring.c_str(), real_iterations, (int)refined_matches.size(), (int) min_inlier_threshold,  refined_error, max_dist_m);
          ROS_INFO("%s: Improvment in iteration %d: inliers: %i (min %i), inlier_error: %.2f (max %.2f)",__FILE__, real_iterations, (int)refined_matches.size(), (int) min_inlier_threshold,  refined_error, max_dist_m);
          rmse = refined_error;
          resulting_transformation = refined_transformation;
          matches.assign(refined_matches.begin(), refined_matches.end());
          //Performance hacks:
          if (refined_matches.size() > initial_matches->size()*0.5) n+=10;///Iterations with more than half of the initial_matches inlying, count tenfold
          if (refined_matches.size() > initial_matches->size()*0.75) n+=10;///Iterations with more than 3/4 of the initial_matches inlying, count twentyfold
          if (refined_matches.size() > initial_matches->size()*0.8) break; ///Can this get better anyhow?
        }
    }
  } //iterations
  ROS_INFO("%i good iterations (from %i), inlier pct %i, inlier cnt: %i, error (MHD): %.2f",valid_iterations, ransac_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size(),rmse);

  bool enough_absolute = matches.size() >= min_inlier_threshold;
  return enough_absolute;
}
