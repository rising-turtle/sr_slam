/*
 * David Z, Apr 20, 2015 
 * wrapper for node in rgbdslam, to handle swiss ranger interface 
 * and extend the content in a node
 *
 * */

#ifndef NODE_WRAPPER_H
#define NODE_WRAPPER_H

#include "parameter_server.h"
// #include "paramSrvMi.h"
#include "node.h"
#include "matching_result.h"
#include <vector>
#include <string>

using namespace std;

typedef pcl::PointXYZ sr_point_type; 
typedef pcl::PointCloud<sr_point_type> sr_cloud_type; 
typedef typename sr_cloud_type::Ptr sr_cloudPtr; 

// parameters for sr4k 
static const int sr_rows = 144; 
static const int sr_cols = 176;
static const int sr_n = sr_rows*sr_cols;

// parameters for realsense 
static const int rs_rows = 480; 
static const int rs_cols = 640;
static const int rs_n = rs_rows*rs_cols;

class CamModel;

class CNodeWrapper : public Node
{
public:
      using Node::id_;
      using Node::pc_col;
      // this is the node type just receiving the depth data 
      CNodeWrapper(const cv::Mat& visual, 
          const cv::Mat& depth, 
          const cv::Mat& detection_mask, 
          std_msgs::Header depth_header, 
          cv::Ptr<cv::FeatureDetector> detector, 
          cv::Ptr<cv::DescriptorExtractor> extractor
          );
      
      // this is the node type just receiving the XYZ data 
      CNodeWrapper(const cv::Mat& visual, 
          const cv::Mat& depth, 
          const cv::Mat& detection_mask, 
          std_msgs::Header depth_header, 
          std::vector<float>& px, std::vector<float>& py, std::vector<float>& pz, 
          cv::Ptr<cv::FeatureDetector> detector, 
          cv::Ptr<cv::DescriptorExtractor> extractor
          );

    	CNodeWrapper(const cv::Mat& visual,
			 const cv::Mat& depth,
			 const cv::Mat& detection_mask,
                         const CamModel& cam_info, 
                         std_msgs::Header depth_header,
			 cv::Ptr<cv::FeatureDetector> detector,
			 cv::Ptr<cv::DescriptorExtractor> extractor);

      // these two below are to be consistent with the original rgbdslam process 

 	CNodeWrapper(const cv::Mat& visual,
			 const cv::Mat& depth,
			 const cv::Mat& detection_mask,
                         const sensor_msgs::CameraInfoConstPtr& cam_info, 
                         std_msgs::Header depth_header,
			 cv::Ptr<cv::FeatureDetector> detector,
			 cv::Ptr<cv::DescriptorExtractor> extractor);

        CNodeWrapper(const cv::Mat visual,
			 cv::Ptr<cv::FeatureDetector> detector,
			 cv::Ptr<cv::DescriptorExtractor> extractor,
			 pointcloud_type::Ptr point_cloud,
                         const cv::Mat detection_mask = cv::Mat());
        
        // construct from a stream containing the sr_array
        // CNodeWrapper(unsigned char* pS);

        CNodeWrapper(); // needed for submap_node
        CNodeWrapper(string path, int id);  // read node from disk
        virtual ~CNodeWrapper();
        CNodeWrapper(const CNodeWrapper&);
        CNodeWrapper& operator=(const CNodeWrapper&);
        void Init(std_msgs::Header&);
        void featureInit(const cv::Mat& visual, 
           const cv::Mat& depth,
           const cv::Mat& detection_mask,
           std_msgs::Header depth_header,
           sr_cloudPtr& tmp_cloud,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor);

        // construct a point cloud using SR distance info
        bool sr_calSRCloud(sr_cloudPtr&, const cv::Mat&);
        
        // set ground truth pose 
        void setGTPose(float*);
        tf::Transform gt_T_;
        bool gt_has_been_set_;  // whether ground truth pose has been set 

        bool sr_projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
            std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d, 
            sr_cloudPtr& );
        void sr_projectTo3DSiftGPU(std::vector<cv::KeyPoint>& feature_locations_2d,
            std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
            sr_cloudPtr& point_cloud, 
            std::vector<float>& descriptors_in, cv::Mat& descriptors_out);

        // store point cloud 
        void storeSRPointCloud(sr_cloudPtr&, const cv::Mat&);

        // rotation to the initial coordinate reference 
        float pitch_; 
        void setPitch(float p);
        float getPitch();

        // virtual MatchingResult matchNodePair(const Node* );
                
        // Data IO 
        bool write(std::string ); 
        bool read(std::string, int );

        int d_N_; // number of the points, sr_n or rs_n 
        int d_rows_;  // rows 
        int d_cols_;  // cols
public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
