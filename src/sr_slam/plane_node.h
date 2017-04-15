/*
 *  Sep 29, 2015, David Z 
 *  
 *  this node, able to detect plane, read sr4k data from disk, overload the ransac process
 *
 * */

#ifndef PLANE_NODE_H
#define PLANE_NODE_H

#include "NodeWrapper.h"
#include "../sr_plane/point_xyz.h"
// #include "../sr_plane/plane.h"

template <typename T >
extern tf::Transform eigenTransf2TF(const T& transf);
 
typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > std_vector_of_eigen_vector4f;
typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > std_vector_of_eigen_vector3f;
typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > std_vector_of_eigen_matrix4f;

class CPlane;
class CPlaneSet;
class CamModel;

class CPlaneNode : public CNodeWrapper
{
  public:
    CPlaneNode(const cv::Mat& visual, 
          const cv::Mat& depth, 
          const cv::Mat& detection_mask, 
          std_msgs::Header depth_header, 
          cv::Ptr<cv::FeatureDetector> detector, 
          cv::Ptr<cv::DescriptorExtractor> extractor);

    CPlaneNode(const cv::Mat& visual, 
          const cv::Mat& depth, 
          const cv::Mat& detection_mask, 
          const CamModel& cam_info, 
          std_msgs::Header depth_header, 
          cv::Ptr<cv::FeatureDetector> detector, 
          cv::Ptr<cv::DescriptorExtractor> extractor);

   CPlaneNode(const cv::Mat& visual, 
          const cv::Mat& depth, 
          const cv::Mat& detection_mask, 
          std_msgs::Header depth_header, 
          std::vector<float>& px, std::vector<float>& py, std::vector<float>& pz, 
          cv::Ptr<cv::FeatureDetector> detector, 
          cv::Ptr<cv::DescriptorExtractor> extractor
          );

    CPlaneNode(string path, int id);
    virtual ~CPlaneNode();
    
    void init_node(); // some initilization work

    //Compute the relative transformation between the nodes
    virtual bool getRelativeTransformationTo(const Node* target_node, 
        std::vector<cv::DMatch>* initial_matches,
        Eigen::Matrix4f& resulting_transformation, 
        float& rmse,
        std::vector<cv::DMatch>& matches) const;  // override the Node' function 

    virtual bool getRelativeTransformationToNew(const Node* earlier_node, 
            std::vector<cv::DMatch>* initial_matches, Eigen::Matrix4f& resulting_transformation,
            float & rmse, std::vector<cv::DMatch>& matches) const ;
    virtual bool getRelativeTransformationToNew_ori(const Node* earlier_node, 
            std::vector<cv::DMatch>* initial_matches, Eigen::Matrix4f& resulting_transformation,
            float & rmse, std::vector<cv::DMatch>& matches) const;

    // virtual MatchingResult matchNodePair(const Node* older_node);
    MatchingResult VRO(const Node* earlier_node);
    bool VRO( const Node* earlier_node, Eigen::Matrix4f& ) ;
    bool VRO_ori(const Node* earlier_node, Eigen::Matrix4f&) ;
  
  public:
    bool b_has_been_projected_; // weather the feature points has been projected onto the plane
    void points2Plane(CPlaneSet* pset, vector<int>& index); //  index indicates for each point belong to which plane
    void points2Plane(CPlane* p, vector<int>& index); //  index indicates for each point weather belong to plane p
    void remove_sift_points(boost::shared_ptr<pointcloud_type>&);  // 
    bool intersect_to_planes(); //  compute the intersection between the ray and the planes
    bool intersect_to_plane();  // compute the intersection between the ray and the plane
    bool intersect_to_plane2();  // compute the intersection between the ray and the plane
    bool intersect_to_plane(CPlane* );  // 
    bool intersect_to_floor();  // compute the intersection between the ray and the floor
    void handle_with_plane(int M, VectorPF3& feature_loc, VectorPF3& f_locs, vector<bool>& b_belong_to_plane);

    std_vector_of_eigen_vector4f feature_locations_3d_on_plane_;  // the extension of the ray tracing with the plane 
    vector<bool>  b_valid_feature_projec_;  // weather its intersection with the plane is valid
    std_vector_of_eigen_matrix4f getTransfromFromMatchesCombination(Node* old_node, std::vector<cv::DMatch>& matches, 
        bool& valid, float max_dist_m);       // get possible transformation using different combination

    Eigen::Matrix4f getTransfromFromMatchesWithProject(const CPlaneNode*, std::vector<cv::DMatch>& matches, 
        vector<bool>& code_src, vector<bool>& code_tar, bool& valid, float) const ; 

    bool isValid(Eigen::Vector3f& p); // check the validity of a point
    bool isValid(std_vector_of_eigen_vector3f& pts);  // check the validity of a point vector 
    
    Eigen::Vector3f selectPos(int select_code, int idx) const ;  // select weather a original point or a point on the fitted plane 
    void selectPosGivenCode(Node* old_node, int code, std::vector<cv::DMatch>& matches, std_vector_of_eigen_vector3f& fp, std_vector_of_eigen_vector3f& tp);      // select point set given the matches idx and selection code 

    vector<int> computeConsistentSet(Node* older_node, std::vector<cv::DMatch>& matches, float max_dist_m); // find the combination set satisfying the max_dist_m condition 

    bool selectMinDis(CPlaneNode* old_node, cv::DMatch& m, Eigen::Matrix4d& transformation4d, int code_src, int code_tar, bool& min_code_src, bool& min_code_tar, double& min_dis);  // select the match providing the minimal distance 


    // compute the inliers and distances with projected points
    void computeInliersAndErrorWithProj(const CPlaneNode* old_node, std::vector<cv::DMatch>& all_matches, 
    Eigen::Matrix4f& transformation4f, vector<bool>& code_src, vector<bool>& code_tar, 
    std::vector<cv::DMatch>& inliers, double& mean_inlier_error, double dis_threhsold ) const ; 

    // compute the error function between matched feature points
    double errorFunctionSR4k(Eigen::Vector3f& x1, Eigen::Vector3f& x2, Eigen::Matrix4d& transformation);
    double errorFunctionSR4kNew(Eigen::Vector3f& x1, Eigen::Vector3f& x2, Eigen::Matrix4d& transformation);

    // for test
    bool testTrans(CPlaneNode* p_earlier_node, Eigen::Matrix4f& transformation);
    bool testTrans2(CPlaneNode* p_earlier_node, Eigen::Matrix4f& transformation);

    // filter out some abnormal transformations 
    bool avaliable_trans(Eigen::Matrix4f&);
    
    // downsampling point cloud 
    void filter_pc();
    
    // EM process to improve transformation 
    bool getRelativeTransformationEM(const Node* earlier_node, std::vector<cv::DMatch>* initial_matches, Eigen::Matrix4f& resulting_transformation, float & rmse, std::vector<cv::DMatch>& matches);
    bool EMRefineTrans(const Node* earlier_node, std::vector<cv::DMatch>& inliers, double ini_rmse, Eigen::Matrix4f& ini_T, double & final_rmse, Eigen::Matrix4f& final_T);

  public:
    cv::Mat intensity_img_; // for debug
    cv::Mat getIntensity_img();

  public:
    void diffRPY(CPlaneNode* node, double& r, double& p, double& y);     // euler angle difference between two nodes 
    void setRPY(float* rpy);
    float rpy_[3];  // read imu 
};

#endif
