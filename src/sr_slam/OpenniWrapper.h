/*
 * David Z, Apr 20, 2015
 * wrapper for the openni_listenner in rgbdslam, 
 * to handle the swiss ranger msg, and construct the wrapped node type
 *
 * */

#ifndef OPENNI_WRAPPER_H
#define OPENNI_WRAPPER_H
#include "openni_listener.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

// class CDuoGraph;
class CIMUReader;

class COpenniWrapper : public OpenNIListener 
{
    Q_OBJECT
public:
    COpenniWrapper(GraphManager* g_mgr);
    // COpenniWrapper(CDuoGraph* );
    virtual ~COpenniWrapper();
Q_SIGNALS:
    // void sendNode2Graph(void*);
public:
    //! For this callback the point cloud is not required. 
    void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                          const sensor_msgs::ImageConstPtr& depth_img_msg,
                          const sensor_msgs::CameraInfoConstPtr& cam_info_msg) ;

    //! as cameraCallback, but create Node without cloud
    void noCloudCameraCallback(cv::Mat visual_img, 
                               cv::Mat depth, 
                               cv::Mat depth_mono8_img,
                               std_msgs::Header depth_header,
                               const sensor_msgs::CameraInfoConstPtr& cam_info);

    //!Call processNode either regularly or as background thread
    void callProcessing(cv::Mat gray_img, Node* node_ptr);
    // void processNode(Node* node_ptr);
    // CDuoGraph * m_duo_mgr;

public:
    void preprocessImg(cv::Mat& in_img, cv::Mat& out_img, bool use_filter = false);
    void convert16UC_8UC(cv::Mat& in_img, cv::Mat& out_img);

    // IMU data 
    boost::shared_ptr<CIMUReader> p_imu_;
    bool setInit2Base(Node*);  // set initial transformation from init to the base or the first frame 

public:
    ros::Publisher sr_ack_pub_; // publisher to the ack process
    message_filters::Subscriber<std_msgs::Bool> *sr_syn_sub_; // subscriber to the syn process
    void srSynCb(const std_msgs::Bool::ConstPtr&);
    message_filters::Subscriber<std_msgs::UInt8MultiArray> * sr_array_sub_; // subscriber to the swiss ranger msg (array)
    void srCallback(const std_msgs::UInt8MultiArray::ConstPtr& sr_array_ptr);
    ros::Publisher slamLost_pub;
    
    // publishers for draw the robot and trajectory 
    // ros::Publisher r_pos_pub_;  // publish robot' current position, 
};


#endif
