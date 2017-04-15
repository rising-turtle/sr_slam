/*
 *  Sep. 29, 2015, David Z 
 *  
 *  a graph work in an offline style, read the camera data from disk, and display it in gui
 *  and the node in graph is plane_node, that use features associated with plane
 *  
 * */

#ifndef GRAPH_PLANE_H
#define GRAPH_PLANE_H

#include "graph_wrapper.h"
#include <QImage> 
#include <string>
#include "gyro_euler.h"

using namespace std;

class COpenniWrapper;
class CSReader;  // read sr4k data
class CRSReader; // read realsense data 
class sr_data;
class rs_data;
class CamModel;

class CGraphPlane : public CGraphWrapper
{
  Q_OBJECT
public:
    CGraphPlane(); 
    virtual ~CGraphPlane();
Q_SIGNALS:
    ///Connect to this signal to get up-to-date optical images from the listener
    void newVisualImage(QImage);
    ///Connect to this signal to get up-to-date featureFlow visualizations from the listener
    void newFeatureFlowImage(QImage);
    ///Connect to this signal to get up-to-date depth images from the listener
    void newDepthImage(QImage);
    //void pauseStatus(bool is_paused);
    ///Set the temporary status-message in the GUI
    void setGUIStatus(QString message);

public Q_SLOTS:
    ///Switch between processing or ignoring new incoming data
    void togglePause();
    ///Process a single incomming frame. Useful in pause-mode for getting one snapshot at a time
    void getOneFrame();
    // void loadPCDFiles(QStringList);

public:
    boost::shared_ptr<CSReader> sr_reader_;
    boost::shared_ptr<CRSReader> rs_reader_;
    void loadSkData(); 
    void loadSkDataAsync();
    void loadRSDataAsync();
    void loadR200RGBDAsync(); 

    void skCallback(sr_data& );
    void rsCallback(rs_data& );
    Node* fromRGBD(cv::Mat& , cv::Mat& , CamModel&);
    Node* fromSR(sr_data& sr);  // from sr4k to node 
    Node* fromRS(rs_data& rs);  // from realsense to node
    void processNode(Node*);    // normal process to handle a node  
    void processNodeForDisplay(Node*);  // only display image, feature and point cloud 
    void processNodeForWrite(Node*);   

    // try to rewrite the nodeComparisons
    virtual bool nodeComparisons(Node* newNode, QMatrix4x4& curr_motion_estimate, bool& edge_to_keyframe);

public:
    bool VRO(string path_1, int tar_id, string path_2, int src_id, tf::Transform& trans);
    bool VRO(string path, int tar_id, int src_id, tf::Transform& trans);
    bool VRO(sr_data& tar, sr_data& src, tf::Transform& trans);
    bool VRO(Node* tar, Node* src, tf::Transform& trans);
    int read_data_id_;       // indicate the index of the data file
    int write_data_id_; 
    string path_;       // where to write the nodes
    bool b_write_img_;  // weather to write grey image into disk

public:
    std::string image_encoding_;
    std::vector<cv::Mat> rgba_buffers_;
    QImage cvMat2QImage(const cv::Mat& , unsigned int);
    
    //Variables
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Mat visualization_img_;

public:
    // publish the current node pose 
    bool b_publish_pose_3d_;
    ros::Publisher pose_publisher_; 
    void publish_new_3D_pose(); 

public:
    // test imu_gui display, display imu oriented rotation using a rectangle 
    void imu_gui_display();   // it doesnot work 
    void imuCallback(Node* );       // the process to handle imu data
    
    bool b_use_gyro_imu_;     // whether to use imu data read from gyro 
    CGyroEuler* pGyroReader_; // point to read gyro data 
    ros::Publisher imu_pub_;  // publisher for imu data 
    int imu_camera_syn_id_;   // used to synchronize imu data and camera data 

    bool load_syn_camera_time(string ); // load syn timestamp for camera 
    vector<int> time_step_;   // record camera time 

    int sequence_frame_id_;   // number of the frames in a video sequence 

public:
    bool pause_; 
    bool getOneFrame_; 
};



#endif
