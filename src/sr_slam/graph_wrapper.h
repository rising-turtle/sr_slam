/*
 * May 28, 2015, David Z 
 * 
 * Subclass for graph, transmit slam data to UI
 *
 * */

#ifndef GRAPH_WRAPPER_H
#define GRAPH_WRAPPER_H

#include "graph_manager.h"
#include <message_filters/subscriber.h>
#include "std_msgs/Float32MultiArray.h"
#include <map>

using namespace std;

class CGraphWrapper : public GraphManager
{
  Q_OBJECT 
public:
    CGraphWrapper(); 
    virtual ~CGraphWrapper();
    virtual bool addNode(Node* newNode); 
    
    virtual bool nodeComparisons(Node* newNode, 
                         QMatrix4x4& curr_motion_estimate,
                         bool& edge_to_keyframe);///Output:contains the best-yet of the pairwise motion estimates for the current node
  
    // call back of save g2o graph 
    // ros::Subscriber graph_save_sub_;  // save graph structure
    
    int graph_size();       // return the number of nodes in the graph

   // publishers for draw the robot and trajectory 
    void publish2D(Node* new_node);
    // void pubRobotPos(Node* newNode);
    ros::Publisher r_pos_pub_;  // publish robot' current position,

    void robotPos2DCallBack(const std_msgs::Float32MultiArray::ConstPtr& r_pos2d_ptr);
    message_filters::Subscriber<std_msgs::Float32MultiArray> *r_pos2D_sub_; // subscriber robot' adjusted 2D position 
    map<int, vector<float> > pos2D_pool_;   // store feedback pos

    tf::Transform T_g2o_; // global to base, camera position can be transformed into global coordinate system by T_g2c_ = T_g2o * T_o2c 
    float floor_level_; // TODO: delete it? 
    int mapping_rule_select_ ; // depends on the start orientation 
    bool addFloorPlane(Node* new_node);
    bool isFloor(void* p); // check whether this is a floor plane in global coordinate 
    double normAngle(double angle, double base); 
    
    // below parameters are used for adding plane vertexes in graph
    int floor_node_id_  ; 
    int offset_sensor_id_;  

    bool firstNode(Node* new_node); // for the first node, handle with T_g2o_, compute T_g2b and T_b2o, T_o2c is the result of VO 

    void publishG2O();  // publish global g2o vertex position 
    
    // detect whether obstacles appear in the front 
    bool getGlobalPose(Node* new_node, Eigen::Isometry3d& Trans); 
    bool getGlobalPose(Node* new_node, Eigen::Isometry3d& Trans, tfScalar& yaw); 
    bool detectObstacleInFront(Node* new_node, bool floor_detected = false);
    ros::Publisher cmd_pub; // notices the client when obstacles appear 
    float z_min_threshold_; // bounding box for min_z
    float y_len_threshold_; // bounding box for [-y_len, y_len]
    float x_max_threshold_; // bounding box for max_x
    float g_euclidean_threshold_;  // density threshold 
    int   g_number_pt_threshold_;  // number of points for the obstacle 

    // getLastNodePose 
    tf::Transform getLastNodePose();

public Q_SLOTS:
    // override these functions in graph manager 
    void saveG2OGraph(QString filename); 
};

#endif 
