#ifndef GRAPH_WRAPPER_H
#define GRAPH_WRAPPER_H

#include "graph_manager.h"
#include "NodeWrapper.h"
#include "ClientNet.h"
#include "tf/tf.h"
#include <map>

template<typename NODE>
class CSubmap;

class CNodeWrapper;

typedef CSubmap<CNodeWrapper> submap_type;

using namespace std;

class CGraphWrapper : public GraphManager
{
    Q_OBJECT
public:
    CGraphWrapper();
    ~CGraphWrapper();
    
    typedef map<int, Node*>::iterator giter;
    giter begin(){return graph_.begin();}
    giter end(){return graph_.end();}

    size_t getLandmarksNum(){ return landmarks.size();}
    tf::Transform getLastNodePose();
    typedef vector<Landmark>::iterator liter;
    liter lbegin(){return landmarks.begin();}
    liter lend(){return landmarks.end();}

    CSubmap<CNodeWrapper> * m_pSubmap;
    unsigned int getFeatureLength()
    {
        if(graph_.size() == 0) return 0;
        return graph_.begin()->second->feature_descriptors_.cols;
    }
    int getSeq(){return next_seq_id;}
    void setSeq(int seq) {next_seq_id=seq;}
    unsigned int size(){return graph_.size();}
    g2o::SparseOptimizer* getOptimizer(){return optimizer_;}
    bool IsVerbose(){return optimizer_->verbose();}
    void submapSwap();
    void reduceGraph(const ClientNet* );
    void publishGraphROS();
    void publishGraphSocket(const ClientNet* );
    void resetGraph();
    void resetFirstPose();
    void setVerbose(bool b) {optimizer_->setVerbose(b);} 
    void setDraw(bool d) {m_bDrawThisGraph = d;}
    void resetViewer(){Q_EMIT resetGLViewer();}
    bool IsDraw(){return m_bDrawThisGraph;}
    void addFirstNode(Node*, tf::Transform);
    bool m_bHasOptimized;
    tf::Transform m_first_pose; // to keep the pose of the first node


protected:
    //ros::Publisher submap_pub_;
    ros::Publisher submap_feature_loc_pub_;
    ros::Publisher submap_feature_des_pub_;
    ros::Publisher submap_cloud_pub_;
    tf::TransformBroadcaster submap_br_;

public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW



};

#endif
