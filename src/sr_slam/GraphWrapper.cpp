#include "GraphWrapper.h"
#include "NodeWrapper.h"
#include "Submap.h"

// #include "ros/ros.h"
//feature message
#include "std_msgs/Float32MultiArray.h"
#include <pcl_ros/point_cloud.h>


CGraphWrapper::CGraphWrapper():
   m_pSubmap(new CSubmap<CNodeWrapper>),
   m_bHasOptimized(false)
{
    optimizer_->setVerbose(false);
    m_first_pose.setIdentity();

    ros::NodeHandle nh;

    //mi parameter server
    ParamSrvMi* mips = ParamSrvMi::instanceMi();

    //whether to build the globalmap online using ros topics
    if(mips->get<bool>("globalmap_online_ros"))
    {
    	//rgbdslam parameter server
    	ParameterServer* ps = ParameterServer::instance();
    	submap_feature_loc_pub_ = nh.advertise<std_msgs::Float32MultiArray>(mips->get<string>("submap_feature_loc_out_topic"), ps->get<int>("publisher_queue_size"));
    	submap_feature_des_pub_ = nh.advertise<std_msgs::Float32MultiArray>(mips->get<string>("submap_feature_des_out_topic"), ps->get<int>("publisher_queue_size"));
    	submap_cloud_pub_ = nh.advertise<pointcloud_type>(mips->get<string>("submap_cloud_out_topic"), ps->get<int>("publisher_queue_size"));
    }


}

CGraphWrapper::~CGraphWrapper()
{
    if(m_pSubmap != 0) delete m_pSubmap;
}

void CGraphWrapper::addFirstNode(Node* new_node, tf::Transform p)
{
    //set the node id only if the node is actually added to the graph
    //needs to be done here as the graph size can change inside this function
    new_node->id_ = graph_.size();
    // ROS_ERROR("GraphWrapper.cpp addFirstNode id: %d", new_node->id_);
    new_node->seq_id_ = next_seq_id++; // allways incremented, even if node is not added
    init_base_pose_ =  new_node->getGroundTruthTransform();//identity if no MoCap available
    printTransform("Ground Truth Transform for First Node", init_base_pose_);
    //new_node->buildFlannIndex(); // create index so that next nodes can use it
    
    m_first_pose = p;
    submap::Pose6d pp(p);
    ROS_ERROR_STREAM("GraphWrapper.cpp set the first node with pose: "<<pp);
    g2o::SE3Quat pose = tf2G2O(p);
    g2o::VertexSE3 ori_identity;
    g2o::VertexSE3* reference_pose = new g2o::VertexSE3;
    reference_pose->setEstimate(ori_identity.estimateAsSE3Quat() * pose);

    new_node->vertex_id_ = next_vertex_id++;
    graph_[new_node->id_] = new_node;
    reference_pose->setId(new_node->vertex_id_);

    camera_vertices.insert(reference_pose);

    // ROS_ERROR("Adding initial node with id %i and seq %i, v_id: %i", new_node->id_, new_node->seq_id_, new_node->vertex_id_);
    g2o::SE3Quat g2o_ref_se3 = tf2G2O(init_base_pose_);
    // reference_pose->setEstimate(g2o_ref_se3);
    reference_pose->setFixed(true);//fix at origin
    optimizer_mutex_.lock();
    optimizer_->addVertex(reference_pose); 
    optimizer_mutex_.unlock();

    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(new_node->vertex_id_));
    tf::Transform tf_trans = eigenTransf2TF(v->estimate());
    submap::Pose6d ppp(tf_trans);
    ROS_ERROR_STREAM("GraphWrapper.cpp to check firstPose: "<<ppp);

    // QString message;
    // Q_EMIT setGUIInfo(message.sprintf("Added first node with %i keypoints to the graph", (int)new_node->feature_locations_2d_.size()));
    //pointcloud_type::Ptr the_pc(new_node->pc_col); //this would delete the cloud after the_pc gets out of scope
    QMatrix4x4 latest_transform = g2o2QMatrix(g2o_ref_se3);
    /*if(!ParameterServer::instance()->get<bool>("glwidget_without_clouds")) { 
        Q_EMIT setPointCloud(new_node->pc_col.get(), latest_transform);
        Q_EMIT setFeatures(&(new_node->feature_locations_3d_));
    }*/
    current_poses_.append(latest_transform);
    this->addKeyframe(new_node->id_);
    process_node_runs_ = false;
}

tf::Transform CGraphWrapper::getLastNodePose()
{
    if(!m_bHasOptimized)
    {
        optimizeGraph();
        m_bHasOptimized = true;
    }
    optimizer_mutex_.lock();
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[graph_.size()-1]->vertex_id_));
    tf::Transform previous = eigenTransf2TF(v->estimate());
    optimizer_mutex_.unlock();
    return previous;
}

void CGraphWrapper::reduceGraph(const ClientNet* m_client)
{
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    ROS_ERROR("start to reduce Graph!");
    if(!m_bHasOptimized)
    {
        optimizeGraph();
        m_bHasOptimized = true;
    }
    ROS_ERROR("GraphWrapper.cpp start to saveGraph!");
    m_pSubmap->saveGraph(this);
    ROS_ERROR("GraphWrapper.cpp after saveGraph!");
    ROS_ERROR("GraphWrapper.cpp start to reduction!");
    m_pSubmap->reduction();
    ROS_ERROR("GraphWrapper.cpp after reduction!");

    ParamSrvMi* mips = ParamSrvMi::instanceMi();

    if(mips->get<bool>("submap_dump"))
    {
    	m_pSubmap->dump2File();
    }
    if(mips->get<bool>("globalmap_online_ros"))
    {
    	this->publishGraphROS();
    }
    if(mips->get<bool>("globalmap_online_socket"))
    {
    	this->publishGraphSocket(m_client);
    }

    // string file("./submaps/prefile.log");
    // m_pSubmap->print(file);
    m_pSubmap->resetSubmap();
    clock_gettime(CLOCK_MONOTONIC, &end);
    double time_elapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    ROS_ERROR("finish reducing Graph, cost: %lf s", time_elapsed);
    resetGraph();
    optimizer_->setVerbose(false);
}

void CGraphWrapper::publishGraphSocket(const ClientNet* m_client)
{
	ROS_ERROR("start to publish graph socket");

	// publish pcd
	int len_pcd = 0;
	ParamSrvMi* mips = ParamSrvMi::instanceMi();
	if(mips->get<bool>("submap_downsample"))
	{
		pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
		voxel_grid.setInputCloud (m_pSubmap->pc_col);
		float leaf_size = ParameterServer::instance()->get<double>("octomap_resolution");
		voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
		voxel_grid.filter (*downsampled);
		m_pSubmap->pc_col = downsampled;
	}
	len_pcd = sizeof(pcl::PointXYZRGB)*m_pSubmap->pc_col->points.size();

	//total size

	int len_tf = sizeof(m_pSubmap->m_root);
	int len_loc = m_pSubmap->m_feature_locs.size()*sizeof(Eigen::Vector4f);
	int len_des = m_pSubmap->m_feature_des.rows * m_pSubmap->m_feature_des.cols * sizeof(float);
	int len = len_tf + len_loc  + len_des + len_pcd;
	char buffer[len];

	//copy tf
	memcpy(buffer, &len_tf, 4);
	memcpy(&buffer[4], &m_pSubmap->m_root, len_tf);

	//copy feature loc
	memcpy(&buffer[4+len_tf], &len_loc, 4);
	memcpy(&buffer[8+len_tf],  &m_pSubmap->m_feature_locs[0], len_loc);
	//copy feature des
	memcpy(&buffer[8+len_tf+len_loc], &len_des, 4);
	memcpy(&buffer[12+len_tf+len_loc], m_pSubmap->m_feature_des.data, len_des);
	//copy point cloud
	memcpy(&buffer[12+len_tf+len_loc+len_des], &len_pcd, 4);
	memcpy(&buffer[16+len_tf+len_loc+len_des], &m_pSubmap->pc_col->points[0], len_pcd);
	//send data to server

	m_client->SendData(buffer,len);
	//char a[]="what ever\n";
	//m_client.SendData(a,sizeof(a));

	ROS_ERROR("finish to publish graph socket %d, %d, %d, %d, %d, %d, %d", m_pSubmap->m_feature_des.rows, m_pSubmap->m_feature_locs.size(),len_tf, len_loc, len_des, len_pcd,len);

}

void CGraphWrapper::publishGraphROS()
{

	ROS_ERROR("PUBLISH GRAPH BEGIN");
	ros::Time cur_t = ros::Time::now();

	//publish tf info
    std::string cam_frame = "/submap/cam";
    std::string base_frame  = "/submap/world";
	tf::StampedTransform submap_tf(m_pSubmap->m_root, cur_t, base_frame, cam_frame);
	submap_br_.sendTransform(submap_tf);

	//publish feature loc
	int num_f = m_pSubmap->m_feature_locs.size();
	std_msgs::Float32MultiArray feature_loc;

	std_msgs::MultiArrayDimension arr_info;
	{
		arr_info.label = "height";
		arr_info.size = num_f;
		feature_loc.layout.dim.push_back(arr_info);

		arr_info.label = "width";
		arr_info.size = 3;
		feature_loc.layout.dim.push_back(arr_info);

		arr_info.label = "channel";
		arr_info.size = 1;
		feature_loc.layout.dim.push_back(arr_info);
	}

	feature_loc.data.clear();
	for(int i=0;i<num_f;i++)
	{
		Eigen::Vector4f& fea_loc = m_pSubmap->m_feature_locs[i];
		feature_loc.data.push_back(fea_loc(0));
		feature_loc.data.push_back(fea_loc(1));
		feature_loc.data.push_back(fea_loc(2));

	}

	submap_feature_loc_pub_.publish(feature_loc);

	ROS_ERROR("PUBLISHED FEATURE LOC %d", num_f);

	//publish feature des

	std_msgs::Float32MultiArray feature_des;

	{
		arr_info.label = "height";
		arr_info.size = m_pSubmap->m_feature_des.rows;
		feature_des.layout.dim.push_back(arr_info);

		arr_info.label = "width";
		arr_info.size = m_pSubmap->m_feature_des.cols;
		feature_des.layout.dim.push_back(arr_info);

		arr_info.label = "channel";
		arr_info.size = 1;
		feature_des.layout.dim.push_back(arr_info);
	}


	for(int i=0;i<num_f;i++)
	{
		for(int j=0;j<m_pSubmap->m_feature_des.cols;j++)
		{
			float des_v = m_pSubmap->m_feature_des.at<float>(i,j);
			feature_des.data.push_back(des_v);
		}
	}

	submap_feature_des_pub_.publish(feature_des);

	ROS_ERROR("PUBLISHED FEATURE DES %d, %d", m_pSubmap->m_feature_des.rows, m_pSubmap->m_feature_des.cols);

	//publish point cloud

	ros::Time stamp = m_pSubmap->pc_col->header.stamp; //temp storage
	m_pSubmap->pc_col->header.stamp = cur_t; //to sync with tf

	ParamSrvMi* mips = ParamSrvMi::instanceMi();
	if(mips->get<bool>("submap_downsample"))
	{
		//publish downsampled
		pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
		voxel_grid.setInputCloud (m_pSubmap->pc_col);
		float leaf_size = ParameterServer::instance()->get<double>("octomap_resolution");//gl_octomap_voxel_size;
		voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
		voxel_grid.filter (*downsampled);
		//pcl::io::savePCDFile(ss.str(), *downsampled);
		submap_cloud_pub_.publish(*downsampled);
	}
	else{
		//publish ori cloud
		submap_cloud_pub_.publish(m_pSubmap->pc_col);
	}


	m_pSubmap->pc_col->header.stamp = stamp; //to sync with tf

	ROS_ERROR("PUBLISHED CLOUD %d", m_pSubmap->pc_col->points.size());

}

void CGraphWrapper::submapSwap()
{
    static bool once = true;
    if(once && graph_.size() > ParamSrvMi::instanceMi()->get<int>("submap_size"))//gl_submap_size)
    {
        ROS_WARN("Submap begin!");
        ROS_INFO("Start save Graph!");
        m_pSubmap->saveGraph(this);
        ROS_WARN("Finish save Graph!");
        ROS_INFO("Start reduction!");
        m_pSubmap->reduction();
        ROS_WARN("Finish save Graph!");
        ROS_INFO("Start dump2File!");
        m_pSubmap->dump2File();
        ROS_WARN("Finish dump 2File!");
        string file("./submaps/prefile.log");
        m_pSubmap->print(file);
        ROS_WARN("Finish print file");
        m_pSubmap->resetSubmap();
        ROS_INFO("Start resetGraph!");
        resetGraph();
        ROS_WARN("Finish resetGraph!");
        // 1, reset current graph
        // reset_request_ = true; // 
        // 2, send current graph info

        once = false;
    }      
}

void CGraphWrapper::resetGraph()
{
    ROS_ERROR("GraphWrapper.cpp in subclass resetGraph()!");
    GraphManager::resetGraph();
    setVerbose(false);
    m_bHasOptimized = false;
}

void CGraphWrapper::resetFirstPose()
{
    submap::Pose6d pp(m_first_pose);
    ROS_ERROR_STREAM("GraphWrapper.cpp reset the first node with pose: "<<pp);
    g2o::SE3Quat pose = tf2G2O(m_first_pose);
    g2o::VertexSE3 ori_identity;
    // reference_pose->setEstimate(ori_identity.estimateAsSE3Quat() * pose);
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[0]->vertex_id_));
    v->setEstimate(ori_identity.estimateAsSE3Quat() * pose);
    // tf::Transform tf_trans = eigenTransf2TF(v->estimate());
}
