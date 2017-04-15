/*
 * paramSrvMi.cpp
 *
 *  Created on: Sep 24, 2013
 *      Author: liu
 */


#include "paramSrvMi.h"

ParamSrvMi* ParamSrvMi::_instanceMi = NULL;

void ParamSrvMi::configMi()
{
        /*
	addOption("submap_size",         			static_cast<int> (50),                      "the size of the submap");
	addOption("submap_overlap_size",         	static_cast<int> (10),                      "the size of the overlap part between submaps");
	addOption("submap_min_matches",         	static_cast<int> (100),                   	"minimum matched features between submaps");

	addOption("submap_downsample",     			static_cast<bool> (true),                   "wether to downsample the pcd, if yes, the octomap_resolution will be used");
	addOption("submap_dump",         			static_cast<bool> (true),                   "wether to save submaps");
	addOption("submap_saved_path",         		std::string("./submaps"),                   "the place to save submaps");

	addOption("globalmap_online_ros",         	static_cast<bool> (false),                  "whether to use ros to build global map online");
	addOption("submap_cloud_out_topic",    		std::string("/submap/cloud"),     			"topic to publish the submap cloud");
	addOption("submap_feature_des_out_topic",    		std::string("/submap/feature_des"),     			"topic to publish the submap cloud");
	addOption("submap_feature_loc_out_topic",    		std::string("/submap/feature_loc"),     			"topic to publish the submap cloud");

	addOption("globalmap_online_socket",        static_cast<bool> (false),                  "whether to use socket to build global map online");
	addOption("socket_host_ip",    				std::string("127.0.0.1"), 			    			"host ip");
	addOption("socket_host_port",    			static_cast<int>(8787), 			    			"host port");

	addOption("socket_client_ip",    			std::string("127.0.0.1"), 			    			"client ip");
	addOption("socket_client_port",    			static_cast<int>(6767), 			    			"client port");

    addOption("submap_number",                    static_cast<int>(10),                               "total submaps num");
    addOption("submap_step",                   static_cast<int>(1),                                "submap steps");
    addOption("submap_first",                    static_cast<int>(0),                               "the first submap id");
    */
    addOption("topic_swiss_ranger",       std::string("sr_array"),            "swiss ranger msg array ");
    addOption("imu_roll",                 static_cast<double>(0),             "initial imu roll");
    addOption("imu_pitch",                static_cast<double>(0),             "initial imu pitch");
    addOption("mapping_rule",             static_cast<int>(1),                "mapping rule from 3D to 2D");
    addOption("min_distance_thre",        static_cast<float>(4),              "minimal distance to delete points too near");
    addOption("send_pc2_vis",             static_cast<bool>(false),           "weather send point cloud to 2D navigation vis");
    addOption("send_2_vis",               static_cast<bool>(false),           "weather send node info to 2D navigation vis");
    addOption("save_added_node",          static_cast<bool>(false),           "weather save every added node into disk"); 
    addOption("save_node_path",           std::string("./"),                  "where to save added node");
    addOption("save_node_image",          static_cast<bool>(false),           "weather to save node' image");
    addOption("process_node_method",      std::string("slam"),                "how to handle node: slam, display, write");
    addOption("read_node_from_disk",      static_cast<bool>(false),           "whether to read node from disk");
    addOption("vro_strategy",             std::string("vro_ori"),              "VRO strategy: VRO_ori, VRO_my, VRO_plane, VRO_EM");
    addOption("b_ransac_fixed_iter",      static_cast<bool>(false),           "RANSAC using fixed number of iteration time or not");
    addOption("run_data",                 std::string("sr4k"),                "which type of data is used: swissranger, realsense");
    addOption("b_add_floor_into_graph",   static_cast<bool>(false),           "whether to add floor into graph");
    addOption("g2o_file",                 std::string(""),                    "where to find g2o file");
    addOption("use_gaussian_filter",      static_cast<bool>(false),           "whether to use gaussian filter for intensity img");
    addOption("load_imu_data",            static_cast<bool>(false),           "whether to read imu data");
    addOption("imu_file_dir",             std::string(""),                    "where to find the imu file");
    addOption("camera_syn_data",          std::string(""),                    "time interval for camera data");
    addOption("b_test_imu_vro",           static_cast<bool>(false),           "whether to comapre imu and vro");
    addOption("b_front_obstacle_detect",  static_cast<bool>(false),           "whether to detect obstacles in the front");
    addOption("obstacle_min_z",           static_cast<float>(-0.5),           "obstacle detection min z threhold");
    addOption("obstacle_len_y",           static_cast<float>(0.2),            "obstacle detection len y threhold");
    addOption("obstacle_max_x",           static_cast<float>(3),              "obstacle detection max x threhold");
    addOption("obstacle_density_threshold",   static_cast<float>(0.02),       "obstacle detection point cloud density");
    addOption("obstacle_number_pt_threshold", static_cast<int>(1000),         "obstacle detection number of point cloud");
    addOption("use_icp_refinement_find_inlier", static_cast<bool>(false),     "whether use icp refinement to check inliers");
    addOption("compute_inlier_outlier_gt", static_cast<bool>(false),          "whether use ground truth to check inliers");
    addOption("gt_file",                  std::string(""),                    "where to find ground truth file");

}

ParamSrvMi::ParamSrvMi(){

	  pre = ros::this_node::getName();
	  pre += "/config/";

	  configMi();
	  getValues();

}
ParamSrvMi::~ParamSrvMi(){
}


ParamSrvMi* ParamSrvMi::instanceMi()
{
	if(_instanceMi == NULL)
	{
		_instanceMi = new ParamSrvMi();
	}

	return _instanceMi;
}



void ParamSrvMi::addOption(std::string name, boost::any value, std::string description){
    config[name] = value;
    descriptions[name] = description;
}

/* Used by GUI */
std::string ParamSrvMi::getDescription(std::string param_name) {
  return descriptions[param_name];
}

void ParamSrvMi::getValues() {
  std::map<std::string, boost::any>::const_iterator itr;
  for (itr = config.begin(); itr != config.end(); ++itr) {
    std::string name = itr->first;
    if (itr->second.type() == typeid(std::string)) {
      config[name] = getFromParameterServer<std::string> (pre + name,
          boost::any_cast<std::string>(itr->second));
      ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<std::string>(itr->second));
    } else if (itr->second.type() == typeid(int)) {
      config[name] = getFromParameterServer<int> (pre + name,
          boost::any_cast<int>(itr->second));
      ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<int>(itr->second));
    } else if (itr->second.type() == typeid(double)) {
      config[name] = getFromParameterServer<double> (pre + name,
          boost::any_cast<double>(itr->second));
      ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<double>(itr->second));
    } else if (itr->second.type() == typeid(bool)) {
      config[name] = getFromParameterServer<bool> (pre + name,
          boost::any_cast<bool>(itr->second));
      ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<bool>(itr->second));
    }
  }
}
