/*
 *  David Z 20 Oct. 2014 
 *  
 *  to view point cloud in the VTK
 *
 * */


#ifndef VTK_VIEWER_H
#define VTK_VIEWER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>

using namespace std;

template<typename PointT>
class CVTKViewer
{
  public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr     CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    CVTKViewer();
    ~CVTKViewer();
    
    void init(); // init viewer size and Viewpoint

    // for interaction
    void keyboard_callback(const pcl::visualization::KeyboardEvent& e, void*);
    void mouse_callback(const pcl::visualization::MouseEvent&, void*);
    void run(); // just show this point cloud
    void runOnce(); // just spinOnce
    bool stopped();
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer>& 
      getViewer(){return cloud_viewer_;}

    bool addSpheres(CloudPtr& in, double r= 0.2, string prefix = "sphere");
    bool addColorSpheres(CloudPtr& in, double r= 0.2, string prefix = "sphere", double red= 1., double g = 1., double b =1.);

    // add point cloud into the viewer 
    bool addPointCloud(CloudPtr& in, string id);
    
    // add point cloud and normals into the viewer 
    bool addPointCloudNormals(CloudPtr& in, string pc_id, pcl::PointCloud<pcl::Normal>::Ptr& normals, int level = 40, double scale = 0.05, string normal_id="normals");

    // set the cloud to be dumpped
    void setDumpCloud(CloudPtr& in);
    
    // save the clusters point 
    void setClusters(vector<CloudPtr>& );
    int saveClusters();

    // configures for the viewer
    void setViewerPosition(int x, int y){cloud_viewer_->setPosition(x,y);}
    void setSize(int w, int h){cloud_viewer_->setSize(w,h);}
    void resetCameraViewpoint(const char* id){cloud_viewer_->resetCameraViewpoint(id);}
    void setCameraPosition(float px, float py, float pz, float vx, float vy, float vz, float ux, float uy, float uz );
  public:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    CloudPtr cloud_to_dump_;
    vector<CloudPtr> cloud_clusters_;
    string cluster_save_prefix_; // cluster_save_prefix_
    int cluster_id_;
};

#include "vtk_viewer.hpp"

#endif
