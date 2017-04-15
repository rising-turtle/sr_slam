/*
 *  David Z 20 Oct. 2014 
 *  
 *  to view point cloud in the VTK
 *
 * */

#include <pcl/io/pcd_io.h>

template<typename PointT>
CVTKViewer<PointT>::CVTKViewer():
cloud_viewer_(new pcl::visualization::PCLVisualizer),
    cloud_to_dump_(new pcl::PointCloud<PointT>),
    cluster_save_prefix_("./clusters")
{
  init();
}

template<typename PointT>
CVTKViewer<PointT>::~CVTKViewer()
{
  // cloud_viewer_->close();
}

template<typename PointT>
void CVTKViewer<PointT>::init()
{
  setViewerPosition(0,0);
  setSize(640, 480);
  resetCameraViewpoint("VTK_Viewer");
  /*setCameraPosition(
      -0.05, -0.13, 1.46,    // Position 
      0,0,-1,    // Viewpoint
      0.096,1,0.09    // Up
      );*/
  /*setCameraPosition(
      0, 0, 3,    // Position 
      0, 0, -1,   // Viewpoint
      0, -1, 0    // Up
      );*/
  /*setCameraPosition(
          0,0,-5,
          0,0,1,
          0,-1,0
          );*/
    setCameraPosition(
            5, -1, 0,
            -1, 0, 0,
            0, -1, 0
            );
    cloud_viewer_->registerMouseCallback(&CVTKViewer::mouse_callback, *this);
    cloud_viewer_->registerKeyboardCallback(&CVTKViewer::keyboard_callback, *this);
    // cloud_viewer_->addCoordinateSystem(0.4, "global");
}

template<typename PointT>
void CVTKViewer<PointT>::setCameraPosition(float px, float py, float pz, float vx, float vy, float vz, float ux, float uy, float uz)
{
  cloud_viewer_->setCameraPosition(
      px, py, pz,
      vx, vy, vz,
      ux, uy, uz
      );
}

template<typename PointT>
void CVTKViewer<PointT>::keyboard_callback(const pcl::visualization::KeyboardEvent &event, void*)
{
 /* if (event.getKeyCode ())
    cout << "the key \'" << event.getKeyCode () << "\' (" << event.getKeyCode () << ") was";
  else
    cout << "the special key \'" << event.getKeySym () << "\' was";*/
  if (event.keyDown ())
  {
      // cout << " pressed" << endl;
      char cmd = event.getKeyCode(); 
      if(cmd == 'd')
      {
          /* if(cloud_to_dump_->points.size() <= 0)
          {
            cout<<"vtk_viewer.hpp: cloud_to_dump_ has no points!"<<endl;
          }else{
              cout<<"vtk_viewer.hpp: dump the pcd file!"<<endl;
              pcl::io::savePCDFile("dump_cloud.pcd", *cloud_to_dump_);
          }*/     
        // save all the clusters
        int ret = saveClusters();
        if(ret > 0)
        {
            cout<<"vtk_viewer.hpp: succeed to save "<<ret<<" clusters!"<<endl;
        }else
        {
            cout<<"vtk_viewer.hpp: failed to save clusters!"<<endl;
        }
      }else if(cmd == 's' || cmd == 'S')
      {
       //  g_stop_capture = !g_stop_capture;
       //  cout<<"vtk_viewer.hpp: "<<(g_stop_capture?"stop_to_capture":"start_to_capture")<<endl;
      }else if(cmd == 't' || cmd == 'T')
      {
       // g_start_record = !g_start_record;
       // cout<<"vtk_viewer.hpp: "<<(g_start_record?"start_to_record_video!":"stop_to_record_video")<<endl;
      }  
  }
  // else
     // cout << " released" << endl;
}

template<typename PointT>
int CVTKViewer<PointT>::saveClusters()
{
    if(cloud_clusters_.size()<=0)
    {
      // no input clusters
        cout<<"vtk_viewer.hpp: failed to save clusters because size <= 0"<<endl;
        return 0;
    }
    
    for(int i=0; i<cloud_clusters_.size(); i++)
    {
        stringstream ss; 
        ss<<cluster_save_prefix_<<"/c_"<<cluster_id_++<<"_U.pcd";
        pcl::io::savePCDFile(ss.str(), *cloud_clusters_[i]);
    }
    return cloud_clusters_.size();
}

template<typename PointT>
void CVTKViewer<PointT>::setDumpCloud(CloudPtr& in)
{
    cloud_to_dump_ = in->makeShared();
}

template<typename PointT>
void CVTKViewer<PointT>::setClusters(vector<CloudPtr>& in_Clusters)
{
    cloud_clusters_.clear();
    for(int i=0; i < in_Clusters.size(); i++ )
    {
        cloud_clusters_.push_back(in_Clusters[i]->makeShared());
    }
}

template<typename PointT>
void CVTKViewer<PointT>::mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void*)
{
  if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton)
  {
    // cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
  } 
}

template<typename PointT>
void CVTKViewer<PointT>::run()
{
  while(!cloud_viewer_->wasStopped() )
  {
    cloud_viewer_->spinOnce(); // thread yield 
  }
}

template<typename PointT>
void CVTKViewer<PointT>::runOnce()
{
  if(!cloud_viewer_->wasStopped())
  {
    cloud_viewer_->spinOnce(); // thread yield
  }
}

template<typename PointT>
bool CVTKViewer<PointT>::stopped()
{
  return cloud_viewer_->wasStopped();
}

template<typename PointT>
bool CVTKViewer<PointT>::addSpheres(CloudPtr& in, double r, string prefix)
{
  static int sphere_num = 0;
  int M = in->points.size();
  for(int i=0; i<M; i++)
  {
    if(std::isnan(in->points[i].x) || std::isnan(in->points[i].y) || std::isnan(in->points[i].z))
      continue;
    stringstream ss; 
    ss<<prefix<<"_"<<++sphere_num;
    cloud_viewer_->addSphere(in->points[i], r, ss.str());
  }
  return true;
}

template<typename PointT>
bool CVTKViewer<PointT>::addColorSpheres(CloudPtr& in, double r, string prefix, double red, double g, double b)
{
  static int sphere_num = 0;
  int M = in->points.size();
  for(int i=0; i<M; i++)
  {
    if(std::isnan(in->points[i].x) || std::isnan(in->points[i].y) || std::isnan(in->points[i].z))
      continue;
    stringstream ss; 
    ss<<prefix<<"_"<<++sphere_num;
    cloud_viewer_->addSphere(in->points[i], r, red, g, b, ss.str());
  }
  return true;
}

template<typename PointT>
bool CVTKViewer<PointT>::addPointCloud(CVTKViewer<PointT>::CloudPtr& in, string id)
{
  if(!cloud_viewer_->updatePointCloud(in, id.c_str()))
  {
    cloud_viewer_->addPointCloud(in, id.c_str());
  }
  return true;
}

template<typename PointT>
bool CVTKViewer<PointT>::addPointCloudNormals(CloudPtr& in, string pc_id, pcl::PointCloud<pcl::Normal>::Ptr& normals, int level, double scale, string normal_id)
{
  addPointCloud(in, pc_id);
  // if(!cloud_viewer_->updatePointCloud(normals, normal_id))
  {
    cloud_viewer_->addPointCloudNormals<PointT, pcl::Normal>(in, normals, level, scale, normal_id);
  }
  return true;
}




