/*
 *  Oct. 31 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  Input: *.pcd file, 
 *  Output: display colored plane segment
 *  
 *    Function: read point cloud data and from which extract and display planes 
 *
 * */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "plane_set.h"
#include "vtk_viewer.h"

using namespace std ;

string pcd_file = ""; 
double dis_threshold = 0.1; 

void markPlanePC(CloudPtr& pc, vector<int>& nv); 
void displayPC(CloudPtr& pc); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "view_extract_planes");
  ros::NodeHandle n;

  // 1. read pcd file 
  if( argc >= 2 )
    pcd_file = string(argv[1]);
  if( argc >= 3)
    dis_threshold = atof(argv[2]); 

  CloudPtr pc(new Cloud);  
  if(pcl::io::loadPCDFile(pcd_file, *pc) == 0)
  {
    cout<<"test_view_extract_planes: succeed to load node point cloud"<<pcd_file<<endl;
  }else
  {
    cout<<"test_view_extract_planes: failed to load point cloud: "<<pcd_file<<endl;
  }
  
  // 2. extract and show planes 
  CPlaneSet p_pSet; 
  CloudPtr out(new Cloud); 
  vector<int> ind;
  int num_p = p_pSet.extractPlanes(pc, out, ind, dis_threshold * dis_threshold); 
  
  if(num_p == 0) 
  {
    // show original plane
    cout <<"test_view_extract_planes: faild to extract planes!"<<endl; 
    cout <<"test_view_extract_planes: show original point cloud!"<<endl;
    displayPC(pc); 
  }else
  {
    // show the point cloud of the extracted planes 
    cout <<"test_view_extract_planes: extract "<<num_p<<" planes!"<<endl;
    cout <<"test_view_extract_planes: show extracted plane segment!"<<endl;
    markPlanePC(out, ind); 
    displayPC(out); 
  }

  return 0;  
}


void markPlanePC(CloudPtr& pc, vector<int>& nv)
{
  int start_i = 0; 
  int c = RED; 
  for(int i=0; i<nv.size(); i++)
  {
      int sid = start_i; 
      int eid = start_i + nv[i]; 
      for(int j=sid; j<eid; j++)
      {
        Point& pt = pc->points[j]; 
        pt.r = g_color[c][0]; 
        pt.g = g_color[c][1];
        pt.b = g_color[c][2]; 
      }
      c = (c++%6); 
      start_i = eid;
  }
}

void displayPC(CloudPtr& pc)
{
  CVTKViewer<pcl::PointXYZRGBA> v;
  // v.getViewer()->addCoordinateSystem(0.2, 0, 0); 
  v.addPointCloud(pc, "pc"); 
  while(!v.stopped())
  {
    v.runOnce(); 
    usleep(100*1000); 
  } 
}


