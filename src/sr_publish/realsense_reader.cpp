#include "realsense_reader.h"
#include <ros/ros.h>

CRSReader::CRSReader()
{
  ros::NodeHandle nh("~");
  // nh.param("sr_data_file_dir", file_dir_, string("/home/davidz/work/data/SwissRanger4000/try")); // default parameters 
  nh.param("rs_data_file_dir", file_dir_, string("/media/work/work/data/realsense/10-23-2015_realsense_R200_etas_2nd_floor/captured_data")); // default parameters 
  nh.param("rs_data_prefix", data_prefix_, string("f")); 
  nh.param("rs_data_suffix", data_suffix_, string("dat")); 
  nh.param("rs_start_frame", start_frame_, 1); 
  nh.param("rs_end_frame", end_frame_, 50); 
}
CRSReader::~CRSReader(){}

namespace
{
  template<typename T, typename D>
  bool readData(T* pTar, ifstream* inputf)
  {
    const static int N = 8192;
    static char buf[N] = {0};
    string delim(" \t");
    // Z 
    // inputf->getline(buf, N); // % Calibrated Distance Z 
    // printf("SR_reader.cpp: skip part %s \n", buf);

    for(int row =0; row< D::HEIGHT ; row++)
    {
      if(!inputf->getline(buf, N))
      {
        printf("SR_reader.cpp: what? cannot read the next line!\n");
        return false;
      }
      // printf("SR_reader.cpp: read data %s\n", buf);
      char * pt  = 0;

      for(int col =0; col < D::WIDTH; col++)
      {
        if(col == 0)
          pt = strtok(buf, delim.c_str());
        else
          pt = strtok(NULL, delim.c_str());
        if(pt == NULL)
        {
          cerr<<"SR_reader.cpp: some error here in read data!"<<endl;
          return false;
        }
        // printf("SR_reader.cpp: successful to read data: %s\n", pt);
        // d.z_[row*SR_WIDTH + col] = atof(pt);
        if(typeid(T).name() == typeid(typename D::IMG_TYPE).name())
          pTar[row*D::WIDTH + col] = atoi(pt);
        if(typeid(T).name() == typeid(typename D::POINT_TYPE).name())
          pTar[row*D::WIDTH + col] = atof(pt);
      }
    }
    return true;
  }
}

bool CRSReader::readRSID(int id, rs_data& rs_d)
{
    stringstream ss; 
    ss<<file_dir_<<"/"<<data_prefix_<<"_"<<setfill('0')<<setw(4)<<id<<"."<<data_suffix_; 
    
    return readRSFile(ss.str(), rs_d);
}

bool CRSReader::readRSFile(string fname, rs_data& rs_d)
{
  ifstream inputf(fname.c_str());
  if(!inputf.is_open())
  {
    cout<<"SR_reader.cpp failed to read file: "<<fname<<endl;
    inputf.close();
    return false;
  }
  if(!readData<rs_data::POINT_TYPE, rs_data>(&rs_d.z_[0], &inputf)) return false;  // Z
  if(!readData<rs_data::POINT_TYPE, rs_data>(&rs_d.x_[0], &inputf)) return false;  // X
  if(!readData<rs_data::POINT_TYPE, rs_data>(&rs_d.y_[0], &inputf)) return false;  // Y
  if(!readData<rs_data::IMG_TYPE, rs_data>(&rs_d.intensity_[0], &inputf)) return false; // intensity 
  
  return true;
}
