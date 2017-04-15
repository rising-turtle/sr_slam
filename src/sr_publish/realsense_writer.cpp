#include "realsense_writer.h"
#include <ros/ros.h>

CRSWriter::CRSWriter()
{
  ros::NodeHandle nh("~");
  nh.param("rs_data_save_dir", file_dir_, string("./rs_data")); // default parameters 
  nh.param("rs_data_prefix", data_prefix_, string("f")); 
  nh.param("rs_data_suffix", data_suffix_, string("dat")); 
}
CRSWriter::~CRSWriter()
{
}

namespace
{
  template<typename T, typename D>
  void writeData(T* pIn, ofstream* ouf, string title)
  {
    // (*ouf)<<title<<endl;
    for(int row =0; row< D::HEIGHT; row++)
    {
      for(int col = 0; col< D::WIDTH; col++)
      {
        if(typeid(T).name() == typeid(unsigned char).name())
          (*ouf)<<(int)pIn[row*D::WIDTH + col]<<" ";
        else
          (*ouf)<<pIn[row*D::WIDTH + col]<<" ";
      }
      (*ouf)<<endl;
    }
  }
}

bool CRSWriter::writeRSFile(string fname, rs_data& rs)
{
  ofstream fout(fname); 
  if(!fout.is_open())
  {
    ROS_ERROR("SR_writer.cpp: failed to open file: %s", fname.c_str());
    return false; 
  }
  writeData<rs_data::POINT_TYPE, rs_data>(&rs.z_[0], &fout, "%/ Calibrated Distance"); // Z
  writeData<rs_data::POINT_TYPE, rs_data>(&rs.x_[0], &fout, "%/ Calibrated xVector");  // X 
  writeData<rs_data::POINT_TYPE, rs_data>(&rs.y_[0], &fout, "%/ Calibrated yVector");  // Y
  writeData<rs_data::IMG_TYPE, rs_data>(&rs.intensity_[0], &fout, "%/ Amplitude"); // intensity 
  // writeData<SR_IMG_TYPE>(&sr.c_[0], &fout, "%/ Confidence map");  // confidence 
  return true;
}


bool CRSWriter::writeRSID(int id, rs_data& rs_d)
{
    stringstream ss; 
    ss<<file_dir_<<"/"<<data_prefix_<<"_"<<setfill('0')<<setw(4)<<id<<"."<<data_suffix_; 
    
    return writeRSFile(ss.str(), rs_d);
}


