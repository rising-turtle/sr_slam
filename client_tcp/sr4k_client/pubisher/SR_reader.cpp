#include "SR_reader.h"
#include <ros/ros.h>
#include <sstream>
#include <iomanip>
#include "libMesaSR.h"

CSReader::CSReader()
{
  ros::NodeHandle nh("~");
  nh.param("sr_data_file_dir", file_dir_, string("/home/davidz/work/data/SwissRanger4000/try")); // default parameters 
  nh.param("sr_data_prefix", data_prefix_, string("d1")); 
  nh.param("sr_data_suffix", data_suffix_, string("bdat")); 
  nh.param("sr_start_frame", start_frame_, 1); 
  nh.param("sr_end_frame", end_frame_, 50); 
  nh.param("sr_new_file_version", b_sr_new_version, true);
  curr_frame_ = 0;
}
CSReader::~CSReader(){}

void CSReader::reset(int index)
{
  curr_frame_ = index % data_set_.size();
}

sr_data CSReader::get_current_frame(bool& finished)
{
  sr_data t;
  if(isEmpty())
  {
    finished = true;
    cout<<"SR_reader.cpp: data is empty!"<<endl;
    return t;
  }
  if(curr_frame_ == data_set_.size())
  {
    finished = true;
    cout<<"SR_reader.cpp: data has been traversed!"<<endl;
    return t;
  }
  finished = false;
  curr_frame_ = curr_frame_ % data_set_.size();
  // cout<<"SR_reader.cpp: return data "<<curr_frame_<<endl;
  return data_set_[curr_frame_++];
}

sr_data CSReader::get_frame(int i)
{
  if(i<0 || i>data_set_.size() || isEmpty())
  {
    cout<<"SR_reader.cpp: i = "<<i<<" data has "<<data_set_.size()<<" elements!"<<endl;
    sr_data t; 
    return t; 
  }
  return data_set_[i];
}

bool CSReader::loadAllData()
{
  // if already has data, delete them 
  if(data_set_.size() > 0 )
  {
    cout<<"SR_reader.cpp: already cache data, clear them!"<<endl;
    vector<sr_data> tmp; 
    data_set_.swap(tmp);
  }
  // read all data in the predefined directory, with predefined frames
  sr_data tmp_data; 
  for(int i=start_frame_; i<end_frame_; i++)
  {
    stringstream ss; 
    // if(!b_sr_new_version)
    {
      // old file version 
      ss<<file_dir_<<"/"<<data_prefix_<<"_"<<setfill('0')<<setw(4)<<i<<"."<<data_suffix_; 
    }
    // else{
      // new file version
    //  ss<<file_dir_<<"/frame_"<<i<<".bin";
    // }
    if(readOneFrame(ss.str(), tmp_data))
    {
      data_set_.push_back(tmp_data);
    }
  }
  cout<<"SR_reader.cpp: load all data, have "<<data_set_.size()<<" frames!"<<endl;
  return data_set_.size() > 0;
}

bool CSReader::isEmpty()
{
  return (data_set_.size() == 0);
}

bool CSReader::readOneFrame(string f_name, sr_data& d)
{
  FILE* fid = fopen(f_name.c_str(), "rb"); 
  if(fid == NULL)
  {
    cout<<"SR_reader.cpp: failed to open file: "<<f_name<<endl; 
    return false;
  }
  if(!b_sr_new_version)
  {
    // old file version 
    fread(&d.z_[0], sizeof(SR_TYPE), SR_SIZE, fid); 
    fread(&d.x_[0], sizeof(SR_TYPE), SR_SIZE, fid);
    fread(&d.y_[0], sizeof(SR_TYPE), SR_SIZE, fid); 
    fread(&d.intensity_[0], sizeof(SR_IMG_TYPE), SR_SIZE, fid); 
    fread(&d.c_[0], sizeof(SR_IMG_TYPE), SR_SIZE, fid);
  }else
  {
    // new file version, the offset way does not work, instead using the camera model way
    // int cam_offset = sizeof(SRCAM); 
    // cout<<"SR_reader.cpp: sizeof(SRCAM) is "<<cam_offset<<endl;
    // fread(&d.cam_handle_[0], sizeof(unsigned char), cam_offset, fid);
    fread(&d.intensity_[0], sizeof(SR_IMG_TYPE), SR_SIZE, fid);
    fread(&d.dis_[0], sizeof(SR_IMG_TYPE), SR_SIZE, fid);
  }
  fclose(fid); 
  return true;
}

