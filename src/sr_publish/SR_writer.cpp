
#include "SR_writer.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <sys/stat.h>
#include <unistd.h>
#include <ros/ros.h>
#include "SR_reader.h"

using namespace std;

CSRWriter::CSRWriter()
{
  init();
}
CSRWriter::~CSRWriter(){}

void CSRWriter::setDir(string dir)
{
  path_ = dir;   
}

// if path not exist, create it 
bool CSRWriter::notExistThenCreate(string path)
{
  struct stat dir;
  int result = -1;
  if(stat(path.c_str(), &dir) == 0 && S_ISDIR(dir.st_mode))
  {
    result = 0;
    cout<<"SR_writer.cpp: dir "<<path<<" exist!"<<endl;
  }else
  {
    // create this directory 
    cout<<"SR_writer.cpp: dir "<<path<<" not exist, create it now!"<<endl;
    result = mkdir(path.c_str(), 0700);
    // cout<<"SR_writer.cpp: mkdir return : "<<result<<endl;
  }
  return (result == 0);
}

void CSRWriter::init()
{
  // path_ = "./data"; 
  // pre_ = "d1";
  // suf_ = "bdat";
  // index_ = 0;

  ros::NodeHandle nh("~");
  nh.param("sr_data_file_dir", path_, string("/home/davidz/work/data/SwissRanger4000/try")); // default parameters 
  nh.param("sr_data_prefix", pre_, string("d1")); 
  nh.param("sr_data_suffix", suf_, string("bdat")); 
  nh.param("sr_new_file_version", b_sr_new_version_, true);
}
namespace
{
  template<typename T>
  void writeData(T* pIn, ofstream* ouf, string title)
  {
    (*ouf)<<title<<endl;
    for(int row =0; row<SR_HEIGHT; row++)
    {
      for(int col = 0; col<SR_WIDTH; col++)
      {
        (*ouf)<<pIn[row*SR_WIDTH + col]<<" ";
      }
      (*ouf)<<endl;
    }
  }
}


bool CSRWriter::writeSRDat(const char *fname, sr_data& sr )
{
  ofstream fout(fname); 
  if(!fout.is_open())
  {
    ROS_ERROR("SR_writer.cpp: failed to open file: %s", fname);
    return false; 
  }
  writeData<SR_TYPE>(&sr.z_[0], &fout, "%/ Calibrated Distance"); // Z
  writeData<SR_TYPE>(&sr.x_[0], &fout, "%/ Calibrated xVector");  // X 
  writeData<SR_TYPE>(&sr.y_[0], &fout, "%/ Calibrated yVector");  // Y
  writeData<SR_IMG_TYPE>(&sr.intensity_[0], &fout, "%/ Amplitude"); // intensity 
  writeData<SR_IMG_TYPE>(&sr.c_[0], &fout, "%/ Confidence map");  // confidence 
  return true;
}

bool CSRWriter::write(const char* buf, const size_t size)
{
  stringstream ss; 
  ss<<path_<<"/"<<pre_<<"_"<<setfill('0')<<setw(4)<<++index_<<"."<<suf_;
  if(index_==0) 
  {
    cout<<"unsigned int may not contain the number, overwrite will happen, error"<<endl;
    return false;
  }
  return writeBin(ss.str().c_str(), buf, size);
}

bool CSRWriter::writeBin(const char* fname, const char* buf, const size_t size)
{
  ofstream ouf_bin(fname, ios::out | ios::binary);
  // ouf_bin.write((char*)&size, sizeof(size_t));
  ouf_bin.write(buf, size);
  ouf_bin.close();
  return true;
}

