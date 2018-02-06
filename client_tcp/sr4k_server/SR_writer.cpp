#include "stdafx.h"
#include "SR_writer.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

// #include <sys/stat.h>
// #include <unistd.h>

using namespace std;

CSRWriter::CSRWriter():
  pfTime_(0)
{
  init();
}
CSRWriter::~CSRWriter()
{
  if(pfTime_ == 0)
  {
    pfTime_->close();
    delete pfTime_;
  }
}

void CSRWriter::setDir(string dir)
{
  path_ = dir;   
}

// if path not exist, create it 
//void CSRWriter::notExistThenCreate(string path)
//{
//  struct stat dir;
//  if(stat(path.c_str(), &dir) == 0 && S_ISDIR(dir.st_mode))
//  {
//    cout<<"SR_writer.cpp: dir "<<path<<" exist!"<<endl;
//  }else
//  {
//    // create this directory 
//    cout<<"SR_writer.cpp: dir "<<path<<" not exist, create it now!"<<endl;
//    int result = mkdir(path.c_str(), 0777);
//    cout<<"SR_writer.cpp: mkdir return : "<<result<<endl;
//  }
//  return ;
//}

//bool CSRWriter::writeTimeStamp()
//{
//  stringstream ss; 
//  ss<<path_<<"/timestamp.log";
//  pfTime_ = new ofstream(ss.str().c_str()); 
//  if(!pfTime_->is_open())
//  {
//    cout<<"SR_writer.cpp: failed to create time stamp file!"<<endl;
//    return false;
//  }
//  
//  return true;
//}

void CSRWriter::init()
{
  path_ = "./data"; 
  pre_ = "d1";
  suf_ = "bdat";
  index_ = 0;
}


bool CSRWriter::write(const char* buf, const size_t size)
{
  stringstream ss; 
  // ss<<path_<<"/"<<pre_<<"_"<<setfill('0')<<setw(4)<<++index_<<"."<<suf_;
  ss<<path_<<"\\"<<pre_<<"_"<<setfill('0')<<setw(4)<<++index_<<"."<<suf_;
  if(index_==0) 
  {
    cout<<"unsigned int may not contain the number, overwrite will happen, error"<<endl;
    return false;
  }
  
  // record time also 
 /* if(pfTime_ != 0)
  {
    TTimeStamp cur_time = getCurrentTime();
    (*pfTime_)<<setfill('0')<<setw(4)<<index_<<" "<<timeDifference(s_time_, cur_time)<<endl;
  }*/

  return writeBin(ss.str().c_str(), buf, size);
}

//void CSRWriter::startTimeStamp()
//{
// // s_time_ = getCurrentTime();
//}

bool CSRWriter::writeBin(const char* fname, const char* buf, const size_t size)
{
  ofstream ouf_bin(fname, ios::out | ios::binary);
  // ouf_bin.write((char*)&size, sizeof(size_t));
  ouf_bin.write(buf, size);
  ouf_bin.close();
  return true;
}

