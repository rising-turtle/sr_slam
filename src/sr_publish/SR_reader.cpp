#include "SR_reader.h"
#include <ros/ros.h>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <stdlib.h>
#include "libMesaSR.h"

sr_data::sr_data()
{
  timestamp_ = 0; 
  b_gt_has_been_set_ = false;
}

CSReader::CSReader()
{
  ros::NodeHandle nh("~");
  // nh.param("sr_data_file_dir", file_dir_, string("/home/davidz/work/data/SwissRanger4000/try")); // default parameters 
  nh.param("sr_data_file_dir", file_dir_, string("/home/davidz/work/data/SwissRanger4000/exp/dataset_82")); // default parameters 
  nh.param("sr_data_prefix", data_prefix_, string("d1")); 
  nh.param("sr_data_suffix", data_suffix_, string("bdat")); 
  nh.param("sr_start_frame", start_frame_, 1); 
  nh.param("sr_end_frame", end_frame_, 500); 
  nh.param("sr_new_file_version", b_sr_new_version_, false);
  nh.param("sr_specific_id", b_sr_specific_id_, false);
  nh.param("sr_specific_log", sr_specific_log_, string(""));
  nh.param("sr_fname_len",  fname_len_, 4); 
  curr_frame_ = 0;
}
CSReader::~CSReader(){}

int CSReader::get_number_frames()
{
  return data_set_.size();
}

void CSReader::setDataDir(string file_dir)
{
  file_dir_ = file_dir;
}

void CSReader::setPrefix(string prefix)
{
  data_prefix_ = prefix;
}

void CSReader::setSuffix(string suffix)
{
  data_suffix_ = suffix;
}

void CSReader::setStartFrame(int sf)
{
  start_frame_ = sf; 
}

void CSReader::setEndFrame(int ef)
{
  end_frame_ = ef;
}

void CSReader::setVersion(bool new_version)
{
  b_sr_new_version_ = new_version;
}

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

bool CSReader::readTimeStamps(vector<double>& ts)
{
  stringstream ss; 
  // ss<<file_dir_<<"/"<<data_prefix_<<"_"<<setfill('0')<<setw(4)<<i<<"."<<data_suffix_; 
  ss<<file_dir_<<"/timestamp.log";
  ifstream inf(ss.str().c_str()); 
  if(!inf.is_open())
  {
    ROS_WARN("SR_reader.cpp: timestamp file %s not exist", ss.str().c_str());
    return false; 
  }
  int id; 
  double time_value; 
  ts.resize(end_frame_ - start_frame_);
  for(int i=1, j=0; i<end_frame_; i++)
  {
     inf>>id>>time_value; 
     if(i>=start_frame_)
     {
      ts[j++] = time_value;
     }
  }
  inf.close();
  return ts.size() > 0;
}

bool CSReader::readFromGT(vector<double>& tv, vector<vector<float> >& pv, string f)
{
  ifstream inf(f.c_str());
  if(!inf.is_open())
  {
    ROS_ERROR("SR_reader.cpp: failed to get GT from file %s", f.c_str());
    return false; 
  }
  const int N = 1024; 
  tv.reserve(N); 
  pv.reserve(N); 

  char buf[N] = {0};
  double timestamp; 
  float x,y,z,qx,qy,qz,qw;
 
  vector<float> p(7); 
  while(inf.getline(buf, N))
  {
    sscanf(buf, "%lf %f %f %f %f %f %f %f", &timestamp, &x, &y, &z, &qx, &qy, &qz, &qw); 
    tv.push_back(timestamp); 
    p[0] = x; p[1] = y; p[2] = z; p[3] = qx; p[4] = qy; p[5] = qz; p[6] = qw; 
    pv.push_back(p); 
  }
 return true;
}

bool CSReader::synFromGT(string gt_file) // get the ground truth pose 
{
  // read all ground truth data 
  vector<vector<float> > pv; 
  vector<double> tv ;
  if(!readFromGT(tv, pv, gt_file))
  {
    return false;
  }

  for(int i=0; i<timestamps_.size(); i++)
  {
    double t = timestamps_[i]; 

    // find out the index of the corresponding timestamp 
    int j;
    for(j=0; j<tv.size(); j++)
    {
      if( fabs(tv[j]-t) < 0.01 )
      {
        data_set_[i].gt_pv_[0] = pv[j][0];   data_set_[i].gt_pv_[1] = pv[j][1];   data_set_[i].gt_pv_[2] = pv[j][2]; 
        data_set_[i].gt_pv_[3] = pv[j][3];   data_set_[i].gt_pv_[4] = pv[j][4];   data_set_[i].gt_pv_[5] = pv[j][5]; 
        data_set_[i].gt_pv_[6] = pv[j][6]; 
        data_set_[i].b_gt_has_been_set_ = true;
        ROS_INFO("SR_reader.cpp: find timestamp: %lf with %lf = %f %f %f %f %f %f %f", t, tv[j], pv[j][0], pv[j][1], pv[j][2],
            pv[j][3], pv[j][4], pv[j][5], pv[j][6]);
        break;
      }else
      {
        // 
      } 
    }
    if(j == tv.size())
    {
      ROS_WARN("SR_reader.cpp: failed to find out transformation at timestamp: %lf ", t); 
      data_set_[i].b_gt_has_been_set_ = false;
    }

  }
return true;
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
  
  // try to read timestamp 
  // vector<double> timestamps;
  bool b_retrive_time_log = false; 
  if(data_suffix_ != string("dat"))
  {
    b_retrive_time_log = readTimeStamps(timestamps_);
    if(b_retrive_time_log)
    {
      ROS_ERROR("SR_reader.cpp: succeed to read timestamps size %u", timestamps_.size());
    }else{
      ROS_ERROR("SR_reader.cpp: failed to read timestamps");
    }
  }

  // whether only using specific frame id
  vector<int> specific_id(end_frame_ - start_frame_, 0); 
  if(b_sr_specific_id_)
  {
    ifstream inf(sr_specific_log_.c_str());
    if(!inf.is_open())
    {
      ROS_ERROR("SR_reader.cpp: use specific id, but id log file %s cannot be found!", sr_specific_log_.c_str());
      b_sr_specific_id_ = false;
    }else
    {
      int id; 
      specific_id.clear();
      while(!inf.eof())
      {
        inf>>id; 
        specific_id.push_back(id);
      }
    }
  }

  // read all data in the predefined directory, with predefined frames
  sr_data tmp_data; 
  int time_index = 0;
  for(int i=start_frame_, j=0; i<end_frame_ && j<specific_id.size(); i++, j++)
  {
    stringstream ss; 
    // if(!b_sr_new_version_)
    {
      // old file version 
      if(b_sr_specific_id_)
         ss<<file_dir_<<"/"<<data_prefix_<<"_"<<setfill('0')<<setw(fname_len_)<<start_frame_+specific_id[j]<<"."<<data_suffix_; 
      else
         ss<<file_dir_<<"/"<<data_prefix_<<"_"<<setfill('0')<<setw(fname_len_)<<i<<"."<<data_suffix_; 
    }
    // else{
      // new file version
    //  ss<<file_dir_<<"/frame_"<<i<<".bin";
    // }
    if(data_suffix_ == string("dat"))
    {
      if(readOneFrameDat(ss.str(), tmp_data))
      {
        data_set_.push_back(tmp_data);
      }
    }
    else{ 
      if(readOneFrame(ss.str(), tmp_data))
      {
        if(b_retrive_time_log) // add timestamp 
        {
          if(b_sr_specific_id_)
          {
            time_index = specific_id[j];
            tmp_data.timestamp_ = timestamps_[time_index];
          }
          else
            tmp_data.timestamp_ = timestamps_[time_index++];
        }

        data_set_.push_back(tmp_data);
      }
    }
  }
  cout<<"SR_reader.cpp: load all data, have "<<data_set_.size()<<" frames!"<<endl;
  return data_set_.size() > 0;
}

bool CSReader::isEmpty()
{
  return (data_set_.size() == 0);
}

namespace
{
  template<typename T>
  bool readData(T* pTar, ifstream* inputf)
  {
    const static int N = 4096;
    static char buf[N] = {0};
    string delim(" \t");
    // For the realsense data has to comment this line, 
    // buf for sr4k data, uncomment this line to skip the '% Calibrated Distance Z' in the first line
    // Z 
    inputf->getline(buf, N); // % Calibrated Distance Z 

    // printf("SR_reader.cpp: skip part %s \n", buf);

    for(int row =0; row<SR_HEIGHT ; row++)
    {
      if(!inputf->getline(buf, N))
      {
        printf("SR_reader.cpp: what? cannot read the next line!\n");
        return false;
      }
      // printf("SR_reader.cpp: read data %s\n", buf);
      char * pt  = 0;

      for(int col =0; col < SR_WIDTH; col++)
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
        if(typeid(T).name() == typeid(SR_IMG_TYPE).name())
          pTar[row*SR_WIDTH + col] = atoi(pt);
        if(typeid(T).name() == typeid(SR_TYPE).name())
          pTar[row*SR_WIDTH + col] = atof(pt);
      }
    }
    return true;
  }

}

bool CSReader::readOneFrameDat(string f_name, sr_data& d)
{
  ifstream inputf(f_name.c_str());
  if(!inputf.is_open())
  {
    cout<<"SR_reader.cpp failed to read file: "<<f_name<<endl;
    inputf.close();
    return false;
  }
  
  const static int N = 8192;
  char buf[N] ={0}; 
  string delim(" ,\t");
  
  if(!readData<SR_TYPE>(&d.z_[0], &inputf)) return false;  // Z
  if(!readData<SR_TYPE>(&d.x_[0], &inputf)) return false;  // X
  if(!readData<SR_TYPE>(&d.y_[0], &inputf)) return false;  // Y
  if(!readData<SR_IMG_TYPE>(&d.intensity_[0], &inputf)) return false; // intensity 
  if(!readData<SR_IMG_TYPE>(&d.c_[0], &inputf)) return false; // confidence map
  
  // TODO: time stamp
  inputf.getline(buf, N);  // % additional data/ time stamp 
  inputf.getline(buf, N);  // first item is the timestamp 
  int timestamp = atoi(strtok(buf, delim.c_str()));
  d.timestamp_ = float(timestamp)*0.001;

  return true;
}

bool CSReader::readOneFrame(string f_name, sr_data& d)
{
  FILE* fid = fopen(f_name.c_str(), "rb"); 
  if(fid == NULL)
  {
    cout<<"SR_reader.cpp: failed to open file: "<<f_name<<endl; 
    return false;
  }
  if(!b_sr_new_version_)
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
    
    // 1 first intensity, and then 2 second distance
    // fread(&d.intensity_[0], sizeof(SR_IMG_TYPE), SR_SIZE, fid);
    fread(&d.mono_intensity_[0], sizeof(char), SR_SIZE, fid);
    fread(&d.dis_[0], sizeof(SR_IMG_TYPE), SR_SIZE, fid);
  }
  fclose(fid); 
  return true;
}

