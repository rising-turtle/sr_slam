
/*
 * To test the SR_writer
 *
 * */

#include "SR_writer.h"

#include <string>
#include <vector>
#include <iostream>
#include <string.h>
#include "libMesaSR.h"

using namespace std;

string path_dir = "/home/davidz/work/data/SwissRanger4000/try";
string srcam_ip = "192.168.1.42";
string sr_data_version = "new"; // only record depth (z) and intensity (i), 
                                // "old" means z x y intensity and confidence

bool openSR(SRCAM& , string); // open SR 
bool getData(SRCAM, char* buf, int N);
bool getDataNew(SRCAM, char* buf, int N);

// ./sr_record "PATH_DIR" "USB" "DATA_VERSION"
// ./sr_recrod "PATH_DIR" "TCP" "IP" "DATA_VERSION"

bool getDataMode(SRCAM cam, char* buf, int N)
{
  if(sr_data_version == "new")
      return getDataNew(cam, buf, N);
  return getData(cam, buf, N);
}

int main(int argc, char* argv[])
{
  SRCAM cam;
  CSRWriter sr_writer; 
  string mode("USB");
  if(argc >=2)
  {
    path_dir = string(argv[1]);
    cout<<"test_writer.cpp: path_dir: "<<path_dir<<endl;
    if(argc >= 3)
    {
      mode = argv[2];
      cout<<"test_writer.cpp: mode: "<<mode<<endl;
   
      if(argc >= 4)
      {
        if(mode == "USB")
        {
          sr_data_version = string(argv[3]);
          cout<<"test_writer.cpp: record data version "<<sr_data_version<<endl;
        }else if(mode == "TCP")
        {
          srcam_ip = string(argv[3]);
          cout<<"test_writer.cpp: srcam_ip: "<<srcam_ip<<endl;
          if(argc >= 5)
          {
            sr_data_version = string(argv[3]);
            cout<<"test_writer.cpp: record data version "<<sr_data_version<<endl;
          }
        }
      }
    }
  }
  sr_writer.setDir(path_dir); 
  sr_writer.notExistThenCreate(path_dir);
  if(!sr_writer.writeTimeStamp())
  {
    cout<<"test_writer.cpp failed to write time stamp file!"<<endl;
  }else
  {
    cout<<"test_writer.cpp succeed to write time stamp file!"<<endl;
  }
  if(openSR(cam, mode))
  {
    cout<<"test_writer.cpp: succeed to openSR"<<endl;
    // SR_Data buffer 
    int W = 176; 
    int H = 144;
    int Sec; 
    if(sr_data_version == "new")
    {
    // int Sec = sizeof(short)*3 + sizeof(float)*3; // depth + intensity + confidence, x, y, z
      Sec = sizeof(short)*2;
    }else
    {
      Sec = sizeof(short)*2 + sizeof(float)*3;
    }
    unsigned int total = W*H*Sec;
    char* buf = new char[total]; 
    
    int cnt = 0;
    sr_writer.startTimeStamp();
    
    cout<<"wait for start cmd!"<<endl;
    int k;
    cin>>k;

    for(;;)
    {
      // 
      // if(getData(cam, buf, W*H))
      // if(getDataNew(cam, buf, W*H))
      if(getDataMode(cam, buf, W*H))
      {
        if(!sr_writer.write(buf, total))
        {
          cout<<"test_writer.cpp: failed to write, return;"<<endl;
          break; 
        }else{
          cout<<"test_writer.cpp: succeed to write frame "<<++cnt<<endl;
        }
      }else
      {
        cout<<"test_writer.cpp: failed to getData, return;"<<endl;
        break;
      }
      usleep(1);
    }
    delete []buf;
    SR_Close(cam);
  }
  
  cout<<"finished!"<<endl;
  return 0; 
}

bool getDataNew(SRCAM cam, char* buf, int N)
{
  SR_Acquire(cam); 
  ImgEntry* imgEntryArray; 
  int nImg = SR_GetImageList(cam, &imgEntryArray); 
 
  // intensity image 
  unsigned short * pIntensity = (unsigned short*)imgEntryArray[1].data; 
 
  // depth image 
  unsigned short * pDepth = (unsigned short*)imgEntryArray[0].data;
  static vector<short> xs16(N, 0); 
  static vector<short> ys16(N, 0); 
  int short_s = sizeof(short);

  char* pS = buf; 
  unsigned short* zs16 = (unsigned short*)(pS + N*sizeof(unsigned short));

  // sequence : z,x,y,intensity,confindence,depth, to be consistent with the previous data
  SR_CoordTrfUint16(cam, &xs16[0], &ys16[0], zs16, short_s, short_s, short_s); 
  
  // first intensity and then depth 
  memcpy(pS, pIntensity, N*sizeof(unsigned short)); 
  // memcpy(pS + N*sizeof(unsigned short), pDepth, N*sizeof(unsigned short)); 
  
  return true;
}

bool getData(SRCAM cam, char* buf, int N)
{
    SR_Acquire(cam); 
    ImgEntry* imgEntryArray; 
    int nImg = SR_GetImageList(cam, &imgEntryArray); 

    static vector<float> xf32(N, 0); 
    static vector<float> yf32(N, 0); 
    static vector<float> zf32(N, 0);
    int float_s = sizeof(float);

    char* pS = buf; 

    // sequence : z,x,y,intensity,confindence,depth, to be consistent with the previous data
    SR_CoordTrfFlt(cam, &xf32[0], &yf32[0], &zf32[0], float_s, float_s, float_s); 
    
    // intensity image 
    unsigned short * pIntensity = (unsigned short*)imgEntryArray[1].data; 
    
    // confidence image 
    unsigned short * pConfidence = (unsigned short*)imgEntryArray[2].data; 

    // depth image 
    unsigned short * pDepth = (unsigned short*)imgEntryArray[0].data;

    unsigned int f_shift = N*sizeof(float);
    memcpy(pS, zf32.data(), f_shift); 
    memcpy(pS + f_shift, xf32.data(), f_shift); 
    memcpy(pS + f_shift*2, yf32.data(), f_shift); 
    
    unsigned int s_shift = N*sizeof(unsigned short); 
    memcpy(pS + f_shift*3, pIntensity, s_shift); 
    memcpy(pS + f_shift*3 + s_shift, pConfidence, s_shift); 
    // memcpy(pS + f_shift*3 + s_shift*2, pDepth, s_shift);
  
    return true;
}

bool openSR(SRCAM& cam, string mode) // USB first, TODO: add tcp SR 
{ 
  int ret ; 
  if(mode == string("TCP"))
  {
    // ret = SR_OpenETH(&cam, "192.168.0.11"); // open camera 
    ret = SR_OpenETH(&cam, srcam_ip.c_str()); // open camera 
    cout<<"pcl_mesa.cpp: open camera using TCP with IP: "<<srcam_ip<<endl;
  }else if(mode == string("USB"))
  {
    ret = SR_OpenUSB(&cam, 0x4000397); 
    cout<<"pcl_mesa.cpp: open camera using USB"<<endl;
  }else{
    ret = SR_OpenUSB(&cam, 0x4000397); 
    cout<<"pcl_mesa.cpp: unknown model: "<<mode<<" using USB default!"<<endl;
  }
  if(ret <= 0)
  {
    cout<<"test_writer.cpp: failed to open SRCAM"<<endl; 
    return false;
  }
  SR_SetIntegrationTime(cam, 80); // 80 30 60
  SR_SetMode(cam, AM_HW_TRIGGER | AM_MEDIAN | AM_COR_FIX_PTRN 
      | AM_CONV_GRAY | AM_CONF_MAP | AM_DENOISE_ANF | AM_MEDIANCROSS); 
  return true;
}
