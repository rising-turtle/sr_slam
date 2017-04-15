#include "imu_reader.h"
#include <cmath>
#include <vector>
#include "mu_sigma.hpp"
// #include "Eigen/Core"
#include <eigen3/Eigen/Eigen>


#define R2D(r) (((r)*180.)/M_PI)
#define D2R(d) (((d)*M_PI)/180.)

using namespace std;

CIMUReader::CIMUReader(): 
fd(-1), 
is_device_ready(false)
{
  init();
}
CIMUReader::~CIMUReader()
{
  if(is_device_ready)
  {
    // recover the previous setting
    tcsetattr(fd,TCSANOW,&oldtio);
  }
}

void CIMUReader::init()
{
  fd = open(bluetooth, O_RDWR | O_NOCTTY ); 
  if (fd <0) 
  {
    perror(bluetooth); // exit(-1); 
    return ;
  }

  tcgetattr(fd,&oldtio); /* save current serial port settings */
  bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */
  /* 
BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
CRTSCTS : output hardware flow control (only used if the cable has
all necessary lines. See sect. 7 of Serial-HOWTO)
CS8     : 8n1 (8bit,no parity,1 stopbit)
CLOCAL  : local connection, no modem contol
CREAD   : enable receiving characters
*/
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

  /*
IGNPAR  : ignore bytes with parity errors
ICRNL   : map CR to NL (otherwise a CR input on the other computer
will not terminate input)
otherwise make device raw (no other input processing)
*/
  newtio.c_iflag = IGNPAR; //IGNPAR | ICRNL

  /*
     Raw output.
     */
  newtio.c_oflag = 0;

  /*
ICANON  : enable canonical input
disable all echo functionality, and don't send signals to calling program
*/
  newtio.c_lflag = 0;  //ICANON

  /* 
     initialize all control characters 
     default values can be found in /usr/include/termios.h, and are given
     in the comments, but we don't need them here
     */
  newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
  newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
  newtio.c_cc[VERASE]   = 0;     /* del */
  newtio.c_cc[VKILL]    = 0;     /* @ */
  newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
  newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
  newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
  newtio.c_cc[VSWTC]    = 0;     /* '\0' */
  newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
  newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
  newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
  newtio.c_cc[VEOL]     = 0;     /* '\0' */
  newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
  newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
  newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
  newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
  newtio.c_cc[VEOL2]    = 0;     /* '\0' */

  /* 
     now clean the modem line and activate the settings for the port
     */
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,&newtio);

  // set device indicator
  is_device_ready = true;
}

bool CIMUReader::getOne2(short& gx, short& gy, short& gz, short& ax, short& ay, short& az)
{
  return getOne(gx, gy, gz, ax, ay, az);
}

inline bool CIMUReader::getOne(short& gx, short& gy, short& gz, short& ax, short& ay, short& az)
{
  if(!is_device_ready)
  {
    cout<<"imu_reader.cpp: device is not ready in getVec()"<<endl;
    return false;
  }
  int res;
  while(1)
  {
    res = read(fd, buf_, PACK_SIZE); 
    if(res <= 0 )
    {
      cout<<"imu_reader.cpp: failed to read the bluetooth!"<<endl;
      return false;
    }
    if(isValid(buf_))
    {
      gx = (short)((buf_[13])<<8 | buf_[14]); // 13, 14 => gx
      gy = (short)((buf_[15])<<8 | buf_[16]); // 15, 16 => gy
      gz = (short)((buf_[17])<<8 | buf_[18]); // 17, 18 => gz
      
      ax = (short)((buf_[19])<<8 | buf_[20]); // 19, 20 => ax
      ay = (short)((buf_[21])<<8 | buf_[22]); // 21, 22 => ay
      az = (short)((buf_[23])<<8 | buf_[24]); // 23, 24 => az
      int len = ax*ax + ay*ay + az*az; 
      if(len > AX_UPPER || len < AX_LOWER)
        continue;
      return true;
    }else
    {
      // cout<<"imu_reader.cpp: invalid data what? "<<endl;
      continue;
    }
    // usleep(10);
  }
  return false;
}

bool CIMUReader::get(float* gx, float* gy, float* gz, float* ax, float* ay, float* az, int n)
{
    return false;
}


bool CIMUReader::getAcc(float* ax_, float* ay_, float* az_, int n)
{
   short gx, gy, gz, ax, ay, az;
   float* x = ax_; float* y = ay_; float* z = az_; 
   int res;
   for(int i=0; i<n; )
   {
     if(!getOne(gx, gy, gz, ax, ay, az))
     {
       return false;
     }
    // cout<<"imu_reader.cpp: get ax ay az "<<ax<<" "<<ay<<" "<<az<<endl;  
    // float len = sqrt(ax*ax + ay*ay + az*az); 
    // cout<<"len unit: "<<len<<" "<<len*AX_DS<<endl;
    *x = ax*AX_DS; *y = ay*AX_DS; *z = az*AX_DS; 
    ++x; ++y; ++z;
    ++i;
   }
  // cout<<"imu_reader.cpp: get n= "<<n<<" pack, exist!"<<endl;
  return true;
}
/*
bool CIMUReader::getAcc(float* ax_, float* ay_, float* az_, int n)
{
   short gx, gy, gz, ax, ay, az;
   float* x = ax_; float* y = ay_; float* z = az_; 
   int res;
   for(int i=0; i<n; )
   {
    // if(!getOne(gx, gy, gz, ax, ay, az))
    // {
    //   return false;
    // }
    res = read(fd, buf_, PACK_SIZE); 
    if(res <= 0 )
    {
      cout<<"imu_reader.cpp: failed to read the bluetooth!"<<endl;
      return false;
    }
    if(isValid(buf_))
    {
      // gx = (short)((buf_[13])<<8 | buf_[14]); // 13, 14 => gx
      // gy = (short)((buf_[15])<<8 | buf_[16]); // 15, 16 => gy
      // gz = (short)((buf_[17])<<8 | buf_[18]); // 17, 18 => gz
  
      ax = (short)((buf_[19])<<8 | buf_[20]); // 19, 20 => ax
      ay = (short)((buf_[21])<<8 | buf_[22]); // 21, 22 => ay
      az = (short)((buf_[23])<<8 | buf_[24]); // 23, 24 => az
    }else
    {
      // cout<<"imu_reader.cpp: invalid buf? "<<endl;
      continue;
      // return false;
    }

    // cout<<"imu_reader.cpp: get ax ay az "<<ax<<" "<<ay<<" "<<az<<endl;  
    *x = ax*AX_DS; *y = ay*AX_DS; *z = az*AX_DS; 
    ++x; ++y; ++z;
    ++i;
   }
  // cout<<"imu_reader.cpp: get n= "<<n<<" pack, exist!"<<endl;
  return true;
}
*/
bool CIMUReader::getVec(float* gx_, float* gy_, float* gz_, int n)
{
  float* x = gx_; float* y = gy_; float* z = gz_;
  short gx, gy, gz, ax, ay, az;
  for(int i=0; i<n; i++)
  {
    if(!getOne(gx, gy, gz, ax, ay, az))
    {
      return false;
    }
    {
      *x = gx*GX_DS; *y = gy*GX_DS; *z = gz*GX_DS;
      ++x; ++y; ++z;
      cout<<"imu_reader.cpp: get "<<i+1<<" "<<gx<<" "<<gy<<" "<<gz<<endl;
    }
  }
  // cout<<"imu_reader.cpp: get n= "<<n<<" pack, exist!"<<endl;
  return true;
}

inline bool CIMUReader::isValid(unsigned char buf[PACK_SIZE])
{
  if(buf[0]==0xFF && buf[1]==0xFF && buf[2]==0xFF && buf[3]==0xFF)
  {
    unsigned char check_sum = 0;
      for(int i=0; i<PACK_SIZE-1; i++)
      {
        check_sum += buf[i];
      }
      if(check_sum == buf[PACK_SIZE-1])
        return true;
  }
  return false;
}

namespace{
// #include <eigen3/Eigen/Eigen>
bool testR(float r, float p, float ax, float ay, float az, float g)
{
  float cr = cos(r); float sr = sin(r);
  float cp = cos(p); float sp = sin(p); 
  Eigen::Matrix3f R_r = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f R_p = Eigen::Matrix3f::Identity(); 
  R_r(0,0) = cr;  R_r(0,2) = sr;
  R_r(2,0) = -sr; R_r(2,2) = cr;
  
  R_p(1,1) = cp; R_p(1,2) = -sp;
  R_p(2,1) = sp; R_p(2,2) = cp;

  R_r = R_p*R_r; 
  Eigen::Vector3f va ; 
  va(0) = ax; va(1) = ay; va(2) = az; 
  Eigen::Vector3f vg = R_r*va; 
  cout<<"imu_reader.cpp: given g: " <<g<<" from va: "<<va(0)<<" "<<va(1)<<" "<<va(2)<<" tp vg: "<<vg(0)<<" "<<vg(1)<<" "<<vg(2)<<endl;
  return true;
}

bool testR2(float r, float p, float ax, float ay, float az, float g)
{
  static float roll_align = D2R(0.35); 
  static float pitch_align = D2R(174.5);
  r -= roll_align;
  p -= pitch_align;
 
  float cr = cos(r); float sr = sin(r);
  float cp = cos(p); float sp = sin(p); 
  Eigen::Matrix3f R_r = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f R_p = Eigen::Matrix3f::Identity(); 
 
  // IMU frame
  //           Xb 
  //          / 
  //         /
  //        O---> Yb
  //        |
  //        |
  //        Zb

  // navigational frame 
  //        Yb Xb 
  //        | / 
  //        |/
  //        O---> Zb
  // roll around Zb
  R_r(0,0) = cr;  R_r(0,1) = -sr;
  R_r(1,0) = sr;  R_r(1,1) = cr;
  
  // pitch around Xb
  R_p(1,1) = cp; R_p(1,2) = -sp;
  R_p(2,1) = sp; R_p(2,2) = cp;
  
  // 
  R_r = R_p*R_r;
  Eigen::Vector3f va ; 
  va(0) = ax; va(1) = -az; va(2) = ay; 
  Eigen::Vector3f vg = R_r*va; 
  cout<<"imu_reader.cpp: given g: " <<g<<" from va: "<<va(0)<<" "<<va(1)<<" "<<va(2)<<" tp vg: "<<vg(0)<<" "<<vg(1)<<" "<<vg(2)<<endl;
  return true;
}

}

bool CIMUReader::getRollPitch(float& roll, float& pitch, int n)
{
  vector<float> r_n(n); 
  vector<float> p_n(n); 
  
  for(int i=0; i<n; i++)
  {
    if(!getRollPitch(r_n[i], p_n[i]))
    {
      return false;
    }
    if(r_n[i] != r_n[i] || p_n[i] != p_n[i])
    {
      cout<<"imu_reader.cpp: nan here, continue."<<endl;
      continue;
    }
  }  

  // use mean +/- t*sigma to filter data 
  // first, compute mean and sigma
  float r_mean, r_sigma, p_mean, p_sigma; 
  compute_mu_sigma<float>(r_n.data(), r_n.size(), r_mean, r_sigma); 
  compute_mu_sigma<float>(p_n.data(), p_n.size(), p_mean, p_sigma);

  // check the distance between mean relative to sigma
  float sum_roll = 0; 
  float sum_pitch = 0;
  int cnt = 0;
  int cnt_failed = 0;
  float t = 1.1;
  float r_thre = SQ(t*r_sigma); 
  float p_thre = SQ(t*p_sigma);
  for(int i=0; i<n; i++)
  {
    if(SQ(r_n[i]-r_mean) > r_thre || SQ(p_n[i]-p_mean) > p_thre)
    {
      ++cnt_failed;
      continue;
    }
    sum_roll += r_n[i]; 
    sum_pitch += p_n[i];
    ++cnt;
  }

  // cout<<"imu_reader.cpp: filter "<<cnt_failed<<" samples!"<<endl;
  if(cnt == 0) return false;
  
  roll = sum_roll / cnt; 
  pitch = sum_pitch / cnt;

  // cout<<"imu_reader.cpp: get roll: "<<R2D(roll)<<" and pitch: "<<R2D(pitch)<<endl;

  // testR(roll, pitch, ax, ay, az, g);
  return true;

}


bool CIMUReader::getRollPitch(float& roll, float& pitch)
{
  float ax, ay, az; 
  if(!getAcc(&ax, &ay, &az))
  {
    return false;
  }
  
  // cout<<"imu_reader.cpp: ax: "<<ax<<" ay: "<<ay<<" az: "<<az<<endl;

  // translate the IMU reference into camera reference 
  // ax = -ax; 
  // az = -az;

  // roll = -tan-1(ax/az)
  roll = -atan2(ax, az); 
  
  // 
  float g = 9.8*sqrt(ax*ax + ay*ay + az*az);
   ax = ax*g; 
   ay = ay*g;
   az = az*g;

  float inv_g = 1./g;
  float u = -ay*inv_g; 
  float c_roll = cos(roll); 
  float s_roll = sin(roll); 
  float v = -(az*c_roll - ax*s_roll)*inv_g;
  float c = sqrt(u*u + v*v); 
  float inv_c = 1./c;
  float gama = atan2(v, u); 
  if(inv_c >1. )
    pitch = M_PI/2. - gama;
  else
    pitch = asin(inv_c) - gama;
  
  if(pitch!=pitch) // nan
  {
    cout<<"nan: u: "<<u<<" v "<<v<<" gama: "<<gama<<" c: "<<c<<" inv_c: "<<inv_c<<endl;
  }

  //cout<<"imu_reader.cpp:  ax "<<ax<<" ay "<< ay<<" az "<<az<<" roll "<<R2D(roll)<<" pitch: "<<R2D(pitch)<<endl;
  
  // testR(roll, pitch, ax, ay, az, g);
  // testR2(roll, pitch, ax, ay, az, g);

  return true;
}

void CIMUReader::getParam()
{
  short gx, gy, gz, ax, ay, az;
  if(!getOne(gx, gy, gz, ax, ay, az))
  {
    return; 
  }
  // default is 
  // Accelerometer Dynamic Range : 6g 
  // Gyroscope Dynamic Range : 500
  // Output Sample Rate : 150

  char GrSr = buf_[9];  // Gyro Range/Sample Rate 
  char Ar = buf_[10];   // Accelerometer Range
  int gr, sr, ar; 
  sr = ar = -1;

  // degree
  switch(GrSr & 0xF0)
  {
    case 0x00:
      gr = 250; 
      break;
    case 0x10:
      gr = 500;
      break;
    case 0x20:
      gr = 2000;
      break;
  }

  // Hz
  switch(GrSr & 0x0F)
  {
    case 0x00:
      sr = 100;
      break;
    case 0x01:
      sr = 125;
      break;
    case 0x02:
      sr = 150;
      break;
  }

  // g
  switch(Ar & 0xF0)
  {
    case 0x00:
      ar = 2;
      break;
    case 0x10:
      ar = 4; 
      break;
    case 0x20:
      ar = 8; // where is 6g? 
      break;
    case 0x30:
      ar = 16;
      break;
  }

  cout<<"imu_reader.cpp: gr = "<<gr<<" sr = "<<sr<<" ar ="<<ar<<endl;
  return ;
}





