/*
 * Jun 1, 2015, David Z
 * read the IMU data 
 *
 * */

#ifndef IMU_READER_H
#define IMU_READER_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <iostream> 
#include <fstream> 
#include <string>
#include <strings.h>
/* baudrate settings are defined in <asm/termbits.h>, which is
   included by <termios.h> */
#define BAUDRATE B115200            
/* change this definition for the correct port */
#define bluetooth "/dev/rfcomm0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */

// #define FALSE 0
// #define TRUE 1
#define Total 500
#define GX_DS 0.022888    // Gyro Digital Sensitivity
#define AX_DS 0.000274666 // Acc  Digital Sensitivity
#define AX_UPPER 13468900 // UPPER limit for ax ay az
#define AX_LOWER 12567025 // LOWER limit for ax ay az
// volatile int STOP=FALSE; 

class CIMUReader 
{
  public:
    enum IMU_SIZE{
      PACK_SIZE=38
    };
  public:
    CIMUReader();
    ~CIMUReader();
  void init(); 
  bool getOne2(short& gx, short& gy, short& gz, short& ax, short& ay, short& az);
  inline bool getOne(short& gx, short& gy, short& gz, short& ax, short& ay, short& az);
  void getParam(); 
  bool getRollPitch(float& roll, float& pitch, int n);
  bool getRollPitch(float& roll, float& pitch);
  bool get(float* gx, float* gy, float* gz, float* ax, float* ay, float* az, int n=1);
  bool getAcc(float* ax, float* ay, float* az, int n=1);
  bool getVec(float* gx, float* gy, float* gz, int n=1);
  inline bool isValid(unsigned char buf[PACK_SIZE]);
  unsigned char buf_[PACK_SIZE];
  int fd;                         // blue tooth device handle
  bool is_device_ready;           // device ready indicator
  struct termios oldtio,newtio;   // device info struct

};



#endif
