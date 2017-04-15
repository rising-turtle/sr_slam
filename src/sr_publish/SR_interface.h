/*
 * David Z, Apr 14, 2015 
 * 
 * provide interface to access the Swiss Ranger data, 
 * either from : 
 *      1) local file 
 *      2) TCP-IP client 
 *      3) directly connect to the SwissRanger 4000
 * */

#ifndef SR_INTERFACE_H
#define SR_INTERFACE_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"
#include "libMesaSR.h"
#include "SR_reader.h"
#include "SR_tcp_socket.h"
#include "SR_writer.h"

using namespace std; 

// typedef enum{SR_FILE=0, SR_TCP, SR_CAM} SR_SOURCE;  

class CSRInterface
{
  public:
    CSRInterface();
    virtual ~CSRInterface();
    void ros_init();
  public:
    virtual bool open();
    virtual bool get( std_msgs::UInt8MultiArray& sr_array );
    virtual void close();
    string sr_src_;             // SR_CAM, SR_FILE, SR_TCP

    // SR_CAM parameters 
    SRCAM sr_cam_;              // physical SR 4000 
    string sr_cam_ip_;          // ip address SR 4000
    int sr_cam_it_time_;        // integrate time, fp = 1/4*(RO+IT*0.1), RO = 4.6ms
    string sr_cam_mode_;        // whether it is USB or TCP 

    // SR_FILE parameters
    CSReader *pSReader_;        // CSReader object to handle files
    bool b_device_ready_ ;      // indicator: weather the src device is ready 
    bool b_new_file_version;    // indicator: weather to use the old file fetch format, old (img, x, y, z), new (img, d)
    
    // SR_TCP parameters
    CSRTcpSocket * pSRTcp_;     // CSRTcp_socket object to handle tcp socket 
    TCP_data tcp_data;          // buf the tcp data

    vector<unsigned char> sqrt_map_; // map (short)[x] -> (unsigned char)sqrt([x])
    
    // SR_writer 
    CSRWriter *pSWriter_;
    bool b_record_data_;         // weather to record data 
    string record_dir_;          // where to record data

  public:
    bool openCam(SRCAM&);
  protected:
    bool getFromCam(std_msgs::UInt8MultiArray&);
    bool getFromFile(std_msgs::UInt8MultiArray&);
    bool getFromTcp(std_msgs::UInt8MultiArray&);

    // SR intensity buffer 
    vector<unsigned short> pRaw_img_; 
    vector<unsigned char> pGrey_img_; 

    // SR distance buffer 
    vector<unsigned short> pDis_;
    
    // SR coordinate buffer 
    vector<float> pX_; 
    vector<float> pY_;
    vector<float> pZ_;

    // SR publisher buffer 
    vector<unsigned char > pPub_;
  private: 
    void map_raw_img_to_grey(unsigned short* pRaw, unsigned char* pGrey, int N);
};


#endif
