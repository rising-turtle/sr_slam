/*
 * Oct. 28, 2015, David Z 
 *
 * Read the realsense data 
 *
 * */

#ifndef REALSENSE_READER_H
#define REALSENSE_READER_H

#include "SR_reader.h"

// const static int RS_WIDTH = 640; 
// const static int RS_HEIGHT = 480; 
// const static int RS_SIZE = SR_WIDTH*SR_HEIGHT;
class rs_data  // realsense data
{
  public:
    typedef float POINT_TYPE; 
    typedef unsigned char IMG_TYPE;
    const static int WIDTH = 640; 
    const static int HEIGHT = 480; 
    const static int SIZE = WIDTH * HEIGHT; 
  public:
    POINT_TYPE z_[SIZE]; 
    POINT_TYPE x_[SIZE]; 
    POINT_TYPE y_[SIZE];
    IMG_TYPE intensity_[SIZE];
};

class CRSReader
{
  public:
    CRSReader();
    ~CRSReader();
    bool readRSFile(string fname, rs_data& );
    bool readRSID(int id, rs_data&);

    string file_dir_;           // data dir 
    string data_prefix_;        // data prefix 
    string data_suffix_;        // data suffix 
    int start_frame_ ;          // start frame
    int end_frame_;             // the last frame
    int curr_frame_;            // current frame

};

#endif 
