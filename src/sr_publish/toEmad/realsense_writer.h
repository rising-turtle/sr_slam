/*
 *  Oct. 29, 2015, David Z 
 *
 *  Write the realsense data 
 *
 * */

#ifndef REALSENSE_WRITER_H
#define REALSENSE_WRITER_H

#include "realsense_reader.h"

class CRSWriter
{
  public:
    CRSWriter();
    ~CRSWriter(); 
    bool writeRSFile(string fname, rs_data& ); 
    bool writeRSID(int id, rs_data& );
    string file_dir_;           // data dir 
    string data_prefix_;        // data prefix 
    string data_suffix_;        // data suffix 
};

#endif 
