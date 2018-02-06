/*
 *  Jun 10, 2015, David Z
 *  simple SR writer to record SR data 
 *
 * */

#ifndef SR_WRITER_H
#define SR_WRITER_H

#include <string>
#include <fstream>
// #include "timestamp.h"

using namespace std;

class CSRWriter
{
  public:
    CSRWriter();
    ~CSRWriter(); 
    void init(); 
    void setDir(string);
    // void notExistThenCreate(string);
    bool write(const char* buf, const size_t size);
    bool writeBin(const char* fname, const char* buf, const size_t size);

    // bool writeTimeStamp();
    ofstream * pfTime_;
    // void startTimeStamp();
    // TTimeStamp s_time_; // start time

    unsigned int index_;  // file index
    string path_; // where to write the file data
    string pre_;  // prefix
    string suf_;  // suffix
};


#endif
