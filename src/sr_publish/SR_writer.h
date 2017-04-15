/*
 *  Jun 10, 2015, David Z
 *  simple SR writer to record SR data 
 *
 * */

#ifndef SR_WRITER_H
#define SR_WRITER_H

#include <string>
class sr_data;
using namespace std;

class CSRWriter
{
  public:
    CSRWriter();
    ~CSRWriter(); 
    void init(); 
    void setDir(string);
    bool notExistThenCreate(string);
    bool write(const char* buf, const size_t size);
    bool writeBin(const char* fname, const char* buf, const size_t size);
    bool writeSRDat(const char *fname, sr_data& sr );

    unsigned int index_;  // file index
    string path_; // where to write the file data
    string pre_;  // prefix
    string suf_;  // suffix
    bool b_sr_new_version_; // new version: only depth and intensity
};


#endif
