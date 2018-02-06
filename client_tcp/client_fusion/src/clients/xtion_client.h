#ifndef XTION_CLIENT_H
#define XTION_CLIENT_H

#include "socket_client.h"
#include "mopenni2.h"
#include <string>
#include <opencv/cv.h>

using namespace std;

class CXtionClient : public CSocketClient
{
public:
    CXtionClient(string path_ = "../data");
    virtual ~CXtionClient();

    void stopSendXtion();    // send local disk data to server 
    bool sendXtionOffline();

    bool recordXtion();     // record Xtion data to local disk
    void stopRecordXtion();
protected:
    bool recordXtion2File(); // record current data in m_rgb m_dpt into disk
    bool sendXtion2Server(char* buf, int len); // send data to server

    bool m_bXtionReady;
    string m_savePath;
    mutable bool m_bStopSending;
    mutable bool m_bStopRecording;
    MOpenni2 m_openni;
    cv::Mat m_rgb; 
    cv::Mat m_dpt;
    double m_timestamp;
public:
    static void* thread_start_send2server(void* param);
    static void* thread_start_record2local(void* param);
};


#endif
