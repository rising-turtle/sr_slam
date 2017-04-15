/*
 * David Z, May 11 2015 
 *
 * tcp socket communication for receiving SR-4000 data 
 * 
 * */

#ifndef SR_TCP_SOCKET_H
#define SR_TCP_SOCKET_H

#include "protocolDefines.h"
#include <string>
#include <pthread.h>
#include <vector>

typedef unsigned short WORD;
// typedef unsigned int   DWORD;
typedef unsigned char   BYTE;
typedef int   SOCKET;

using namespace std; 

struct TCP_data{
  vector<char> data; 
};

class CSRTcpSocket
{
  public:
    CSRTcpSocket();
    virtual ~CSRTcpSocket(); 
    void ros_init(); // parameter initialization 
    bool open(); 
    bool get(TCP_data& tcp_data);
    void close();
    void closeServer();   // send cmd EXIT to server

    //// TCP socket Functions and Parameters /////
    // connect to server 
    bool connectToServer(const char* ip, unsigned int port_id, SOCKET& s); 
    int TCPRecv(SOCKET s, void *buf, int len, int flags);     // TCP functions
    int TCPClient1(const char* ip);

    string server_ip_; 
    unsigned int server_port_id_;
    SOCKET tcp_socket_;

    //// thread Functions and Parameters ////
    // thread to get image 
    static void* thread_SR_Receiver_Helper(void*);
    void* thread_SR_Recevicer();
    volatile bool b_thread_on;
    pthread_t thread_id; 
    pthread_mutex_t mut_data;

    char *data;                     // tcp recv data buffer 
    int data_n;                     // bufffer size, data_n = c_data.size()
    vector<char> c_data;            // critical region
    volatile bool b_new_data_ready; // weather the buffer has new data
};


#endif
