/*
 * David Z, May 11 2015 
 *
 * tcp socket communication for receiving SR-4000 data 
 * 
 * */

#include "SR_tcp_socket.h"
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <arpa/inet.h>

CSRTcpSocket::CSRTcpSocket() : 
  b_new_data_ready(false),
  tcp_socket_(-1),
  b_thread_on(false), 
  data(0),
  data_n(0)
{
  pthread_mutex_init(&mut_data,NULL);
  ros_init();
}

CSRTcpSocket::~CSRTcpSocket()
{
  if(data!=0) free(data);
}

void CSRTcpSocket::ros_init()
{
  ros::NodeHandle nh_p("~"); 
  int port_id;
  // nh_p.param("sr_server_ip", server_ip_, string("127.0.0.1")); 
  
  nh_p.param("sr_server_ip", server_ip_, string("192.168.1.5")); 
  // nh_p.param("sr_server_ip", server_ip_, string("192.168.1.3")); 
  nh_p.param("sr_server_port_id", port_id, 27015); 
  server_port_id_ = port_id;
  
  ROS_INFO("SR_tcp_socket.cpp: sr_server_ip: %s, port id %d", server_ip_.c_str(), server_port_id_);
}

void CSRTcpSocket::close()
{
  if(b_thread_on)
  {
    b_thread_on = false;
    pthread_join(thread_id, NULL);
    cout<<"SR_tcp_socket.cpp: thread has exited!"<<endl;
  }
  if(tcp_socket_ > 0)
  {
    closeServer();
    ::close(tcp_socket_);
  }
}

bool CSRTcpSocket::open()
{
  // connect to server 
  if(!connectToServer(server_ip_.c_str(), server_port_id_, tcp_socket_))
  {
    cout<<"SR_tcp_socket.cpp: failed to connect to server: "<<server_ip_<<" with port_id: "<<server_port_id_<<endl;
    return false;
  }
  
  // succeed to connect to server 
  int to=3000;//3000ms timeout
  setsockopt(tcp_socket_,SOL_SOCKET,SO_RCVTIMEO,(char*)&to,sizeof(to));

  // start thread 
  int tmp = pthread_create(&thread_id, 0, &CSRTcpSocket::thread_SR_Receiver_Helper, (void*)this);
  if(tmp !=0)
  {
    cout<<"SR_tcp_socket.cpp: failed to create thread!"<<endl;
    return false;
  }

  // this flag has to be set
  b_thread_on = true;
  return true;
}

bool CSRTcpSocket::get(TCP_data& tcp_data)
{
  if(!b_new_data_ready)
  {
    // cout<<"SR_tcp_socket.cpp: no new data, wait!"<<endl; 
    return false; 
  }
  
  if(tcp_data.data.size() != c_data.size())
  {
    tcp_data.data.resize(c_data.size(), 0);
  }

  char* buf = tcp_data.data.data();
  int N = tcp_data.data.size();

  // get the critical buffer data
    pthread_mutex_lock(&mut_data); 
    memcpy(buf, &c_data[0], N); 
    b_new_data_ready = false;
    pthread_mutex_unlock(&mut_data);
 
  return true;
}

// this is the strategy to use class member function in the thread
void* CSRTcpSocket::thread_SR_Receiver_Helper(void* p)
{
  CSRTcpSocket* pClass = (CSRTcpSocket*)(p); 
  pClass->thread_SR_Recevicer();
  return 0;
}
/*
void* CSRTcpSocket::thread_SR_Recevicer()
{
  int res;
 
  IPRSGetImage rx, hx;
  hx._p.id = ZH_ImageConfirm; // confirm msg 
  hx._p.size = sizeof(hx);

  bool b_connected = true;

  // 1, firstly send cmd: Start Stream 
  IPObj tx;
  tx.id = ZH_ImageStreamStart;  // start the ImageStreamStart
  tx.size=sizeof(tx);

  // start ImageTransferStart
  res=send(tcp_socket_,(char*)&tx,sizeof(tx),0);  
  printf("sent %d bytes, start imageTransfer\n",res);

  // For test TCP hz 
  ros::Time begin = ros::Time::now();
  int number_of_frames = 0;

  int size = sizeof(tx) + 1024 * 200;

  if(data == NULL || data_n < size)
  {
      if(data != NULL) 
        free(data);
      data=(char*)malloc(size);
      data_n = size;
      c_data.resize(data_n, 0);
  }

  // 2, then receive SR_data, and send Confirm tag
  while(b_thread_on) // sym
  {
    // res=TCPRecv(tcp_socket_,(char*)&rx,sizeof(rx),0);
    res = TCPRecv(tcp_socket_, data, size, 0);
    ++number_of_frames;
    // printf("TCPRecv consume %d\n",res);
    if(res<=0)
    {
      puts("thread_RecordImage.cpp: TCPRecv from mesa error");
      b_connected = false;
      break;
    }

    // critical area 
    pthread_mutex_lock(&mut_data); 
    memcpy(&c_data[0], data, data_n); 
    b_new_data_ready = true;
    pthread_mutex_unlock(&mut_data);

    // sleep 
    usleep(10); 
  }
  
  ros::Time end = ros::Time::now(); 
  double ms =  (end.sec - begin.sec)*1000+(end.nsec - begin.nsec)*0.000001;
  printf("SR_tcp_socket.cpp: recv %d frames, cost %f ms, Hz is: %f\n", number_of_frames, ms, 1000.*number_of_frames/ms);

  // still connected, so have to send cmd STOP 
  if(b_connected)
  {
    hx._p.id = ZH_ImageStreamStop;
    send(tcp_socket_,(char*)&hx,sizeof(hx),0);  
  }

  // free(data);
  printf("thread_RecordImage.cpp: exit thread_RecordImage!\n");
  pthread_exit(NULL);
  return 0;

}
*/

void* CSRTcpSocket::thread_SR_Recevicer()
{
  int res;
 
  IPRSGetImage rx, hx;
  hx._p.id = ZH_ImageConfirm; // confirm msg 
  hx._p.size = sizeof(hx);

  // unsigned char w=176,h=144;
  // int frame_num = 1;
  // char FName[255];
  // memset(FName, 0, sizeof(FName));

  bool b_connected = true;

  // 1, firstly send cmd: Start Stream 
  IPObj tx;
  // tx.id=RQ_GetImage;
  tx.id = ZH_ImageStreamStart;  // start the ImageStreamStart
  tx.size=sizeof(tx);

  // start ImageTransferStart
  res=send(tcp_socket_,(char*)&tx,sizeof(tx),0);  
  printf("sent %d bytes, start imageTransfer\n",res);

  // For test TCP hz 
  ros::Time begin = ros::Time::now();
  int number_of_frames = 0;

  // 2, then receive SR_data, and send Confirm tag
  while(b_thread_on) // sym
  {
    res=TCPRecv(tcp_socket_,(char*)&rx,sizeof(rx),0);
    ++number_of_frames;
    // printf("TCPRecv consume %d\n",res);
    if(res<=0)
    {
      puts("thread_RecordImage.cpp: TCPRecv from mesa error");
      b_connected = false;
      break;
    }

    if(rx._p.id != RS_GetImage)
    {
      printf("SR_tcp_socket.cpp: unexpected tag, something wrong!\n"); 
      b_connected = false; 
      break;
    }
    
    int size=rx._p.size-sizeof(IPRSGetImage); // assume each time the size is the same
    if(data == NULL || data_n < size)
    {
      if(data != NULL) 
        free(data);
      data=(char*)malloc(size);
      data_n = size;
      c_data.resize(data_n, 0);
    }

    // record data into file
    if(data)
    {
      // unsigned short* img=(unsigned short*)data;
      res=TCPRecv(tcp_socket_,data,size,0);
      
      // critical area 
        pthread_mutex_lock(&mut_data); 
        memcpy(&c_data[0], data, data_n); 
        b_new_data_ready = true;
        pthread_mutex_unlock(&mut_data);

      // sprintf(FName, "imgs/d1_%04d.bdat", frame_num++);
      // record2FileBin(FName, data, w*h*(3*sizeof(float) + 2*sizeof(WORD)));
      // printf("SR_tcp_socket.cpp: Host computer receive %d frames with %d bytes\n", frame_num - 1, res);
    }
    else
    {
      puts("malloc failed");
      break;
    }
    
    // send confirm tag  
    // do not have to send ACK 
    
    // res=send(tcp_socket_,(char*)&hx,sizeof(hx),0);  
    // if(res <= 0)
    // {
    //  printf("SR_tcp_socket.cpp: failed to send confirm tag, net may not connect\n"); 
    //  b_connected = false;
    //  break;
    // }else
    //  printf("SR_tcp_socket.cpp: succeed to send confirm tag: %d bytes\n",res);
    
    // sleep 
    usleep(240); 
  }
  
  ros::Time end = ros::Time::now(); 
  double ms =  (end.sec - begin.sec)*1000+(end.nsec - begin.nsec)*0.000001;
  printf("SR_tcp_socket.cpp: recv %d frames, cost %f ms, Hz is: %f\n", number_of_frames, ms, 1000.*number_of_frames/ms);

  // still connected, so have to send cmd STOP 
  if(b_connected)
  {
    hx._p.id = ZH_ImageStreamStop;
    send(tcp_socket_,(char*)&hx,sizeof(hx),0);  
  }

  // free(data);
  printf("thread_RecordImage.cpp: exit thread_RecordImage!\n");
  pthread_exit(NULL);
  return 0;
}

void CSRTcpSocket::closeServer()
{
  IPRSGetImage hx;
  hx._p.id = ZH_EXIT; // confirm msg 
  hx._p.size = sizeof(hx);
  send(tcp_socket_,(char*)&hx,sizeof(hx),0);  
}

int CSRTcpSocket::TCPRecv(SOCKET s, void* buf, int len, int flags)
{
  int pos,res;
  BYTE* p=(BYTE*)buf;

  for(pos=0,res=0;res<len;)
  {
    res = recv(s, (char*)&p[pos], len-pos, flags);
    if ( res <= 0)
    {
      printf("TCPRecv: recv[] failed %d. received %d/%d\n",res,pos,len);
      break;
    }
    else
    {
      if(!(flags&MSG_PEEK))
      {
        pos+=res;
        //Printf(MK_DEBUG_STRING|MC_ETH,"received %d/%d (+%d) bytes\r",pos,len,res);
        res=pos;
      }
      else
      {
        //Printf(MK_DEBUG_STRING|MC_ETH,"peeked %d/%d bytes\r",res,len);
      }
    }
  }
  //Printf(MK_DEBUG_STRING|MC_ETH,"received %d/%d (+%d) bytes done.\n",pos,len,res);
  return res;
}

bool CSRTcpSocket::connectToServer(const char* ip, unsigned int port_id, SOCKET& s)
{
  WORD rmtPort=htons(port_id);    // port id 
  int timeout=1000;               // ms
  struct sockaddr_in saRmt;       // Information about the saRmt
  int saLen=sizeof(saRmt);        // Length of struct sockaddr_in
  unsigned int rmtAddr=inet_addr(ip);
  // SOCKET s;
  puts("connect TCPClient...");
  if ((s=socket(AF_INET, SOCK_STREAM, 0))<0)
  { 
    puts("TCPConnect: socket() failed.");
    return false;
  }
  memset((char *)&saRmt, 0, sizeof(saRmt));
  saRmt.sin_family = AF_INET;
  saRmt.sin_port = rmtPort;
  saRmt.sin_addr.s_addr=rmtAddr;
  if (connect(s, (struct sockaddr*)&saRmt, sizeof(saRmt))!=0)
  {
    puts("TCPConnect: connect() failed.");
    // return -1;
    return false;
  }
  printf("TCPConnect connect on %s:%d\n",inet_ntoa(saRmt.sin_addr), ntohs(saRmt.sin_port));
  return true;
}
