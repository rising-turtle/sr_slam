#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#ifdef WIN32
#define _WIN32_WINNT 0x0501
	#include "NodeHandle.h"
	// #include "HomeMessages/messages_@home_zjupanda_odometer.pb.h"
	// #include "HomeMessages/messages_@home_zjupanda_laser.pb.h"
	#include <WinSock2.h>
	#include <WinSock.h>
	#include <windows.h>
#else
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <pthread.h>
#endif
// 1. Create a socket
// 2. Connect to remote server
// 3. Send some data
// 4. Receive a reply

class CSocketClient
{
public:
    CSocketClient();
    virtual ~CSocketClient();
    bool connect(char* hip, int hport);
    bool recvData(char* recvbuf, unsigned int len);
    bool sendData(char* sendbuf, unsigned int len);
    bool sendXtion();
	virtual void run();
	virtual bool startRecordData();
	virtual void stopRecordData();
	virtual bool startSendDataOffline();
	virtual void stopSendDataOffline();
	static DWORD WINAPI thread_start_record(LPVOID param_);
	static DWORD WINAPI thread_start_send(LPVOID param_);
protected:
#ifdef WIN32 
	SOCKET m_socket_desc;
#else
    int m_socket_desc;
#endif
    bool m_bConneted;
	mutable bool m_bRecord; 
	mutable bool m_bSendOffline;
};


#endif
