/*
 May 12, 2015 David Z
	a server interface in windows 
*/

#ifndef BASE_SERVER_H
#define BASE_SERVER_H

#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>

#pragma comment (lib, "Ws2_32.lib")
#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"

// only handle one client session, no 
// reconnect, get a client and wait for its work
class CServerBase
{
public:
	CServerBase(std::string port_id=DEFAULT_PORT);
	virtual ~CServerBase(); 
	void init();			// listen socket 
	void listen();			// listen for a connecting socket
	void wait();			// wait for its job

	// for thread functions
	static DWORD WINAPI thread_do_it(LPVOID param_);
	virtual void do_it();

	// for thread functions
	static DWORD WINAPI thread_send_it(LPVOID param_);
	virtual void send_it();

	// for data transmission 
	bool recvData(char* recvbuf, unsigned int len); 
	bool sendData(char* sendbuf, unsigned int len);

	volatile bool bConnected; // weather there is a connected client 
	
	// TCP port stuff
	SOCKET ListenSocket;
	SOCKET ClientSocket;
	std::string listen_port;
	
	// thread variables
	HANDLE thread_to_handle_client;
	DWORD thread_to_handle_client_id;

	// thread variables
	HANDLE thread_to_send_client;
	DWORD thread_to_send_client_id;

};


#endif