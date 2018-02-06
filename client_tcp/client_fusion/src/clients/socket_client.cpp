#include "socket_client.h"
#include <iostream>
#include <string.h>
#include <stdio.h>

#ifndef WIN32
#include <arpa/inet.h> //inet_addr
#include <unistd.h>
#endif

using namespace std;

DWORD WINAPI CSocketClient::thread_start_record(LPVOID param_)
{
	CSocketClient * pClient = static_cast<CSocketClient*>(param_);
	pClient->startRecordData();
	return 1;
}

DWORD WINAPI CSocketClient::thread_start_send(LPVOID param_)
{
	CSocketClient *pClient = static_cast<CSocketClient*>(param_);
	pClient->startSendDataOffline();
	return 1;
}


CSocketClient::CSocketClient(): 
m_bConneted(false),
m_bRecord(false),
m_bSendOffline(false)
{
}
CSocketClient::~CSocketClient()
{
	if(m_bConneted)
	{
#ifdef WIN32
		if(m_socket_desc) 
			closesocket(m_socket_desc);
		WSACleanup();
#else
		close(m_socket_desc);
#endif
	}
}

bool CSocketClient::connect(char* hip, int hport)
{
#ifdef WIN32
	// Start up Winsock ...
	WSADATA wsadata;
	int error = WSAStartup(0x0202, &wsadata);

	if(error)
	{
		cout<<"error to start up WSA!"<<endl;
		return false;
	}
	
	// Did we get the right Winsock version
	if(wsadata.wVersion != 0x0202)
	{
		cout<<"wsa get the version version: "<<std::hex<<wsadata.wVersion<<endl;
		WSACleanup();
		return false;
	}
	// Create socket
	m_socket_desc = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(m_socket_desc == INVALID_SOCKET)
	{
		cout<<"socket_client.cpp: failed to create socket!"<<endl;
		return false;
	}
#else
	// Create socket
	m_socket_desc = socket(AF_INET , SOCK_STREAM , 0);
	if (m_socket_desc == -1)
	{
		printf("Could not create socket\n");
		return false;
	}
#endif
   
   
	// Fill out the information needed to initialize a socket ...
#ifdef WIN32
	SOCKADDR_IN server; 
#else
    // Hoster info
    sockaddr_in server;
#endif
    server.sin_addr.s_addr = inet_addr(hip);
    server.sin_family = AF_INET; 
    server.sin_port = htons(hport);
#ifdef WIN32
	if(::connect(m_socket_desc,(SOCKADDR*)&server, sizeof(server)) == SOCKET_ERROR)
#else
    if(::connect(m_socket_desc, (sockaddr*)&server, sizeof(server)) < 0)
#endif
    {
        cerr<<"socket_client.cpp: could not connect to ip: "<<hip<<" with port: "<<hport<<endl;
        return false;
    }else
    {
        cout<<"socket_client.cpp: succeed connect to ip: "<<hip<<" with port: "<<hport<<endl;
    }
    m_bConneted = true;
	return true;
}

bool CSocketClient::sendData(char* sendbuf, unsigned int len)
{
    bool ret = false;
    if(!m_bConneted)
    {
        cout<<"socket_client.cpp: not connect yet!"<<endl;
        return false;
    }
    unsigned int iResult = 0;

    iResult = send(m_socket_desc, sendbuf, len, 0);
	Sleep(100);
#ifdef WIN32
	if(iResult == SOCKET_ERROR)
#else
    if(iResult <= 0)
#endif
        cout<<"socket_client.cpp: failed to send data! iResult < 0 "<<endl;
    else if(iResult != len)
        cout<<"socket_client.cpp: failed to send complete data! iResult < len "<<endl;
    else
    { 
        ret = true;
        cout<<"socket_client.cpp: succeed to send data len: "<<iResult<<endl;
    }
    return ret;
}

bool CSocketClient::recvData(char* recvbuf, unsigned int len)
{
    if(!m_bConneted)
    {
        cout<<"socket_client.cpp: not connect yet!"<<endl;
        return false;
    }
    unsigned int iResult = 0; 
    bool ret = false;
    iResult = recv(m_socket_desc, recvbuf, len, 0);
    if(iResult <= 0)
        cout<<"socket_client.cpp: failed to recv data! iResult < 0 "<<endl;
    else
    { 
        ret = true;
        cout<<"socket_client.cpp: succeed to recv data len: "<<iResult<<endl;
    }
    return ret;
}

bool CSocketClient::startRecordData()
{
	if(m_bRecord)
	{
		cout<<"socket_client.cpp: already start!"<<endl;
		return true;
	}
	m_bRecord = true;
	while(m_bRecord)
	{
		cout<<"socket_client.cpp: default behavior: nothing to do!"<<endl;
		Sleep(5000); // 
	}
}

void CSocketClient::stopRecordData()
{
	m_bRecord = false;
}

bool CSocketClient::startSendDataOffline()
{
	if(m_bSendOffline)
	{
		cout<<"socket_client.cpp: already start!"<<endl;
		return true;
	}
	m_bSendOffline = true;
	while(m_bSendOffline)
	{
		cout<<"socket_client.cpp: default behavior : nothing to do!"<<endl;
		Sleep(5000);
	}
}
void CSocketClient::stopSendDataOffline()
{
	m_bSendOffline = false;
}
