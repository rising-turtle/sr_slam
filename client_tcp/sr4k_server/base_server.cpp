#include "stdafx.h"
#include "base_server.h"

DWORD WINAPI CServerBase::thread_do_it(LPVOID param_)
{
	CServerBase* pServer = static_cast<CServerBase*>(param_); 
	pServer->do_it();
	ExitThread(0);
	return 1;
}

DWORD WINAPI CServerBase::thread_send_it(LPVOID param_)
{
	CServerBase* pServer = static_cast<CServerBase*>(param_); 
	pServer->send_it();
	ExitThread(0);
	return 1;
}

CServerBase::CServerBase(std::string port_id): 
listen_port(port_id),
bConnected(false)
{
	init();
}
CServerBase::~CServerBase()
{
	if(bConnected)
	{
		if(ListenSocket)
		{
			closesocket(ListenSocket);
		}
		WSACleanup();
	}
}

// init socket for listenning 
void CServerBase::init()
{
	WSADATA wsaData;
    int iResult;
	ListenSocket = INVALID_SOCKET;
	ClientSocket = INVALID_SOCKET;
	struct addrinfo *result = NULL;
    struct addrinfo hints;
	
	// Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return ;
    }

	ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;
	
	// Resolve the server address and port
    // iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
	iResult = getaddrinfo(NULL, listen_port.c_str(), &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return ;
    }
	
	 // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        // return 1;
		return ;
    }

    // Setup the TCP listening socket
    iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return ;
    }

    freeaddrinfo(result);
	return ;
}

// interface for handling something 
void CServerBase::do_it()
{
	printf("base_server.cpp: nothing to do here..\n");
	return;
}

// interface for sending something
void CServerBase::send_it()
{
	printf("base_server.cpp: nothing to send here..\n");
	return ;
}

void CServerBase::wait()
{
	if(bConnected)
	{
		// printf("base_server.cpp: try to wait for thread!\n");
		// pthread_join
		WaitForSingleObject(thread_to_handle_client, INFINITE);
		WaitForSingleObject(thread_to_send_client, INFINITE);
		// printf("base_server.cpp: thread has exited\n");
	}
}

// listen , only one client is handled 
void CServerBase::listen()
{
	if(bConnected)
	{
		printf("base_server.cpp: already connected, now only handle this client\n");
		return;
	}
	int iResult;
	iResult = ::listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return ;
    }

    // Accept a client socket
    ClientSocket = accept(ListenSocket, NULL, NULL);
    if (ClientSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return ;
    }

    // No longer need server socket
    // closesocket(ListenSocket); // may relisten in the for disconnection
	
	bConnected = true;
	// start thread to handle the client' job
	thread_to_handle_client = CreateThread(0,0,(LPTHREAD_START_ROUTINE)&CServerBase::thread_do_it, \
		(LPVOID)this, 0, &thread_to_handle_client_id);
	if(thread_to_handle_client == NULL)
	{
		printf("base_server.cpp: failed to create thread handle client issue!\n");
	}else
	{
		printf("base_server.cpp: succeed to create thread handle client issue!\n");
	}

	// start thread to send the client's sending part 
	thread_to_send_client = CreateThread(0,0,(LPTHREAD_START_ROUTINE)&CServerBase::thread_send_it, \
		(LPVOID)this, 0, &thread_to_send_client_id);
	if(thread_to_send_client == NULL)
	{
		printf("base_server.cpp: failed to create thread send client issue!\n");
	}else
	{
		printf("base_server.cpp: succeed to create thread send client issue!\n");
	}

	return ;
}

bool CServerBase::sendData(char *sendbuf, unsigned int len) 
{
	bool ret = false; 
	// wchar_t *s = NULL; 

	if(!bConnected)
	{
		printf("base_server.cpp: not connect yet\n");
		return false;
	}
	unsigned int iResult = 0; 
	iResult = send(ClientSocket, sendbuf, len, 0); 
	// Sleep(100);
	Sleep(10);
	if(iResult == SOCKET_ERROR)
	{
		printf("base_server.cpp: failed to send data! iResult = %d with error: %d\n", iResult, WSAGetLastError());
		// printf("send failed with error: %d\n", WSAGetLastError());
	}else if(iResult != len)
	{
		printf("base_server.cpp: failed to send complete data! iResut = %d < len= %d", iResult, len);
	}else 
	{
		// FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, WSAGetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPWSTR)&s, 0, NULL);
		ret = true;
		printf("base_server.cpp: succeed to send data len: %d\n", len);
	}
	return ret;
}

bool CServerBase::recvData(char *recvbuf, unsigned int len)
{
	if(!bConnected)
	{
		printf("base_server.cpp: not connect yet! fail to recvData.");
		return false;
	}

	bool ret = false;
	unsigned int iResult = 0; 
	iResult = recv(ClientSocket,recvbuf, len, 0); 

	if(iResult == SOCKET_ERROR)
	{
		printf("base_server.cpp: failed to recvData iResult = %d with error: %d\n", iResult, WSAGetLastError());
	}else if(iResult != len)
	{
		printf("base_server.cpp: failed to recv complete data, iResult = %d < len = %d\n", iResult, len); 
	}else
	{
		ret = true; 
		printf("base_server.cpp: succeed to recv data len: %d\n", len); 
	}
	return ret;
}

