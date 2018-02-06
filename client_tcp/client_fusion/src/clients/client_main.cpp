#include <iostream>
#include <string.h>
#include <stdio.h>
#include "socket_client.h"
#include "sick_client.h"
#include "odo_client.h"
#ifdef WIN32
	#include <windows.h>
#else
	#include <pthread.h>
#endif
// #include "xtion_client.h"

using namespace std;

void test_sick_client();

int main(int argc, char* argv[])
{
    // test_socket_client();
    test_sick_client();
    return 0; 
}

void test_sick_client()
{
    // CXtionClient client; 
	// CSocketClient* client = new CODOClient;
	CSocketClient* client = new CSickClient;
	// CSocketClient * client = new CSocketClient;
    client->connect("192.168.1.103", 9015); // laser 9015 
	// client->connect("192.168.1.107", 9016); // ODO 9016 
    // client.connect("192.168.0.4", 9012);
    // client.connect("192.168.0.4", 6060);

    char rbuf[4096];
    memset(rbuf, 0, sizeof(rbuf));
    int cmd  = -1; 
    bool m_close = false;

#ifdef WIN32
	HANDLE thread_record;
	HANDLE thread_send;
	DWORD thread_record_id;
	DWORD thread_send_id;
#else
    pthread_t xtion_thread_record;
    pthread_t xtion_thread_send;
#endif
    while(client->recvData(rbuf, sizeof(rbuf)))
    {
        memcpy(&cmd, rbuf, sizeof(4));
        switch(cmd)
        {
            case 0: 
                cout<<"cmd = 0, OK, let's asy our time!"<<endl;
                unsigned long sec, usec; 
				memcpy(&sec, rbuf +4, sizeof(unsigned long));
				memcpy(&usec, rbuf +4 +sizeof(unsigned long), sizeof(unsigned long));
				g_dwHighDateTime = sec; 
				g_dwLowDateTime = usec;
				//g_has_time_asy = true;
				cout<<"client rece asy time: "<<sec<<"."<<usec<<endl;
                break;
            case 1:
                cout<<"cmd = 1, OK, let's record data to local disk!"<<endl;
				thread_record = CreateThread(0,0, (LPTHREAD_START_ROUTINE)&CSocketClient::thread_start_record, \
					(LPVOID)client,0,&thread_record_id);
				if(thread_record == NULL)
				{
					cout<<"failed to create thread to record data locally!"<<endl;
				}else{
					cout<<"succeed to create thread to record data locally!"<<endl;
				}
				Sleep(10);
                break; 
            case 2:
                cout<<"cmd = 2, OK, let's stop send xtion data!"<<endl;
                client->stopRecordData();
                // client.stopSendXtion();
                break; 
            case 3:
                cout<<"cmd = 3, OK, let's clear all the data in the DIR!"<<endl;
                
                break; 
            case 4:
                cout<<"cmd = 4, OK, let's send all the data to server offline!"<<endl;
				thread_send = CreateThread(0,0, (LPTHREAD_START_ROUTINE)&CSocketClient::thread_start_send, \
							(LPVOID)client,0,&thread_send_id);
				if(thread_send == NULL)
				{
					cout<<"failed to create thread to send data to server!"<<endl;
				}else
				{
					cout<<"succeed to create thread to send data to server!"<<endl;
				}
                Sleep(10);
                break;
            default:
                cout<<"Oh, no, Unknown cmd: "<<cmd<<" let's close"<<endl;
                m_close = true;
                break;
        }
        if(m_close ) break;
		Sleep(1); // sleep 1ms
        // usleep(100);
    }
}
/*
void test_socket_client()
{
    CSocketClient client; 
    client.connect("127.0.0.1", 6060);
    char rbuf[4096]; 
    char sbuf[4096];
    memset(sbuf, 0, sizeof(sbuf)); 
    memset(rbuf, 0, sizeof(rbuf));
    char* sendMsg = "Hello server, now I can only send you This!";
    memcpy(sbuf, sendMsg, strlen(sendMsg));
    int cmd = -1;
    bool bclose = false;
    double timestamp = 0;

    // receive data from server 
    while(client.recvData(rbuf, sizeof(rbuf)))
    {
        memcpy(&cmd, rbuf, 4);
        switch(cmd)
        {
            case 0: 
                memcpy(&timestamp, rbuf+4, sizeof(double));
                cout<<"cmd = 0: time asy! recv timestamp: "<< timestamp<<endl;
                break; 
            case 1:
                cout<<"cmd = 1: begin send data!"<<endl;
                client.sendData(sbuf, strlen(sbuf));
                break; 
            case 2:
                cout<<"cmd = 2: stop send data!"<<endl; 
                break; 
            case 3:
                cout<<"cmd = 3: send all data to server !"<<endl;
                break; 
            default:
                cout<<"unknown cmd = "<<cmd<<" now close connection!"<<endl;
                bclose = true;
                break;
        }
        if(bclose) break;
    }
}
*/