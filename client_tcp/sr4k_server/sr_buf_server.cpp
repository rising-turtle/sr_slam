#include "stdafx.h"
#include "sr_buf_server.h"
#include "protocolDefines.h"
#include "SR_writer.h"

CSRBufServer::CSRBufServer(string sr_ip,std::string port_id)
: CSRServer(sr_ip, port_id)
{
	init();
}

CSRBufServer::~CSRBufServer(){}


void CSRBufServer::init()
{
	int rn = 1024; 
	buf_to_send_.reserve(rn); 
	buf_to_get_.reserve(rn); 
	buf_timestamp_.reserve(rn);
	bufMutex_ = CreateMutex(NULL, FALSE, NULL);
	b_send_thread_ok_ = true; 
	b_recv_thread_ok_ = true;
	b_send_buf_empty_ = true;
}

// this is what it will do in its own way, the following
void CSRBufServer::do_it()
{
	bool bResult;
	bool bLoopStatus;
	IPRSGetImage rx;
    char* recvbuf = (char*)(&rx);
    int recvbuflen = sizeof(rx);

	int count = 0; 
    do {
		  // iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
		  bResult = recvData(recvbuf, recvbuflen);
		  if (bResult) 
		  {
			  if(rx._p.id == ZH_ImageStreamStart)
			  {
				printf("sr_buf_server.cpp: start to acquire stream!\n"); 
				// bLoopStatus = sendSRDataLoop(); // this is the main loop to send data
				bLoopStatus = recvSRDataLoop(); 
				if(!bLoopStatus)
				{
					printf("sr_server.cpp: loop failed, something wrong!\n");
					break;
				}
			  }else if(rx._p.id == ZH_ImageStreamStop || rx._p.id == ZH_ImageConfirm)
			  {
				printf("sr_buf_server.cpp: weired, this part should not receive this tag! something wrong!\n");
			  }else if(rx._p.id == ZH_EXIT)
			  {
				printf("sr_buf_server.cpp: server exist!\n");
				break;
			  }else
			  {
				printf("sr_buf_server.cpp: something wrong in the pointer, cannot recognize tag\n");
				break;
			  }
        }
    } while (bResult > 0);
	
	printf("sr_buf_server.cpp: server thread exit!\n");
}


bool CSRBufServer::recvSRDataLoop()
{
	bool ret = true;
	bool bAcquireData;

	IPRSGetImage rxtag; // recv confirm

	IPRSGetImage failtag; 
	failtag._p.id = ZH_ImageCameraFail; 
	failtag._p.size = sizeof(IPRSGetImage);

	DWORD begin_acquire, end_acquire; 
	DWORD begin_send, end_send;

	int failed_n = 20;
	int step = 2;
	int cnt = 0;
	int failed_cnt = 0; 
	int last_acquire_time = 0;
	while(b_send_thread_ok_ && b_recv_thread_ok_) // thread send work well 
	{
		begin_acquire = GetTickCount();
		bAcquireData = acquireSRData(); // z, intensity, 75 KB
		end_acquire = GetTickCount();
		if(!bAcquireData) // failed to acquire data from camera
		{
			printf("sr_buf_server.cpp: failed to acquire Data, try again!\n");
			Sleep(1); // wait for 1ms
			if(++failed_cnt > failed_n)
			{
				b_recv_thread_ok_ = false;
				printf("sr_buf_server.cpp: continuous %d failed, exist\n", failed_n);
			}
			continue;
		}else
		{
			failed_cnt = 0;
			bool add_this_one = false; 
			if(buf_to_get_.size() <= 0)
				add_this_one = true;
			else
			{
				if(end_acquire - last_acquire_time > send_speed_)
				{
					add_this_one = true;
				}
			}
			if(add_this_one)
			{
				buf_to_get_.push_back(pSend_);
				buf_timestamp_.push_back(end_acquire);
				last_acquire_time = end_acquire;
				if(b_dump_sr_to_disk_)
				{
					int shift = sizeof(IPRSGetImage); // get rid of the protocal header
					pSRWriter_->write(pSend_.data()+shift, pSend_.size()-shift);
				}
			}
			WaitForSingleObject(bufMutex_, INFINITE);
			if(b_send_buf_empty_)
			{
				buf_to_get_.swap(buf_to_send_);
				buf_timestamp_.clear();
				b_send_buf_empty_ = false;
			}
			ReleaseMutex(bufMutex_);
			Sleep(1); // wait for 1ms
		}	
	}

	return ret;

}

void CSRBufServer::send_it()
{
	bool ret = true;
	bool bSendResult;
	DWORD begin_send, end_send;
	int cnt = 0;

	while(b_send_thread_ok_ && b_recv_thread_ok_)
	{
		if(b_send_buf_empty_)
		{
			Sleep(1);
		}else
		{
			b_send_buf_empty_ = false;
			int total_time = 0; 
			int t_cnt = buf_to_send_.size();

			for(int i=0; i<t_cnt; i++)
			{
				begin_send = GetTickCount();
				bSendResult = sendData(buf_to_send_[i].data(), tcp_send_size_);
				end_send = GetTickCount();
				if(!bSendResult)
				{
					printf("sr_buf_server.cpp: failed to send sr data, exit! \n");
					ret = false;
					b_send_thread_ok_ = false;
					continue;
				}
				printf("sr_buf_server.cpp: send %d tcp cost %d ms\n", 
					++cnt , end_send - begin_send);
				total_time = total_time + end_send - begin_send;
			}

			send_speed_ = total_time/t_cnt;
			if(send_speed_ < 30) send_speed_ = 30;
			buf_to_send_.clear();
			b_send_buf_empty_ = true;
		}
	}
}