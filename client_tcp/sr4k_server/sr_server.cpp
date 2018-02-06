#include "stdafx.h"
#include "sr_server.h"
#include "protocolDefines.h"
#include "sr_server/huffman_code.h"
#include "rle_code.h"
#include "SR_writer.h"

CSRServer::CSRServer(string sr_ip,std::string port_id)
: CServerBase(port_id), 
sr_ip_(sr_ip),
b_sr_connected_(false),
b_dump_sr_to_disk_(false)
{
	initSR();
}

CSRServer::~CSRServer(){}

int CSRServer::openSR_impl()
{
	int ret = -1;
#ifdef USE_TCP_CAMERA
	ret = SR_OpenETH(&sr_cam_, sr_ip_.c_str()); 
#else
#ifdef USE_USB_CAMERA 
	ret = SR_OpenUSB(&sr_cam_, 0x40000397); // SN number in the camera
#endif
#endif
	if(ret <= 0)
	{
		return ret;
	}
	SR_SetIntegrationTime(sr_cam_, 80); // 80 60 30 // this parameter set 30hz = 1/4*(4.6+30*0.1) 
	SR_SetMode(sr_cam_, AM_HW_TRIGGER | AM_MEDIAN | AM_COR_FIX_PTRN // | AM_SW_ANF   
		| AM_CONV_GRAY | AM_CONF_MAP | AM_DENOISE_ANF | AM_MEDIANCROSS);
	return ret;
}

bool CSRServer::openSR()
{
	// int ret = SR_OpenETH(&sr_cam_, sr_ip_.c_str()); 
	int ret = openSR_impl();
	if(ret <= 0)
	{
		printf("sr_server.cpp: failed to open SR4000 with ip: %s\n", sr_ip_.c_str());
		return false; 
	}else // sometime the first time is not right 
	{
		// close and open again
		// SR_SetIntegrationTime(sr_cam_, 30); // this parameter set 30hz = 1/4*(4.6+30*0.1) 
		// SR_SetMode(sr_cam_, AM_SW_TRIGGER | AM_MEDIAN | AM_COR_FIX_PTRN | AM_SW_ANF   
		//	| AM_CONV_GRAY | AM_CONF_MAP | AM_DENOISE_ANF | AM_MEDIANCROSS);
		// SR_SetMode(sr_cam_, AM_SW_TRIGGER | AM_MEDIAN );
		SR_Acquire(sr_cam_);
		SR_Close(sr_cam_); 
		// SR_OpenETH(&sr_cam_, sr_ip_.c_str()); 
		openSR_impl();
	}
	return true;
}

void CSRServer::initSR()
{
	b_sr_connected_ = openSR();
	if(b_sr_connected_)
	{
		// SR_SetIntegrationTime(sr_cam_, 30); // this parameter set 30hz = 1/4*(4.6+30*0.1) 
		// SR_SetMode(sr_cam_, AM_SW_TRIGGER | AM_MEDIAN | AM_COR_FIX_PTRN | AM_SW_ANF   
		//	| AM_CONV_GRAY | AM_CONF_MAP | AM_DENOISE_ANF | AM_MEDIANCROSS);
		printf("sr_server.cpp: succeed to open sr_cam, Let' do it!\n");

		// allocate for the buffers 
		// pDis_.resize(SR_SIZE, 0);
		// pInt_.resize(SR_SIZE, 0); 
		// huff protocol: 1. IPRSGetImage + 2. frequency table 256*4, 3. raw len, compress len, + len
		pSend_.resize(sizeof(IPRSGetImage)+ SR_SIZE*(sizeof(char) + 3*sizeof(unsigned short)), 0);  
		pHuff_.resize(sizeof(int)*(256+2)+2*SR_SIZE*(sizeof(char) + 3*sizeof(unsigned short)), 0);	// maximum length
		// pSend_.resize(sizeof(IPRSGetImage)+ SR_SIZE*(3*sizeof(float) + sizeof(unsigned short)), 0); 
		// pSend_.resize(sizeof(IPRSGetImage) + SR_SIZE*2*sizeof(unsigned short), 0);
		pX_.resize(SR_SIZE, 0); 
		pY_.resize(SR_SIZE, 0);
		pZ_.resize(SR_SIZE, 0);
	}

	// initialize square map
	/*int N = 65536; 
	sqrt_map_.resize(N, 0);
	for(int i=0; i<N; i++)
	{
		sqrt_map_[i] = (unsigned char)(sqrt((double)i));
	}*/

	pSRWriter_ = new CSRWriter;
	return ;
}

// sequence : z,x,y,intensity,confindence, to be consistent with the previous data
bool CSRServer::acquireSRDataHuff()
{
	if(!b_sr_connected_)
	{
		printf("sr_server.cpp: camera not connected!\n");
		exit(1);
		return false; 
	}
	SR_Acquire(sr_cam_); 
	IPRSGetImage sx;
	sx._p.id = RS_GetImage; 
	int shift = sizeof(sx);		// this is the size of the tag
	sx._p.size = shift + SR_SIZE*(3*sizeof(float) + sizeof(unsigned short)); // tag + Distance + Intensity

	char* pS = pSend_.data();  // pointer to the send buffer

	// Intensity image 
	ImgEntry* imgEntryArray; 
	int nImg = SR_GetImageList(sr_cam_, &imgEntryArray); 
	WORD* p = (WORD*)imgEntryArray[1].data;
	unsigned char* pI = (unsigned char*)pS + shift + 3*SR_SIZE*sizeof(short);
	if(!map_raw_img_to_grey(p, pI, SR_SIZE))
	{
		printf("sr_server.cpp: the intensity of this frame is invalid!\n");
		return false;
	}

	// int float_s = sizeof(float);
	// sequence : z,x,y,intensity,confindence, to be consistent with the previous data
	// SR_CoordTrfFlt(sr_cam_, pX_.data(), pY_.data(), pZ_.data(), float_s, float_s, float_s); 
	SR_CoordTrfUint16(sr_cam_, pX_.data(), pY_.data(), pZ_.data(), sizeof(short), sizeof(short), sizeof(unsigned short));

	int shift_f = SR_SIZE * sizeof(float); 
	int shift_s = SR_SIZE * sizeof(short);
	memcpy(pS + shift, pZ_.data(), shift_s); // z
	memcpy(pS + shift + shift_s, pX_.data(), shift_s); // x
	memcpy(pS + shift + 2*shift_s, pY_.data(), shift_s); // y

	// copy intensity img to buffer 
	// memcpy(&pInt_[0], p, SR_SIZE*sizeof(unsigned short)); 
	// memcpy(pS + shift + 3*shift_f, p, SR_SIZE*sizeof(unsigned short));

	char* rawStream = pS + shift;
	char* compStream = pHuff_.data();
	int * ftable = (int*)(rawStream);
	int rawLen = SR_SIZE*(sizeof(char) + 3*sizeof(short));
	static CHuffman huff; 
	int frequencies[UniqueSymbols] = {0};
	int compLen = 0; 
	huff.encode(rawStream, rawLen, &compStream, compLen, frequencies);
	// huff protocol: 1. IPRSGetImage + 2. frequency table 256*4, 3. raw len, compress len, + len
	// 2. frequency table 256*4
	for(int i=0; i<UniqueSymbols; i++)
	{
		(*ftable++) = frequencies[i];
	}
	// 3. raw len, compress len, + len
	(*ftable++) = rawLen; 
	(*ftable++) = compLen; 
	char* pComp = (char*)(ftable); 
	memcpy(pComp, compStream, compLen);

	// actually tcp needs to send 
	tcp_send_size_ = compLen + (256+2)*sizeof(int) + shift;

	// 1. add protocol 
	memcpy(pS, &sx, shift);
}

void CSRServer::testSRRLE()
{
	if(!b_sr_connected_)
	{
		printf("sr_server.cpp: camera not connected!\n");
		return ;
	}
	int NUM = 10; 
	for(int j=0; j<NUM; ++j)
	{
		SR_Acquire(sr_cam_); 
		// Intensity image 
		ImgEntry* imgEntryArray; 
		int nImg = SR_GetImageList(sr_cam_, &imgEntryArray); 
		WORD* p = (WORD*)imgEntryArray[1].data;

		memset(pSend_.data(),0,pSend_.size());
		vector<char> pTMP(pSend_.size(),0);	 // tp compare with original string

		int shift = sizeof(IPRSGetImage);
		char* pS = pSend_.data();  // pointer to the send buffer
		unsigned char* pI = (unsigned char*)pS + shift + 3*SR_SIZE*sizeof(short);
		if(!map_raw_img_to_grey(p, pI, SR_SIZE))
		{
			printf("sr_server.cpp: the intensity of this frame is invalid!\n");
			return ;
		}
		SR_CoordTrfUint16(sr_cam_, pX_.data(), pY_.data(), pZ_.data(), sizeof(short), sizeof(short), sizeof(unsigned short));

		int shift_s = SR_SIZE * sizeof(short);
		memcpy(pS + shift, pZ_.data(), shift_s); // z
		memcpy(pS + shift + shift_s, pX_.data(), shift_s); // x
		memcpy(pS + shift + 2*shift_s, pY_.data(), shift_s); // y
		CRLECode rle; 
		char* rawStream = pS + shift;
		char* compStream = pHuff_.data();
		char* decompStream = pTMP.data()+shift;
		int * ftable = (int*)(rawStream);
		int rawLen = SR_SIZE*(sizeof(char) + 3*sizeof(short));
		int compLen = 0; 
		rle.encode(rawStream, rawLen, &compStream, compLen);
		rle.decode(compStream, compLen, &decompStream, rawLen);

		printf("sr_server.cpp: compress ratio = %f\n", (float)compLen/(float)rawLen);
		if(strcmp(pS, pTMP.data()) == 0)
		{
			printf("sr_server.cpp: succeed! two stream equals at frame %d!\n", j);
		}else
		{
			printf("sr_server.cpp: failed! two stream not equal at frame %d!\n", j);
		}
		Sleep(1);
	}
}

void CSRServer::testSRHuffman()
{
	if(!b_sr_connected_)
	{
		printf("sr_server.cpp: camera not connected!\n");
		return ;
	}
	int NUM = 10; 
	for(int j=0; j<NUM; ++j)
	{
		SR_Acquire(sr_cam_); 
		// Intensity image 
		ImgEntry* imgEntryArray; 
		int nImg = SR_GetImageList(sr_cam_, &imgEntryArray); 
		WORD* p = (WORD*)imgEntryArray[1].data;

		memset(pSend_.data(),0,pSend_.size());
		vector<char> pTMP(pSend_.size(),0);	 // tp compare with original string

		int shift = sizeof(IPRSGetImage);
		char* pS = pSend_.data();  // pointer to the send buffer
		unsigned char* pI = (unsigned char*)pS + shift + 3*SR_SIZE*sizeof(short);
		if(!map_raw_img_to_grey(p, pI, SR_SIZE))
		{
			printf("sr_server.cpp: the intensity of this frame is invalid!\n");
			return ;
		}
		SR_CoordTrfUint16(sr_cam_, pX_.data(), pY_.data(), pZ_.data(), sizeof(short), sizeof(short), sizeof(unsigned short));

		int shift_s = SR_SIZE * sizeof(short);
		memcpy(pS + shift, pZ_.data(), shift_s); // z
		memcpy(pS + shift + shift_s, pX_.data(), shift_s); // x
		memcpy(pS + shift + 2*shift_s, pY_.data(), shift_s); // y
		CHuffman huff; 
		int frequencies[UniqueSymbols] = {0};
		char* rawStream = pS + shift;
		char* compStream = pHuff_.data();
		char* decompStream = pTMP.data()+shift;
		int * ftable = (int*)(rawStream);
		int rawLen = SR_SIZE*(sizeof(char) + 3*sizeof(short));
		int compLen = 0; 
		huff.encode(rawStream, rawLen, &compStream, compLen, frequencies);
		huff.decode(compStream, compLen, &decompStream, rawLen, frequencies);

		if(strcmp(pS, pTMP.data()) == 0)
		{
			printf("sr_server.cpp: succeed! two stream equals at frame %d!\n", j);
		}else{
			printf("sr_server.cpp: failed! two stream not equal at frame %d!\n", j);
		}
		Sleep(1);
	}
}

bool CSRServer::fakeData()
{
	int K1 = 1024; 
	int K100 = 100 * K1;
	int K300 = 3 * K100; 
	int K174 = 174 * K1;
	int K200 = 200 * K1;
	int K75 = 75 * K1;

	IPRSGetImage sx;
	sx._p.id = RS_GetImage; 
	int shift = sizeof(sx);		// this is the size of the tag
	sx._p.size = shift + K75; // tag + Distance + Intensity
	if(pSend_.size() < sx._p.size)
		pSend_.resize(sx._p.size, 0);
	
	char* pS = pSend_.data();  // pointer to the send buffer
	// add protocol 
	memcpy(pS, &sx, shift);
	tcp_send_size_ = sx._p.size;
	return true; 
}

// seq: z, x, y, intensity
bool CSRServer::acquireSRData2()
{
	if(!b_sr_connected_)
	{
		printf("sr_server.cpp: camera not connected!\n");
		exit(1);
		return false;
	}
	IPRSGetImage sx;
	sx._p.id = RS_GetImage; 
	int shift = sizeof(sx);		// this is the size of the tag
	int shift_f = SR_SIZE*sizeof(float);
	sx._p.size = shift + SR_SIZE*(3*sizeof(unsigned short)+sizeof(char)); // tag + Distance + Intensity

	SR_Acquire(sr_cam_); // retrieve a frame

	// Intensity image 
	ImgEntry* imgEntryArray; 
	int nImg = SR_GetImageList(sr_cam_, &imgEntryArray); 
	WORD* p = (WORD*)imgEntryArray[1].data;

	char* pS = pSend_.data();  // pointer to the send buffer
	unsigned char* pI = (unsigned char*)pS + shift + 3*SR_SIZE*sizeof(short);
	if(!map_raw_img_to_grey(p, pI, SR_SIZE))
	{
		printf("sr_server.cpp: the intensity of this frame is invalid!\n");
		return false;
	}
	SR_CoordTrfUint16(sr_cam_, pX_.data(), pY_.data(), pZ_.data(), sizeof(short), sizeof(short), sizeof(unsigned short));

	int shift_s = SR_SIZE * sizeof(short);
	memcpy(pS + shift, pZ_.data(), shift_s); // z
	memcpy(pS + shift + shift_s, pX_.data(), shift_s); // x
	memcpy(pS + shift + 2*shift_s, pY_.data(), shift_s); // y
	
	// add protocol 
	memcpy(pS, &sx, shift);

	tcp_send_size_ = pSend_.size();

	// dump into file
	if(b_dump_sr_to_disk_)
	{
		static vector<char> buf(3*SR_SIZE*sizeof(float) + 2*SR_SIZE*sizeof(short)); 
		char* pS = buf.data(); 
		float* pZ = (float*)(pS); 
		float* pX = (float*)(pS + shift_f);
		float* pY = (float*)(pS + 2*shift_f);
		SR_CoordTrfFlt(sr_cam_, pX, pY, pZ, 4, 4, 4);
		memcpy(pS+3*shift_f, p, shift_s); 
		WORD* pC = (WORD*)imgEntryArray[2].data;
		memcpy(pS+3*shift_f + shift_s, pC, shift_s);
		pSRWriter_->write(buf.data(), buf.size());
	}

	return true;
}

bool CSRServer::acquireSRData()
{
	if(!b_sr_connected_)
	{
		printf("sr_server.cpp: camera not connected!\n"); 
		exit(1);
		return false; 
	}

	unsigned int new_size = sizeof(IPRSGetImage) + SR_SIZE*sizeof(unsigned short) + SR_SIZE*sizeof(char); 
	if(pSend_.size() != new_size)
	{
		// pSend_.resize(sizeof(IPRSGetImage) + SR_SIZE*2*sizeof(unsigned short));
		pSend_.resize(new_size);
	}

	DWORD begin, end;
	// begin = GetTickCount();
	SR_Acquire(sr_cam_); 
	// end = GetTickCount();
	// printf("sr_server.cpp: SR_Acquire cost: %d ms\n", end-begin);

	IPRSGetImage sx;
	sx._p.id = RS_GetImage; 
	int shift = sizeof(sx);		// this is the size of the tag
	sx._p.size = new_size; // tag + Intensity + Distance

	char* pS = pSend_.data();  // pointer to the send buffer

	// Intensity image 
	ImgEntry* imgEntryArray; 
	int nImg = SR_GetImageList(sr_cam_, &imgEntryArray); 
	WORD* p = (WORD*)imgEntryArray[1].data;

	// copy intensity img to buffer 
	// memcpy(&pInt_[0], p, SR_SIZE*sizeof(unsigned short)); 
	unsigned char* pI = (unsigned char*)pS + shift;
	// convert intensity img to CV_8UC1
	if(!map_raw_img_to_grey(p, pI, SR_SIZE))
	{
		printf("sr_server.cpp: the intensity of this frame is invalid!\n");
		return false;
	}
	// memcpy(pS + shift, p, SR_SIZE*sizeof(unsigned short));

	// copy distance to buffer 
	// vector<unsigned short> pDis_(SR_SIZE, 0);
	// p = (WORD*)imgEntryArray[0].data;
	// memcpy(&pDis_[0], p, SR_SIZE*sizeof(unsigned short)); 

	// memcpy(pS + shift + SR_SIZE*sizeof(unsigned short), p, SR_SIZE*sizeof(unsigned short));
	SR_CoordTrfUint16(sr_cam_, pX_.data(), pY_.data(), pZ_.data(), sizeof(short), sizeof(short), sizeof(unsigned short));
	memcpy(pS + shift + SR_SIZE*sizeof(char), pZ_.data(), SR_SIZE*sizeof(unsigned short));

	// add protocol 
	memcpy(pS, &sx, shift);
	tcp_send_size_ = pSend_.size();
	return true;
}

bool CSRServer::sendSRDataLoop()
{
	bool ret = true;
	bool bAcquireData;
	bool bSendResult;
	bool bRecvResult;

	IPRSGetImage rxtag; // recv confirm

	IPRSGetImage failtag; 
	failtag._p.id = ZH_ImageCameraFail; 
	failtag._p.size = sizeof(IPRSGetImage);


	DWORD begin_acquire, end_acquire; 
	DWORD begin_send, end_send;

	int step = 2;
	int cnt = 0;
	while(1)
	{
		begin_acquire = GetTickCount();
		bAcquireData = acquireSRData(); // z, intensity, 75 KB
		// bAcquireData = fakeData();
		// bAcquireData = acquireSRDataHuff();
		// bAcquireData = acquireSRData2(); // x, y, z, intensity, 174 KB
		end_acquire = GetTickCount();
		if(!bAcquireData) // failed to acquire data from camera
		{
			//printf("sr_server.cpp: failed to acquire Data, exit loop!\n");
			//bSendResult = sendData((char*)(&failtag), failtag._p.size);
			//if(!bSendResult)
			//{
			//	printf("sr_server.cpp: failed to send fail tag \n");
			//}
			//ret = false;  
			//break;
			printf("sr_server.cpp: failed to acquire Data, try again!\n");
			Sleep(1); // wait for 1ms
			continue;
		}else
		{
			//if(++cnt == step) cnt = 0;
			begin_send = GetTickCount();
			// send data to client 
			// bSendResult = sendData(pSend_.data(), pSend_.size());
			//if(cnt!=0)
			   bSendResult = sendData(pSend_.data(), tcp_send_size_);
			end_send = GetTickCount();
			if(!bSendResult)
			{
				printf("sr_server.cpp: failed to send sr data, exit! \n");
				ret = false;
				break;
			}
			printf("sr_server.cpp: acquiredata %d ms, tcp cost %d ms, all cost: %d ms \n", 
					end_acquire - begin_acquire, end_send - begin_send, end_send - begin_acquire);
			// wait for confirm 
			//bRecvResult = recvData((char*)(&rxtag), sizeof(IPRSGetImage)); 
			//if(!bRecvResult)
			//{
			//	printf("sr_server.cpp: failed to recv sr confirm tag, exit!\n");
			//	ret = false;
			//	break;
			//}
			//if(rxtag._p.id == ZH_ImageConfirm) // get confirm, send next
			//{
			//	printf("sr_server.cpp: succeed to recv confirm tag!\n"); 
			//}else if(rxtag._p.id == ZH_ImageStreamStop) 
			//{
			//	printf("sr_server.cpp: receive stop stream cmd, exit sendSRDataLoop!\n");
			//	break;
			//}else
			//{
			//	printf("sr_server.cpp: receive unexpected tag, something wrong, exit!\n");
			//	ret = false; 
			//	break;
			//}
			Sleep(1); // wait for 1ms
		}	
	}

	return ret;
}

// this is what it will do in its own way, the following
void CSRServer::do_it()
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
				printf("sr_server.cpp: start to send stream!\n"); 
				bLoopStatus = sendSRDataLoop(); // this is the main loop to send data
				if(!bLoopStatus)
				{
					printf("sr_server.cpp: loop failed, something wrong!\n");
					break;
				}
			  }else if(rx._p.id == ZH_ImageStreamStop || rx._p.id == ZH_ImageConfirm)
			  {
				printf("sr_server.cpp: weired, this part should not receive this tag! something wrong!\n");
			  }else if(rx._p.id == ZH_EXIT)
			  {
				printf("sr_server.cpp: server exist!\n");
				break;
			  }else
			  {
				printf("sr_server.cpp: something wrong in the pointer, cannot recognize tag\n");
				break;
			  }
        }
    } while (bResult > 0);
	
	printf("sr_server.cpp: server thread exit!\n");
}


bool CSRServer::map_raw_img_to_grey(unsigned short* pRaw, unsigned char* pGrey, int N)
{
	unsigned short limit_s = 65000; 
	unsigned short* p1 = pRaw; 
	unsigned char* p2 = pGrey; 
	unsigned char max_c = 0; 
	unsigned char tc = 0; 
	static bool first_time = true; 
	static vector<unsigned char> sqrt_map_;
	
	if(first_time)
	{
		int N = 65536; 
		sqrt_map_.resize(N, 0);
		for(int i=0; i<N; i++)
		{
			sqrt_map_[i] = (unsigned char)(sqrt((double)i));
		}
		first_time = false;
	}

	for(int i=0; i<N; i++)
	{
		if(*p1 >= limit_s)
		{
			tc = 0;
		}else
		{
			tc = sqrt_map_[*p1];
		}
		if(tc > max_c){ max_c = tc;}
		*p2 = tc; 
		++p2; ++p1;
	}
	// assert(max_c > 0);
	if(max_c <= 0) 
	{
		printf("sr_server.cpp: no intensity available!\n");
		return false;
	}
	p2 = pGrey; 
	float inv_max = (float)(255./max_c); 
	for(int i=0; i<N; i++)
	{
		*p2 = (unsigned char)((*p2)*inv_max);
		++p2;
	}
	return true;
}