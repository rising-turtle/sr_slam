#include "stdafx.h"
#include "sr_server.h"
#include "sr_buf_server.h"
#include "SR_writer.h"
#include "sr_server/huffman_code.h"
#include "rle_code.h"

// test tcp comm
void testTCP(int argc, char* argv[]); 

char* param_help = "sr_server.exe (SRCAM) ip_addr (TCP)port_id";

void testAcquire(); 

void testSRS(const char* f=""); // test read from .srs stream file, and then extract and save frames 
void testHuffman();
void testSRHuffman(); 
void testRLE();
void testSRRLE();

void testLocalSave(int argc, char* argv[]); // save sr_frames locally

string srs_file_name =  "frames_on_ob2.srs"; //"frames_creep_ob1.srs";
void print_args(int argc, char* argv[]);

// cmd sr_server sr_ip tcp_port
int main(int argc, char* argv[])
{
	printf("main.cpp: %s\n", param_help);
	// testRLE();
	// testSRRLE();
	// testHuffman();
	// testSRHuffman();
	
	 print_args(argc, argv);
	 testTCP(argc, argv); // this is the main function to transmit SR data through TCP 
	
	// testLocalSave(argc, argv);
	// testAcquire();
	if(argc >=2)
	{
		srs_file_name = argv[1];
	}
	// testSRS(srs_file_name.c_str());
	return 0; 
}

void testLocalSave(int argc, char* argv[])
{
	// open SR camera first 
		int ret = -1;
		SRCAM sr_cam_;
#ifdef USE_TCP_CAMERA
	ret = SR_OpenETH(&sr_cam_, sr_ip_.c_str()); 
#else
#ifdef USE_USB_CAMERA 
	ret = SR_OpenUSB(&sr_cam_, 0x40000397); // SN number in the camera
#endif
#endif
	if(ret <= 0)
	{
		printf("main.cpp: failed to open SR\n");
		return ;
	}
	SR_SetIntegrationTime(sr_cam_, 80); // 80 60 30 // this parameter set 30hz = 1/4*(4.6+30*0.1) 
	SR_SetMode(sr_cam_, AM_HW_TRIGGER | AM_MEDIAN | AM_COR_FIX_PTRN // | AM_SW_ANF   
		| AM_CONV_GRAY | AM_CONF_MAP | AM_DENOISE_ANF | AM_MEDIANCROSS);
	
	CSRWriter* pSRWriter = new CSRWriter; 
	if(argc >= 2) pSRWriter->setDir(argv[1]);
	
	int SR_SIZE = 176*144;
	unsigned int new_size = SR_SIZE*sizeof(unsigned short) + SR_SIZE*sizeof(char); 
	vector<char> pSend_(new_size);
	vector< short> pX_(SR_SIZE);		// camera data 
	vector< short> pY_(SR_SIZE); 
	vector<unsigned short> pZ_(SR_SIZE); 
	int cnt = 0;
	while(1)
	{
		SR_Acquire(sr_cam_);
		// Intensity image 
		ImgEntry* imgEntryArray; 
		int nImg = SR_GetImageList(sr_cam_, &imgEntryArray); 
		WORD* p = (WORD*)imgEntryArray[1].data;
		char* pS = pSend_.data();  // pointer to the send buffer
		unsigned char* pI = (unsigned char*)pS;

		if(!((CSRServer*)0)->map_raw_img_to_grey(p, pI, SR_SIZE))
		{
			printf("main.cpp: the intensity of this frame is invalid!\n");
			return ;
		}
		// memcpy(pS + shift + SR_SIZE*sizeof(unsigned short), p, SR_SIZE*sizeof(unsigned short));
		SR_CoordTrfUint16(sr_cam_, pX_.data(), pY_.data(), pZ_.data(), sizeof(short), sizeof(short), sizeof(unsigned short));
		memcpy(pS + SR_SIZE*sizeof(char), pZ_.data(), SR_SIZE*sizeof(unsigned short));

		pSRWriter->write(pSend_.data(), pSend_.size());
		printf("main.cpp: write frame %d \n", ++cnt);
		Sleep(1); // wait for 1ms
	}
	SR_Close(sr_cam_); 
	printf("main.cpp: finish save local sr_frames!\n");
	return ;
}

void testSRRLE()
{
	CSRServer sr_sv;
	sr_sv.testSRRLE();
}

void testRLE()
{
	CRLECode rle;
	char* sampleStr = "In making our selection, we tried to avoid the perennial trends areas"; 
	// const char* sampleStr = "In";
	int len = strlen(sampleStr) + 1; 
	char* encoded = 0; 
	int encoded_len; 
	
	rle.encode(sampleStr, len, &encoded, encoded_len); 
	char * decoded = 0; 
	rle.decode(encoded, encoded_len, &decoded, len); 
	printf("main.cpp: sampleStr: %s\n after decod:e %s\n", sampleStr, decoded);
	printf("main.cpp: sample len = %d, compression = %d, ratio = %f\n", len, encoded_len, (float)encoded_len/(float)len);
	return ;
}

void testSRHuffman()
{
	CSRServer sr_sv;
	sr_sv.testSRHuffman();
}

void testHuffman()
{
	CHuffman huff;
	char* sampleStr = "In making our selection, we tried to avoid the perennial trends areas"; 
	// const char* sampleStr = "In";
	int len = strlen(sampleStr) + 1; 
	char* encoded = 0; 
	int encoded_len; 
	int frequencies[UniqueSymbols] = {0}; 
	huff.encode(sampleStr, len, &encoded, encoded_len, frequencies); 
	char * decoded = 0; 
	huff.decode(encoded, encoded_len, &decoded, len, frequencies); 
	printf("main.cpp: sampleStr: %s\n after decod:e %s\n", sampleStr, decoded);
	printf("main.cpp: sample len = %d, compression = %d, ratio = %f\n", len, encoded_len, (float)encoded_len/(float)len);
	return ;
}

void testSRS(const char* file_srs)
{
	SRCAM sr_cam_; 
	int ret = SR_OpenFile(&sr_cam_, file_srs);
	// int ret = SR_OpenFile(&sr_cam_, "D:\\work\\sr_tcp_module\\sr4k_server_zh\\sr_server\\frames_creep_ob1.srs"); 
	if(ret <= 0)
	{
		printf("main.cpp: failed to load .SRS : %s\n", file_srs); 
		return; 
	}
	printf("main.cpp: succeed to load .SRS: %s\n", file_srs);
	CSRWriter * pW = new CSRWriter; 
	pW->setDir("D:\\work\\data\\SLAM\\SR4000\\imu_fuse");
	
	// set up buffer, sequence: Z, X, Y, Intensity, Confidence
	int Width = 176; 
	int Height = 144;
	int N = Width * Height; 
	int Byte_per_Pixel = 2*1 + 4*3; // 2: intensity, confidence 4: x,y,z, 
	int OFFSET_S = N * sizeof(unsigned short); 
	int OFFSET_F = N * sizeof(float); 
	int TOTAL_SIZE = Byte_per_Pixel * N;
	int float_s = sizeof(float);
	char* buffer = new char[TOTAL_SIZE]; 
	char* pS = buffer; 
	float* pZ = (float*)buffer; 
	float* pX = (float*)(pS + OFFSET_F); 
	float* pY = (float*)(pS + OFFSET_F*2); 
	unsigned short* pI = (unsigned short*)(pS + OFFSET_F*3);
	unsigned short* pC = (unsigned short*)(pS + OFFSET_F*3 + OFFSET_S);
	// unsigned short* pDepth = (unsigned short*)(pS + OFFSET_F*4 + OFFSET_S);

	int n_frames = 0;
	for(;;)
	{
		if(SR_Acquire(sr_cam_) < 0)
		{
			printf("main.cpp: failed to acquire a new data! return. \n");
			break; 
		}
		
		ImgEntry* imgEntryArray; 
		int nImg = SR_GetImageList(sr_cam_, &imgEntryArray); 
		// sequence : z,x,y,intensity,confindence, to be consistent with the previous data
		SR_CoordTrfFlt(sr_cam_, pX, pY, pZ, float_s, float_s, float_s); 
		
		// intensity image 
		unsigned short * pIntensity = (unsigned short*)imgEntryArray[1].data; 
		memcpy(pI, pIntensity, OFFSET_S); 

		// confidence image 
		// unsigned short * pConfidence = (unsigned short*)imgEntryArray[2].data; 
		// memcpy(pC, pConfidence, OFFSET_S); 

		pW->write(buffer, TOTAL_SIZE);
		Sleep(1);
		printf("main.cpp: successfully write %d frames\n", n_frames++);
	}
	
	delete []buffer; 
}

void testAcquire()
{
	SRCAM sr_cam_;
	int ret = SR_OpenUSB(&sr_cam_, 0x40000397); // SN number in the camera
	if(ret <= 0)
	{
		printf("main.cpp: failed to open SRCAM!\n");
		return ;
	}
	SR_SetIntegrationTime(sr_cam_, 30); // this parameter set 30hz = 1/4*(4.6+30*0.1)
	// SR_SetMode(sr_cam_, AM_HW_TRIGGER | AM_MEDIAN);
	SR_SetMode(sr_cam_, AM_HW_TRIGGER | AM_MEDIAN | AM_COR_FIX_PTRN  // | AM_SW_ANF   
		| AM_CONV_GRAY | AM_CONF_MAP | AM_DENOISE_ANF | AM_MEDIANCROSS);
	
	DWORD begin, end;
	for(;;)
	{
		begin = GetTickCount();
		SR_Acquire(sr_cam_); 
		end = GetTickCount();
		printf("main.cpp: SR_Acquire cost: %d ms\n", end-begin);
		Sleep(1);
	}
	return ;
}

void testTCP(int argc, char* argv[])
{
	string sr_ip; 
	string port_id;
	CSRServer* pServer;
#ifdef USE_TCP_CAMERA
	if(argc == 3) // define ip and port
	{
		sr_ip = argv[1]; 
		port_id = argv[2];
		pServer = new CSRServer(sr_ip, port_id);
	}else if(argc == 2)
	{	
		sr_ip = argv[1]; 
		pServer = new CSRServer(sr_ip);
	}else if(argc == 4)
	{
		// TODO: add save file 
		
	}
	else
	{
		pServer = new CSRServer();
	}
#endif
#ifdef USE_USB_CAMERA
	// pServer = new CSRServer();
	pServer = new CSRBufServer();
	if(argc == 2)
	{
		printf("main.cpp: dump to disk at %s\n", argv[1]);
		pServer->b_dump_sr_to_disk_ = true;
		pServer->pSRWriter_->setDir(argv[1]);
	}
	else
	{
		// pServer->b_dump_sr_to_disk_ = true;
		// pServer->pSRWriter_->setDir(argv[1]);
		// pServer = new CSRServer();
	}
#endif
	printf("main.cpp: begin to listen to client...\n"); 	
	pServer->listen();
	printf("main.cpp: begin to wait for thread... \n");
	pServer->wait();
	printf("main.cpp: thread exist, let' exist\n");

	delete pServer; 
	return ;
}

void print_args(int argc, char* argv[])
{
	for(int i=0; i<argc; i++)
	{
		printf("argc %d : %s \n", i, argv[i]);
	}
}