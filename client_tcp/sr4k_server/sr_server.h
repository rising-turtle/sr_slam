/*
 May 12, 2015, David Z
	a server class to transmit SR-4000 data 
*/

#ifndef SR_SERVER_H
#define SR_SERVER_H

#include "base_server.h"
#include "libMesaSR.h"
#include <string>
#include <vector>

class CSRWriter; 
#pragma comment( lib, "libMesaSR" )
using namespace std;

// #define USE_TCP_CAMERA		// use TCP interface camera 
#define USE_USB_CAMERA			// use USB interface camera

class CSRServer : public CServerBase
{
public:
	enum SR_PARAM{
		SR_ROWS = 144, 
		SR_COLS = 176,
		SR_SIZE = 144*176
	};
public:
	CSRServer(string sr_ip = "192.168.1.4",std::string port_id = DEFAULT_PORT); 
	virtual ~CSRServer();

	// main loop for transmitting data 
	virtual void do_it(); 

	// parameters for sr camera
	SRCAM sr_cam_;
	bool openSR();				// open camera
	int  openSR_impl();			// implementation for open camera
	void initSR();				// init sr camera 
	volatile bool b_sr_connected_;  // weather the camera is connetced 
	string sr_ip_;

	// buffers for SR data 
	bool acquireSRData();		  // seq: insensity, z, get a frame and save it into the buffer
	bool acquireSRData2();		  // seq: z, x, y, intensity
	bool sendSRDataLoop();		  // send SR data 
	bool acquireSRDataHuff();	  // use huffman encode
	// vector<unsigned short> pDis_; // distance or Z vector 
	// vector<unsigned short> pInt_; // intensity img 
	//vector< float> pX_;		// camera data 
	//vector< float> pY_; 
	//vector< float> pZ_; 

	 bool fakeData(); // fake data 
	 vector< short> pX_;		// camera data 
	 vector< short> pY_; 
	 vector<unsigned short> pZ_; 
	 vector<char> pSend_;		  // tcp send buffer 
	 vector<char> pHuff_;		  // huffman encode buffer
	 int tcp_send_size_ ;		  // actual number of BYTES to send

	 // save the required SR data
	 CSRWriter * pSRWriter_; 
	 bool b_dump_sr_to_disk_;	

public:
	 void testSRRLE();			  // test RLE encode for SR data
	 void testSRHuffman();		  // test huffman encode for SR data
	 // vector<unsigned char> sqrt_map_;
	 bool map_raw_img_to_grey(unsigned short* pRaw, unsigned char* pGrey, int N);


};

#endif