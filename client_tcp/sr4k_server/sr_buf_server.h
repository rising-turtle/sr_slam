/*
   July. 13, 2016 David Z
   add a buffer for sending data 
*/


#ifndef SR_BUF_SERVER_H
#define SR_BUF_SERVER_H

#include "sr_server.h"

class CSRBufServer : public CSRServer
{
public:
	CSRBufServer(string sr_ip = "192.168.1.4",std::string port_id = DEFAULT_PORT); 
	virtual ~CSRBufServer();

	void init();

	// main loop for handling data 
	virtual void do_it(); 
	bool recvSRDataLoop();

	// main loop for sending data 
	virtual void send_it(); 

	bool b_send_thread_ok_; 
	bool b_recv_thread_ok_;

	bool b_send_buf_empty_;

	vector< vector<char> > buf_to_send_;
	vector<int> buf_timestamp_; 
	vector< vector<char> > buf_to_get_;

	int send_speed_;	// approximately x ms/frame

	HANDLE bufMutex_;

};


#endif