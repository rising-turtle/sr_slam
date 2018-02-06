#ifndef SICK_CLIENT_H
#define SICK_CLIENT_H

#include "socket_client.h"
#include "global_func.h"

#include "NodeHandle.h"
#include "NormalSleep.hpp"

#include <string>
#include <iostream>
#include <fstream>

/*********************************************************************
** Preprocessor
*********************************************************************/

// Could probably do some better and more elaborate checking
// and definition here.
#define HAS_CLOCK_GETTIME (_POSIX_C_SOURCE >= 199309L)

// USE_GOOGLE_TYPE(Home_Odometer)
// USE_GOOGLE_TYPE(Home_Laser)

// class Home_Odometer;
class Home_Laser;

using namespace std;
using namespace suro::platform::core::os;  //使用此命名空间


class CSickClient : public CSocketClient
{
public:
	CSickClient(string path_ = "D:\\client_fusion\\data");
	~CSickClient();
	bool startRecordData();
	void stopRecordData();
	bool startSendDataOffline();
	// void stopSendDataOffline();
protected:
	// string m_path_odo;
	string m_path_laser;
private:
	// HANDLE mtx_odo;
	HANDLE mtx_laser;
	bool sendSICKData(ifstream&);
	void laserCallBack(const Home_Laser& laser);
	// bool sendODOData(ifstream&);
	// void odoCallBack(const Home_Odometer& odo);
};

#endif