#ifndef ODO_CLIENT_H
#define ODO_CLIENT_H

#include "socket_client.h"
#include "global_func.h"

#include "NodeHandle.h"

// Could probably do some better and more elaborate checking
// and definition here.
#define HAS_CLOCK_GETTIME (_POSIX_C_SOURCE >= 199309L)

class Home_Odometer;

using namespace std;
using namespace suro::platform::core::os;  //使用此命名空间

class CODOClient : public CSocketClient
{
public :
	CODOClient(string path = "D:\\client_fusion\\data");
	~CODOClient();
	bool startRecordData();
	void stopRecordData();
	bool startSendDataOffline();
protected:
	string m_path_odo;
private:
	HANDLE mtx_odo;
	bool sendODOData(ifstream&);
	void odoCallBack(const Home_Odometer& odo);
};

#endif