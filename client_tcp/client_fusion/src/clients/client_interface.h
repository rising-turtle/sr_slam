#ifndef CLIENT_INTERFACE_H
#define CLIENT_INTERFACE_H

#include "socket_client.h"
#include "sick_client.h"
#include "odo_client.h"

#include <vector>
#include <string>

using namespace std;

class CClientInterface : public CSocketClient
{
public:
	CClientInterface(string configf = "");
	~CClientInterface();
	void init(string configf);
	void uninit();
	void run();
public:
	vector<CSocketClient*> m_client_impls;
	vector<string> m_ip_impls;
	vector<int> m_ports_impls;
};

#endif