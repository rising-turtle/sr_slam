#include "client_interface.h"

CClientInterface::CClientInterface(string configf )
{
	init(configf);
}

CClientInterface::~CClientInterface(){}
void CClientInterface::uninit()
{
	for(int i=0; i<m_client_impls.size();i++)
	{
		// 1 disconnect
		// 2 delete 
		delete m_client_impls[i]; 
	}
	m_client_impls.clear();
}

void CClientInterface::init(string configf)
{
	if(configf !="")
	{
		//TODO: config from file
		
	}else
	{
		// 1 SICK client 
		CSocketClient * pSICK = new CSickClient;
		m_client_impls.push_back(pSICK);
		// 2 ODO client 
		CODOClient * pODO = new CODOClient;
		m_client_impls.push_back(pODO);
		// 3 TODO add Xtion client
	}
	
}

void CClientInterface::run()
{
	
}