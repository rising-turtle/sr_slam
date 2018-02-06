#include "odo_client.h"
#include <string>
#include <fstream>

#include "HomeMessages/messages_@home_zjupanda_odometer.pb.h"

using namespace suro::platform::core::os;  //使用此命名空间

USE_GOOGLE_TYPE(Home_Odometer)

CODOClient::CODOClient(string path)
{
	m_path_odo = path + string("\\odo.txt");
	mtx_odo = CreateMutex(0, FALSE, 0);
}
CODOClient::~CODOClient(){}


bool CODOClient::startRecordData()
{
	if(m_bRecord)
	{
		cout<<"odo_client.cpp: already listen to ODO data!"<<endl;
		return true;
	}
	m_bRecord = true;
	cout<<"odo_client.cpp: start record ODO data!"<<endl;
	// start up bat or file
	NODE.init("sub_test");
	boost::shared_ptr<Subscriber<Home_Odometer> >  subOdoPtr = NODE.subscribe<Home_Odometer>("home_odometer",boost::bind(&CODOClient::odoCallBack,this,_1));
	NODE.spin();
	return true;
}

void CODOClient::stopRecordData()
{
	m_bRecord = false;
}

bool CODOClient::startSendDataOffline()
{
	bool ret = false; 
	ifstream odo_inf(m_path_odo.c_str());
	m_bSendOffline = true;
	if(odo_inf.is_open())
	{
		ret = sendODOData(odo_inf);
	}
	return ret;
} 

bool CODOClient::sendODOData(ifstream& in)
{
	static const int odo_num = 3;
	static const int odo_size = (odo_num+1)*8; 
	char line[4096];

	double odo_data[odo_num+1]; // 3 value + 1 timestamp
	char* sbuf = new char[odo_size + 1];
	char sper[] = " ,";

	while(m_bSendOffline)
	{
		if(in.getline(line, sizeof(line)))
		{
			char* token = NULL;
			char* next_token = NULL;
			token = strtok_s(line, sper, &next_token);
			if(token == NULL) continue; 
			odo_data[0] = atof(token);
			bool is_err = false;
			for(int i=0; i<odo_num; i++)
			{
				token = strtok_s(NULL, sper, &next_token);
				if(token == NULL)
				{
					is_err  = true;
					break; 
				}
				odo_data[1+i] = atof(token);
			}
			if(is_err) continue ;
			char* pbuf = (char*)(odo_data);
			char* tbuf = sbuf;
			memcpy(tbuf, pbuf, odo_size);
			if(sendData(sbuf, odo_size))
			{
				cout<<"sick_client.cpp: succeed to send odo data!"<<endl;
			}else{
				cout<<"sick_client.cpp: failed to send odo data!"<<endl;
			}
		}else{
			break;
		}
	}
	delete []sbuf;
	m_bSendOffline = false;
	return true;
}

void CODOClient::odoCallBack(const Home_Odometer& odo)
{
	static ofstream outf_odo(m_path_odo.c_str());
	if(!outf_odo.is_open())
	{
		cout<<"sick_client.cpp: failed to open record file: "<<m_path_odo<<endl;
		Sleep(100);
		return ;
	}
	double t = getRosTime();
	if(m_bRecord && odo.IsInitialized())
	{
		printf("ODO REC cycle=%d, <xyz> = <%f, %f, %f>\n", odo.cycle(),odo.x(),odo.y(),odo.angle());

		//LOGS_DEBUG("Odo")<<"@Odo call back : " << odo.cycle() <<", "<< odo.x()<<", "<< odo.y()<<", "<< odo.angle();
		//save: timestamp x y theta		

		DWORD dwWaitTime=20, dwWaitResult;
		dwWaitResult = WaitForSingleObject(
			mtx_odo,
			INFINITE);
		switch (dwWaitResult)
		{
		case WAIT_OBJECT_0:
			//__try{
			//write to data
			outf_odo<<std::fixed<<std::setprecision(6)<<t<<" "<<odo.x()<<" "<<odo.y()<<" "<<odo.angle()<<endl;
			//}
			//__finally{
			//release
			if(!ReleaseMutex(mtx_odo))
			{
				//error
			}				
			// }
			break;
		case WAIT_ABANDONED:
			return ;

		}
	}
}

