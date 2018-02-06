#include "sick_client.h"
#include <sstream>
#include <string.h>

// #include "HomeMessages/messages_@home_zjupanda_odometer.pb.h"
#include "HomeMessages/messages_@home_zjupanda_laser.pb.h"
#include "signal_exit.h"

using namespace suro::platform::core::os;  //使用此命名空间

// USE_GOOGLE_TYPE(Home_Odometer)
USE_GOOGLE_TYPE(Home_Laser)

CSickClient::CSickClient(string path_ )
{
	// m_path_odo = path_ + string("\\odo.txt");
	m_path_laser = path_ + string("\\laser.txt");
	// NODE.init("sub_test");
	// boost::shared_ptr<Subscriber<Home_Odometer> >  subOdoPtr = NODE.subscribe<Home_Odometer>("home_odometer",boost::bind(&CSickClient::odoCallBack,this,_1));
	// boost::shared_ptr<Subscriber<Home_Laser> >  subLaserPtr = NODE.subscribe<Home_Laser>("home_laser",boost::bind(&CSickClient::laserCallBack,this,_1));
	// mtx_odo = CreateMutex(0, FALSE, 0);
	mtx_laser = CreateMutex(0, FALSE, 0);
}

CSickClient::~CSickClient()
{

}

bool CSickClient::startRecordData()
{
	if(m_bRecord)
	{
		cout<<"sick_client.cpp: already listen to SICK data!"<<endl;
		return true;
	}
	cout<<"sick_client.cpp: start record SICK data!"<<endl;
	// start up bat or file
	NODE.init("sick_client");
	// boost::shared_ptr<Subscriber<Home_Odometer> >  subOdoPtr = NODE.subscribe<Home_Odometer>("home_odometer",boost::bind(&CSickClient::odoCallBack,this,_1));
	boost::shared_ptr<Subscriber<Home_Laser> >  subLaserPtr = NODE.subscribe<Home_Laser>("home_laser",boost::bind(&CSickClient::laserCallBack,this,_1));
	m_bRecord = true;
	NODE.spin();
	return true;
}
void CSickClient::stopRecordData()
{
	// TODO: stop NODE spin?
	cout<<"sick_client.cpp: set m_bRecord = false"<<endl;
	m_bRecord = false;	
}
bool CSickClient::startSendDataOffline()
{
	// TODO: read file from disk and send it to server!
	bool ret = false;
	m_bSendOffline = true;
	ifstream sick_inf(m_path_laser.c_str());
	if(sick_inf.is_open())
		ret = sendSICKData(sick_inf) ;
	else
		cout<<"sick_client.cpp: failed to send data from: "<<m_path_laser<<endl;
	// if(odo_inf.is_open())
		// ret = sendODOData(odo_inf) | ret;
	return ret;
}


bool CSickClient::sendSICKData(ifstream& in)
{
	static const int sick_num = 541;
	static const int sick_size = (sick_num+1)*8;
	char line[4096*2];

	double sick_data[sick_num + 1]; // 541 value + 1 timestamp
	char *sbuf  = new char[sick_size]; // sick data 
	char seps[] = " ,";
	
	while(m_bSendOffline)
	{
		if(in.getline(line, sizeof(line)))
		{
			char* token = NULL;
			char* next_token = NULL;
			token = strtok_s(line, seps, &next_token);
			if(token == NULL) continue; 
			sick_data[0] = atof(token);
			bool is_err = false;
			for(int i=0; i<sick_num; i++)
			{
				token = strtok_s(NULL, seps, &next_token);
				if(token == NULL) 
				{
					is_err = true; 
					break;
				}
				sick_data[1+i] = atof(token);
			}
			if(is_err) continue;
			char * pbuf = (char*)(sick_data);
			char * tbuf = sbuf;
			// sbuf[0] = 'l';  // tag for ladar
			memcpy(tbuf, pbuf, sick_size); // data for sick
			// for debug
			if(sendData(sbuf, sick_size))
			{
				cout<<"sick_client.cpp: succeed send data!"<<endl;
			}else{
				cout<<"sick_client.cpp: failed to send data!"<<endl;
			}
		}else{
			break;
		}
	}
	delete []sbuf;
	m_bSendOffline = false;
	return true;
}


void CSickClient::laserCallBack(const Home_Laser& laser)
{
	static ofstream outf_laser(m_path_laser.c_str());
	if(!outf_laser.is_open())
	{
		cout<<"sick_client.cpp: failed to open record file: "<<m_path_laser<<endl;
		Sleep(100);
		return ;
	}else{
		// static int ncout = 0;
		// ofstream test_bat("test_bat.txt");
		// cout<<"sick_client.cpp: succeed to record "<<++ncout<<" data."<<endl;
	}
	double t = getRosTime();
	
	if(m_bRecord && laser.IsInitialized())
	{
		printf("Laser REC: cycle=%d, num=%d \n", laser.laser_range().cycle(), laser.laser_range().laser_data_num());

		//LOGS_DEBUG("Laser")<<"@Laser call back : "<<laser.laser_range().laser_data_num()<<", "<< laser.laser_range().c_angle_min()<<", "<< laser.laser_range().c_angle_max()<<", "<< laser.laser_range().c_angle_step();
		//save: timestamp dist1-361


		DWORD dwWaitTime=20, dwWaitResult;
		dwWaitResult = WaitForSingleObject(
			mtx_laser,
			INFINITE);
		switch (dwWaitResult)
		{
		case WAIT_OBJECT_0:
			//__try{
				//write to data
				outf_laser<<std::fixed<<std::setprecision(6)<<t;
				for(unsigned int i=0;i<laser.laser_range().laser_data_num();i++)
				{
					outf_laser<<" "<< laser.laser_data(i).dist();
				}
				outf_laser<<endl;
			//}
			//__finally{
				//release
				if(!ReleaseMutex(mtx_laser))
				{
					//error
				}
			//}			
			break;
		case WAIT_ABANDONED:
			return ;

		}

	}else{
		// cout<<"sick_client.cpp: failed to initialize laser!"<<endl;
	}
}