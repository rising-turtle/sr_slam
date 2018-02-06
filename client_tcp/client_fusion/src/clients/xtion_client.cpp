#include "xtion_client.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cvwimage.h"

void* CXtionClient::thread_start_record2local(void* param)
{
    cout<<"xtion_client.cpp: start thread record!"<<endl;
    CXtionClient* pXtion = static_cast<CXtionClient*>(param);
    pXtion->recordXtion();
}

void* CXtionClient::thread_start_send2server(void* param)
{
    CXtionClient* pXtion = static_cast<CXtionClient*>(param);
    // pXtion->sendXtion();
    pXtion->sendXtionOffline();
}

CXtionClient::CXtionClient(string path_):
m_bXtionReady(false),
m_bStopSending(false),
m_bStopRecording(false),
m_savePath(path_)
{
    m_bXtionReady = m_openni.init();
}

CXtionClient::~CXtionClient()
{
    m_openni.close();
}


bool CXtionClient::recordXtion()
{
    if(!m_bXtionReady)
    {
        cout<<"xtion_client.cpp: xtion is not ready yet!"<<endl;
        return false;
    }
    if(m_bStopRecording)
    {
        cout<<"xtion_client.cpp: xtion is already recording!"<<endl;
        return true;
    }
    m_bStopRecording = false;
    
    cout<<"xtion_client.cpp: start to record xtion data!"<<endl;
    while(!m_bStopRecording)
    {
        cv::Mat rgb; 
        cv::Mat depth;
        if(m_openni.getFrame(rgb, depth))
        {
            m_rgb = rgb; 
            m_dpt = depth ;
            m_timestamp = m_openni.getRosTime();
            // cout<<"depth size: "<<depth.rows<<" * "<<depth.cols<<endl;
            // cout<<"m_dpt size: "<<m_dpt.rows<<" * "<<m_dpt.cols<<endl;
            if(!recordXtion2File())
            {
                cout<<"xtion_client.cpp: failed to recordXtion2File()!"<<endl;
                break; 
            }
            usleep(10);
        }
    }   
}

bool CXtionClient::sendXtionOffline()
{
    string input_f = m_savePath + string("/rgbd_assoc.txt");
    ifstream inf(input_f.c_str());
    if(!inf.is_open())
    {
        cout<<"xtion_client.cpp: failed to load file: "<<input_f<<endl;
        return false;
    }
    char line[4096];
    char fname[256];
    double t;
    string dptName;
    string rgbName;
    cv::Mat rgb;
    cv::Mat dpt;
    static const int data_size = 8 + 640*480*5;
    char *pbuf = new char[data_size];
    int cnt = 0 ;
    while(inf.getline(line, 4096))
    {
        sscanf(line, "%lf %s", &t, fname);
        cout<<"read time: "<<t<<" fname: "<<fname<<endl;
        dptName = m_savePath + string("/depth/") + string(fname);
        rgbName = m_savePath + string("/rgb/") + string(fname);
        m_dpt = imread(dptName.c_str(), -1);
        m_rgb = imread(rgbName.c_str(), -1);
        m_timestamp = t;
        if(!sendXtion2Server(pbuf, data_size))
        {
            cout<<"xtion_client.cpp: fail to send data to server!"<<endl;
            break;
        }
        ++cnt;
        if(cnt > 99) break;
    }
    delete []pbuf;
}

bool CXtionClient::sendXtion2Server(char* pbuf, int len)
{
    static const int pixel = 640 * 480;
    static const int dpt_size = 2*pixel; 
    static const int rgb_size = 3*pixel;
    static const int total_size = sizeof(double) + 640*480*5;
    assert(len == total_size);
    // char * pbuf = new char[total_size];
    // 1 timestamp 
    memcpy(pbuf,&m_timestamp, sizeof(double)); 
    // 2 dpt data 
    memcpy(pbuf+sizeof(double), m_dpt.data, dpt_size);
    // 3 rgb data 
    memcpy(pbuf+sizeof(double)+dpt_size, m_rgb.data, rgb_size);
    int iResult = send(m_socket_desc, pbuf, total_size, 0);
    if(iResult < total_size)
    {
        cout<<"xtion_client.cpp: sendXtion2Server fail: send iResult: "<<iResult<<endl;
        // delete []pbuf;
        return false;
    }else
    {
        cout<<"xtion_client.cpp: sendXtion2Server succeed!"<<endl;
    }
    usleep(100000);
    // delete []pbuf;
    return true;
}

bool CXtionClient::recordXtion2File()
{
    static const int width = 640; 
    static const int height = 480; 
    static const int n_pixel = width*height;
    static const int header_size = 8; // timestamp
    static const int depth_size = 2 * n_pixel;
    static const int rgb_size = 3 * n_pixel;
    static const int total_size = header_size + depth_size + rgb_size; 

    string savePathDepth;
    string savePathRGB; 
    string png; 
    string frgbd_name; 

    // png name
    stringstream ss; 
    ss<<std::fixed<<m_timestamp;
    png = ss.str() + string(".png");
    // assoc name
    frgbd_name = m_savePath + string("/rgbd_assoc.txt");
    static ofstream outf_rgbd(frgbd_name.c_str());
    outf_rgbd<<std::fixed<<std::setprecision(6)<<m_timestamp<<" "<<png<<endl;
    // depth name
    savePathDepth = m_savePath + string("/depth/") + png;
    if(imwrite(savePathDepth.c_str(), m_dpt))
    {   
        cout<<"successful write dpt file: "<<savePathDepth<<endl;
    }else{
        cout<<"failed to write dpt file: "<<savePathDepth<<endl;
        return false;
    }   
    // rgb name
    // sprintf(savePathRGB, "../data/rgb/%f.png", t); 
    savePathRGB = m_savePath + string("/rgb/") + png;
    if(imwrite(savePathRGB.c_str(), m_rgb))//seems imwrite only work with bgr now
    {   
        cout<<"successful write rgb file: "<<savePathRGB<<endl;
    }else
    {   
        cout<<"failed to write rgb file: "<<savePathRGB<<endl;
        return false;
    } 
    return true;
}
/*
bool CXtionClient::sendXtion()
{
    if(!m_bXtionReady)
    {
        cout<<"xtion_client.cpp: xtion is not ready yet!"<<endl;
        return false;
    }
    m_bStopSending = false;
    Mat rgb, depth;
    static const int width = 640; 
    static const int height = 480; 
    static const int n_pixel = width*height;
    static const int header_size = 8; // timestamp
    static const int depth_size = 2 * n_pixel;
    static const int rgb_size = 3 * n_pixel;
    static const int total_size = header_size + depth_size + rgb_size;
    
    char savePathDep[255];
    char savePathRGB[255];
    char png[255];
    ofstream outf_rgbd("../data/rgbd_assoc.txt");

    char* pbuf = new char[total_size];
    memset(pbuf, 0, total_size);
    while(!m_bStopSending)
    {
        cout<<"xtion_client.cpp start getFrame!"<<endl;
        if(m_openni.getFrame(rgb, depth))
        {
            cout<<"xtion_client.cpp: getframe and to send now!"<<endl;
            //save
            double t = m_openni.getRosTime();
            // construct buf 
            // 1 header timestamp
            memcpy(pbuf, &t, sizeof(double)); 
            // 2 depth value 
            memcpy(pbuf+header_size, depth.data, depth_size);
            // 3 rgb value 
            memcpy(pbuf+header_size+depth_size, rgb.data, rgb_size);
            
            int sent_data = 0; 
            int slice_data = 0; 
            // while(sent_data < total_size)
            {
                int iResult = send(m_socket_desc, pbuf, total_size, 0);
                if(iResult<=0) 
                {
                    cout<<"xtion_client.cpp failed to send data!"<<endl;
                    // break;
                }
                sent_data += iResult;
                cout<<"xtion_client.cpp: cur data iResult: "<<iResult<<" sent data: "<<sent_data<<" total_size: "<<total_size<<endl;
            }
             // cout.flush();
            if(sent_data < total_size)
            {
                if(m_bStopSending)
                   break;
                cout<<"xtion_client.cpp: only send data size: "<<sent_data<<endl;
            }else if(sent_data == total_size)
            {
                cout<<"xtion_client: succeed! send all data: "<<sent_data<<endl;
            }
            sleep(1); // sleep 1 s
            sprintf(png, "%f.png", t); 
            outf_rgbd<<std::fixed<<std::setprecision(6)<<t<<" "<<png<<endl;
            sprintf(savePathDep, "../data/depth/%f.png", t); 
            if(imwrite(savePathDep, depth))
            {   
                cout<<"successful write dpt file: "<<savePathDep<<endl;
            }else{
                cout<<"failed to write dpt file: "<<savePathDep<<endl;
            }   
            sprintf(savePathRGB, "../data/rgb/%f.png", t); 
            if(imwrite(savePathRGB, rgb))//seems imwrite only work with bgr now
            {   
                cout<<"successful write rgb file: "<<savePathRGB<<endl;
            }else
            {   
                cout<<"failed to write rgb file: "<<savePathRGB<<endl;
            }   
        }else break;
    }
    cout<<"stop to send xtion data!"<<endl;
    delete []pbuf;
}
*/
void CXtionClient::stopSendXtion()
{
    m_bStopSending = true;
}

void CXtionClient::stopRecordXtion()
{
    m_bStopRecording = true;
}

