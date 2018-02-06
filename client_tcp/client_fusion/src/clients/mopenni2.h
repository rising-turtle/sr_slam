#ifndef M_OPENNI2_H
#define M_OPENNI2_H


#ifdef _MSC_VER
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>  
#include <cmath>
//openni
#include <stdio.h>
#include <OpenNI.h>

//opencv
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <opencv/cvwimage.h>

#include <windows.h>

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define HAS_CLOCK_GETTIME (_POSIX_C_SOURCE >= 199309L)

using namespace std;
using namespace cv;
using namespace openni;


class MOpenni2
{
public:
	MOpenni2(){
		fps_ =30;
		height_ = 480;
		width_ = 640;


	};
	~MOpenni2(){};

	bool init();
	bool getFrame(cv::Mat& /*rgb*/, cv::Mat& /*depth*/);
	void close();

	double getRosTime();

protected:
	bool init_status_;
	int fps_;
	int height_;
	int width_;
	Device device;
	openni::VideoStream**		m_streams;

	openni::VideoStream			m_depthStream;
	openni::VideoStream			m_colorStream;

	openni::VideoMode depthVideoMode;
	openni::VideoMode colorVideoMode;
	
	openni::VideoFrameRef		m_depthFrame;
	openni::VideoFrameRef		m_colorFrame;

	Mat  m_depth16u_;
	Mat  m_rgb8u_;
	Mat  m_bgr8u_;
};


void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec);
 double round(double val);


#endif