
#include "mopenni2.h"



bool MOpenni2::init()
{
	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("MOpenni2:Initialize failed\n%s\n", OpenNI::getExtendedError());
		init_status_ = false;
		return 0;
	}

	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK)
	{
		printf("MOpenni2:Couldn't open device\n%s\n", OpenNI::getExtendedError());
		init_status_ = false;
		return 0;
	}
	depthVideoMode.setFps(fps_);
	depthVideoMode.setResolution(width_, height_);
	//depthVideoMode.setPixelFormat(PIXEL_FORMAT_DEPTH_100_UM);
	depthVideoMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	colorVideoMode.setFps(fps_);
	colorVideoMode.setResolution(width_, height_);
	colorVideoMode.setPixelFormat(PIXEL_FORMAT_RGB888);

	if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		rc = m_depthStream.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			init_status_ = false;
			return 0;
		}
		//set depth mode
		m_depthStream.setVideoMode(depthVideoMode);

		rc = m_depthStream.start();
		if (rc != STATUS_OK)
		{
			printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
			init_status_ = false;
			return 0;
		}

	}
	rc = m_colorStream.create(device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		//first set mode
		m_colorStream.setVideoMode(colorVideoMode);
		//second start
		rc = m_colorStream.start();
		if (rc != openni::STATUS_OK)
		{
			printf("MOpenni2: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
			m_colorStream.destroy();
			init_status_ = false;
			return 0;
		}
	}
	else
	{
		printf("MOpenni2: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
		init_status_ = false;
		return 0;
	}
	if (!m_depthStream.isValid() || !m_colorStream.isValid())
	{
		printf("MOpenni2: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		init_status_ = false;
		return 0;
	}

	m_streams = new openni::VideoStream*[2];
	m_streams[0] = &m_depthStream;
	m_streams[1] = &m_colorStream;

	//registration
	if(device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		if(device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR)!=STATUS_OK)
		{
			printf("MOpenni2:Can't hardware registration \n");
			init_status_ = false;
			return 0;
		}
	}

	//synchronization
	rc = device.setDepthColorSyncEnabled(true);
	if (rc != openni::STATUS_OK)
	{
		printf("MOpenni2:Can't frame sync \n");
		init_status_ = false;
		return 0;
	}

	if(m_depthStream.getMirroringEnabled())
	{
		m_depthStream.setMirroringEnabled(false);

	}
	if(m_colorStream.getMirroringEnabled())
	{
		m_colorStream.setMirroringEnabled(false);
	}


	//init container
	m_depth16u_.create( height_,width_,CV_16UC1);
	m_rgb8u_.create( height_,width_,CV_8UC3);
	m_bgr8u_.create( height_,width_,CV_8UC3);


	//final
	init_status_ = true;

	return 1;

}

bool MOpenni2::getFrame(cv::Mat& rgb, cv::Mat& depth)
{

	if(!init_status_)
	{
		printf("MOpenni2::getFrame: init is required! \n");
		return false;
	}

	int changedIndex;
	openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
	if (rc != openni::STATUS_OK)
	{
		printf("Wait failed\n");
		return 0;
	}

	switch (changedIndex)
	{
	case 0:
		m_depthStream.readFrame(&m_depthFrame); break;
	case 1:
		m_colorStream.readFrame(&m_colorFrame); break;
	default:
		printf("Error in wait\n");
	}

	if (m_depthFrame.isValid())
	{
		DepthPixel* pDepth = (DepthPixel*)m_depthFrame.getData();
		memcpy(m_depth16u_.data,(void*)pDepth,m_depthFrame.getHeight()*m_depthFrame.getWidth()*2);
	}

	if (m_colorFrame.isValid())
	{
		RGB888Pixel* pColor = (RGB888Pixel*)m_colorFrame.getData();
		memcpy(m_rgb8u_.data,pColor,m_colorFrame.getHeight()*m_colorFrame.getWidth()*3);
		cvtColor(m_rgb8u_,m_bgr8u_,CV_RGB2BGR);
	}

	rgb = m_bgr8u_;
	depth = m_depth16u_;

	return true;


}


void MOpenni2::close()
{
	m_depthStream.stop();
	m_colorStream.stop();
	m_depthStream.destroy();
	m_colorStream.destroy();
	device.close();
	OpenNI::shutdown();
}
double MOpenni2::getRosTime() 
{
	uint32_t sec, nsec;
#ifndef WIN32
#if HAS_CLOCK_GETTIME
	timespec start;
	clock_gettime(CLOCK_REALTIME, &start);
	sec  = start.tv_sec;
	nsec = start.tv_nsec;
#else
	struct timeval timeofday;
	gettimeofday(&timeofday,NULL);
	sec  = timeofday.tv_sec;
	nsec = timeofday.tv_usec * 1000;
#endif
#else
	static LARGE_INTEGER cpu_freq, init_cpu_time;
	static uint32_t start_sec = 0;
	static uint32_t start_nsec = 0;
	if ( ( start_sec == 0 ) && ( start_nsec == 0 ) )
	{
		QueryPerformanceFrequency(&cpu_freq);
		if (cpu_freq.QuadPart == 0) {
			printf("NoHighPerformanceTimersException()\n");
		}
		QueryPerformanceCounter(&init_cpu_time);
		// compute an offset from the Epoch using the lower-performance timer API
		FILETIME ft;
		GetSystemTimeAsFileTime(&ft);
		LARGE_INTEGER start_li;
		start_li.LowPart = ft.dwLowDateTime;
		start_li.HighPart = ft.dwHighDateTime;
		// why did they choose 1601 as the time zero, instead of 1970?
		// there were no outstanding hard rock bands in 1601.
#ifdef _MSC_VER
		start_li.QuadPart -= 116444736000000000Ui64;
#else
		start_li.QuadPart -= 116444736000000000ULL;
#endif
		start_sec = (uint32_t)(start_li.QuadPart / 10000000); // 100-ns units. odd.
		start_nsec = (start_li.LowPart % 10000000) * 100;
	}
	LARGE_INTEGER cur_time;
	QueryPerformanceCounter(&cur_time);
	LARGE_INTEGER delta_cpu_time;
	delta_cpu_time.QuadPart = cur_time.QuadPart - init_cpu_time.QuadPart;
	// todo: how to handle cpu clock drift. not sure it's a big deal for us.
	// also, think about clock wraparound. seems extremely unlikey, but possible
	double d_delta_cpu_time = delta_cpu_time.QuadPart / (double) cpu_freq.QuadPart;
	uint32_t delta_sec = (uint32_t) floor(d_delta_cpu_time);
	uint32_t delta_nsec = (uint32_t) round((d_delta_cpu_time-delta_sec) * 1e9);

	int64_t sec_sum  = (int64_t)start_sec  + (int64_t)delta_sec;
	int64_t nsec_sum = (int64_t)start_nsec + (int64_t)delta_nsec;

	// Throws an exception if we go out of 32-bit range
	normalizeSecNSecUnsigned(sec_sum, nsec_sum);

	sec = sec_sum;
	nsec = nsec_sum;
#endif

	return (double)sec + 1e-9*(double)nsec;
}


double round(double val)
{    
	return floor(val + 0.5);
}


void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec)
{
	int64_t nsec_part = nsec;
	int64_t sec_part = sec;

	while (nsec_part >= 1000000000L)
	{
		nsec_part -= 1000000000L;
		++sec_part;
	}
	while (nsec_part < 0)
	{
		nsec_part += 1000000000L;
		--sec_part;
	}

	if (sec_part < 0 || sec_part > INT_MAX)
		throw std::runtime_error("Time is out of dual 32-bit range");

	sec = sec_part;
	nsec = nsec_part;
}
