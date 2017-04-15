#ifndef LIVE_SLAM_WRAPPER_H
#define LIVE_SLAM_WRAPPER_H

#include <iostream>
#include <fstream>
#include <chrono>

#include "IOWrapper/Timestamp.h"
#include "IOWrapper/NotifyBuffer.h"
#include "IOWrapper/TimestampedObject.h"
#include "util/SophusUtil.h"

namespace cv {
	class Mat;
}

namespace lsd_slam
{

// class SlamSystem;
class CLiveDptWrapperROS;
class ROSrgbdStreamThread;
class Output3DWrapper;
}

class CDLSDSlamSystem;

using namespace lsd_slam; 

struct CLiveDptWrapper : public Notifiable
{
friend class CLiveDptWrapperROS;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CLiveDptWrapper(ROSrgbdStreamThread* imageStream, Output3DWrapper* outputWrapper);

	/** Destructor. */
	~CLiveDptWrapper();
	
	
	/** Runs the main processing loop. Will never return. */
	void Loop();
	
	/** Requests a reset from a different thread. */
	void requestReset();
	
	/** Resets everything, starting the odometry from the beginning again. */
	void resetAll();

	/** Callback function for new RGB images. */
	void newImageCallback(const cv::Mat& img, Timestamp imgTime);

        void newrgbdCallback(cv::Mat& img, cv::Mat& dpt, Timestamp imgTime);
	/** Writes the given time and pose to the outFile. */
	void logCameraPose(const SE3& camToWorld, double time);
	
	
	inline CDLSDSlamSystem* getSlamSystem() {return dptOdometry;}
	
private:
	
	ROSrgbdStreamThread* imageStream;
	Output3DWrapper* outputWrapper;

	// initialization stuff
	bool isInitialized;



	// dptOdometry
	// SlamSystem* monoOdometry;
        CDLSDSlamSystem* dptOdometry;

	std::string outFileName;
	std::ofstream* outFile;
	
	float fx, fy, cx, cy;
	int width, height;


	int imageSeqNumber;

};


#endif
