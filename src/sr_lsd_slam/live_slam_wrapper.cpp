
#include "live_slam_wrapper.h"
#include <vector>
#include "util/SophusUtil.h"

// #include "SlamSystem.h"
#include "dpt_lsd_slam.h"

#include "IOWrapper/ImageDisplay.h"
#include "IOWrapper/Output3DWrapper.h"
// #include "IOWrapper/ROSrgbdStreamThread.h"
#include "IOWrapper/ROS/ROSrgbdStreamThread.h"
#include "util/globalFuncs.h"

#include <iostream>

// using namespace lsd_slam;
//

CLiveDptWrapper::CLiveDptWrapper(ROSrgbdStreamThread* imageStream, Output3DWrapper* outputWrapper)
{
	this->imageStream = imageStream;
	this->outputWrapper = outputWrapper;
	imageStream->getBuffer()->setReceiver(this);

	fx = imageStream->fx();
	fy = imageStream->fy();
	cx = imageStream->cx();
	cy = imageStream->cy();
	width = imageStream->width();
	height = imageStream->height();

	outFileName = packagePath+"estimated_poses.txt";


	isInitialized = false;


	Sophus::Matrix3f K_sophus;
	K_sophus << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

	outFile = nullptr;


	// make Odometry
	// monoOdometry = new SlamSystem(width, height, K_sophus, doSlam);
        dptOdometry = new CDLSDSlamSystem(width, height, K_sophus, doSlam);

	dptOdometry->setVisualization(outputWrapper);

	imageSeqNumber = 0;
}


CLiveDptWrapper::~CLiveDptWrapper()
{
	if(dptOdometry != 0)
		delete dptOdometry;
	if(outFile != 0)
	{
		outFile->flush();
		outFile->close();
		delete outFile;
	}
}

void CLiveDptWrapper::Loop()
{
	while (true) {
		boost::unique_lock<boost::recursive_mutex> waitLock(imageStream->getBuffer()->getMutex());
		while (!fullResetRequested && !(imageStream->getBuffer()->size() > 0)) {
			notifyCondition.wait(waitLock);
		}
		waitLock.unlock();
		

		if(fullResetRequested)
		{
			resetAll();
			fullResetRequested = false;
			if (!(imageStream->getBuffer()->size() > 0))
				continue;
		}
		
		TimestampedMat2 image = imageStream->getBuffer()->first();
		imageStream->getBuffer()->popFront();
		
	        // ROS_WARN("live_slam_wrapper.cpp: obtain a rgbd frame ");

		// process image
		// Util::displayImage("Video RGB", image.data1, true);
                // Util::displayImage("Video DPT", image.data2, true);
		// newImageCallback(image.data, image.timestamp);
		newrgbdCallback(image.data1, image.data2, image.timestamp);

                // Util::waitKey(1);
	}
}

void CLiveDptWrapper::newrgbdCallback(cv::Mat& img, cv::Mat& dpt, Timestamp imgTime)
{
    	++ imageSeqNumber;

	// Convert image to grayscale, if necessary
	cv::Mat grayImg;
	if (img.channels() == 1)
		grayImg = img;
	else
		cvtColor(img, grayImg, CV_RGB2GRAY);

	// Assert that we work with 8 bit images
	assert(grayImg.elemSize() == 1);
	assert(fx != 0 || fy != 0);

	// need to initialize
	if(!isInitialized)
	{
		// dptOdometry->randomInit(grayImg.data, imgTime.toSec(), 1);
                dptOdometry->gtDepthInit(grayImg.data, (float*)(dpt.data), imgTime.toSec(), 1);
		isInitialized = true;
	}
	else if(isInitialized && dptOdometry != nullptr)
	{
                // TODO: add depth here 
                // printf("live_slam_wrapper.cpp: trackFrame depth has not been implemented!\n");
		dptOdometry->trackFrame(grayImg.data, (float*)dpt.data, imageSeqNumber,false,imgTime.toSec());
	}

}

void CLiveDptWrapper::newImageCallback(const cv::Mat& img, Timestamp imgTime)
{
	++ imageSeqNumber;

	// Convert image to grayscale, if necessary
	cv::Mat grayImg;
	if (img.channels() == 1)
		grayImg = img;
	else
		cvtColor(img, grayImg, CV_RGB2GRAY);
	

	// Assert that we work with 8 bit images
	assert(grayImg.elemSize() == 1);
	assert(fx != 0 || fy != 0);


	// need to initialize
	if(!isInitialized)
	{
		dptOdometry->randomInit(grayImg.data, imgTime.toSec(), 1);
		isInitialized = true;
	}
	else if(isInitialized && dptOdometry != nullptr)
	{
                // TODO: add depth here 
                printf("live_slam_wrapper.cpp: trackFrame depth has not been implemented!\n");
		// dptOdometry->trackFrame(grayImg.data,imageSeqNumber,false,imgTime.toSec());
	}
}

void CLiveDptWrapper::logCameraPose(const SE3& camToWorld, double time)
{
	Sophus::Quaternionf quat = camToWorld.unit_quaternion().cast<float>();
	Eigen::Vector3f trans = camToWorld.translation().cast<float>();

	char buffer[1000];
	int num = snprintf(buffer, 1000, "%f %f %f %f %f %f %f %f\n",
			time,
			trans[0],
			trans[1],
			trans[2],
			quat.x(),
			quat.y(),
			quat.z(),
			quat.w());

	if(outFile == 0)
		outFile = new std::ofstream(outFileName.c_str());
	outFile->write(buffer,num);
	outFile->flush();
}

void CLiveDptWrapper::requestReset()
{
	fullResetRequested = true;
	notifyCondition.notify_all();
}

void CLiveDptWrapper::resetAll()
{
	if(dptOdometry != nullptr)
	{
		delete dptOdometry;
		printf("Deleted SlamSystem Object!\n");

		Sophus::Matrix3f K;
		K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
		// monoOdometry = new SlamSystem(width,height,K, doSlam);
                dptOdometry = new CDLSDSlamSystem(width, height, K, doSlam);
		dptOdometry->setVisualization(outputWrapper);

	}
	imageSeqNumber = 0;
	isInitialized = false;

	Util::closeAllWindows();

}

