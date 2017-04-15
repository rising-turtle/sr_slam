
// #include "LiveSLAMWrapper.h"

#include "live_slam_wrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"


#include "IOWrapper/ROS/ROSrgbdStreamThread.h"
#include "IOWrapper/ROS/ROSImageStreamThread.h"
#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
#include "IOWrapper/ROS/rosReconfigure.h"

#include <X11/Xlib.h>

using namespace lsd_slam;
int main( int argc, char** argv )
{
    XInitThreads();

	ros::init(argc, argv, "main_live_dpt_lsd");

	dynamic_reconfigure::Server<lsd_slam_core::LSDParamsConfig> srv(ros::NodeHandle("~"));
	srv.setCallback(dynConfCb);

	dynamic_reconfigure::Server<lsd_slam_core::LSDDebugParamsConfig> srvDebug(ros::NodeHandle("~Debug"));
	srvDebug.setCallback(dynConfCbDebug);

	packagePath = ros::package::getPath("lsd_slam_core")+"/";

	// InputImageStream* inputStream = new ROSImageStreamThread();
        ROSrgbdStreamThread* inputStream = new ROSrgbdStreamThread(); 

	std::string calibFile;
	if(ros::param::get("~calib_rgb", calibFile))
	{
		ros::param::del("~calib_rgb");
		inputStream->setCalibration_rgb(calibFile);
	}
	else
		inputStream->setCalibration_rgb("");

	// std::string calibFile;
	if(ros::param::get("~calib_dpt", calibFile))
	{
		ros::param::del("~calib_dpt");
		inputStream->setCalibration_dpt(calibFile);
	}
	else
		inputStream->setCalibration_dpt("");

	inputStream->run();

	Output3DWrapper* outputWrapper = new ROSOutput3DWrapper(inputStream->width(), inputStream->height());
	// LiveSLAMWrapper slamNode(inputStream, outputWrapper);
        CLiveDptWrapper slamNode(inputStream, outputWrapper);
	slamNode.Loop();


	if (inputStream != nullptr)
		delete inputStream;
	if (outputWrapper != nullptr)
		delete outputWrapper;

	return 0;
}
