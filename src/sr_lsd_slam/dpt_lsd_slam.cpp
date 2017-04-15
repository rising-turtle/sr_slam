#include "dpt_lsd_slam.h"

#include "DataStructures/Frame.h"
#include "DataStructures/FrameSE3.h"
#include "Tracking/SE3Tracker.h"
#include "Tracking/Sim3Tracker.h"
// #include "DepthEstimation/DepthMap.h"
// #include "dpt_map.h"
// #include "DepthEstimation/DepthMapSE3.h"
#include "dpt_map_se3.h"
#include "Tracking/TrackingReference.h"
#include "Tracking/TrackingReferenceSE3.h"
#include "LiveSLAMWrapper.h"
#include "util/globalFuncs.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "GlobalMapping/KeyFrameGraphSE3.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
#include "GlobalMapping/TrackableKeyFrameSearchSE3.h"
#include "GlobalMapping/g2oTypeSim3Sophus.h"
#include "GlobalMapping/g2oTypeSE3Sophus.h"
#include "IOWrapper/ImageDisplay.h"
#include "IOWrapper/Output3DWrapper.h"
#include <g2o/core/robust_kernel_impl.h>
#include "DataStructures/FrameMemorySE3.h"
#include "deque"

// for mkdir
#include <sys/types.h>
#include <sys/stat.h>

#ifdef ANDROID
#include <android/log.h>
#endif

#include "opencv2/opencv.hpp"
#include <ros/ros.h>

using namespace lsd_slam;


CDLSDSlamSystem::CDLSDSlamSystem(int w, int h, Eigen::Matrix3f K, bool enableSLAM, int KF_every_k, int search_pre)
: SLAMEnabled(enableSLAM), relocalizer(w,h,K)
{
	if(w%16 != 0 || h%16!=0)
	{
		printf("image dimensions must be multiples of 16! Please crop your images / video accordingly.\n");
		assert(false);
	}

	this->width = w;
	this->height = h;
	this->K = K;
	trackingIsGood = true;


	currentKeyFrame =  nullptr;
	trackingReferenceFrameSharedPT = nullptr;
	// keyFrameGraph = new KeyFrameGraph();
        keyFrameGraph = new KeyFrameGraphSE3();
	createNewKeyFrame = false;

	// map =  new DepthMap(w,h,K);
        // map = new CDepthMap(w, h, K); 
	// map = new DepthMapSE3(w,h,K);
        map = new CDepthMapSE3(w, h, K);

	newConstraintAdded = false;
	haveUnmergedOptimizationOffset = false;


	tracker = new SE3Tracker(w,h,K);
	// Do not use more than 4 levels for odometry tracking
	for (int level = 4; level < PYRAMID_LEVELS; ++level)
		tracker->settings.maxItsPerLvl[level] = 0;
	trackingReference = new TrackingReferenceSE3();
	mappingTrackingReference = new TrackingReferenceSE3();


	if(SLAMEnabled)
	{
		trackableKeyFrameSearch = new TrackableKeyFrameSearchSE3(keyFrameGraph,w,h,K);
		constraintTracker = new Sim3Tracker(w,h,K);
		constraintSE3Tracker = new SE3Tracker(w,h,K);
		newKFTrackingReference = new TrackingReferenceSE3();
		candidateTrackingReference = new TrackingReferenceSE3();
	}
	else
	{
		constraintSE3Tracker = 0;
		trackableKeyFrameSearch = 0;
		constraintTracker = 0;
		newKFTrackingReference = 0;
		candidateTrackingReference = 0;
	}


	outputWrapper = 0;

	keepRunning = true;
	doFinalOptimization = false;
	depthMapScreenshotFlag = false;
	lastTrackingClosenessScore = 0;

	thread_mapping = boost::thread(&CDLSDSlamSystem::mappingThreadLoop, this);

	if(SLAMEnabled)
	{
		thread_constraint_search = boost::thread(&CDLSDSlamSystem::constraintSearchThreadLoop, this);
	        // thread_constraint_search = boost::thread(&CDLSDSlamSystem::constraintSearchThreadLoop_simple, this);
		thread_optimization = boost::thread(&CDLSDSlamSystem::optimizationThreadLoop, this);
	}



	msTrackFrame = msOptimizationIteration = msFindConstraintsItaration = msFindReferences = 0;
	nTrackFrame = nOptimizationIteration = nFindConstraintsItaration = nFindReferences = 0;
	nAvgTrackFrame = nAvgOptimizationIteration = nAvgFindConstraintsItaration = nAvgFindReferences = 0;
	gettimeofday(&lastHzUpdate, NULL);

        g_key_frame_k = KF_every_k; 
        g_match_pre_k = search_pre*g_key_frame_k;
}

CDLSDSlamSystem::~CDLSDSlamSystem()
{
	keepRunning = false;

	// make sure none is waiting for something.
	printf("... waiting for CDLSDSlamSystem's threads to exit\n");
	newFrameMappedSignal.notify_all();
	unmappedTrackedFramesSignal.notify_all();
	newKeyFrameCreatedSignal.notify_all();
	newConstraintCreatedSignal.notify_all();

	thread_mapping.join();
	thread_constraint_search.join();
	thread_optimization.join();
	printf("DONE waiting for CDLSDSlamSystem's threads to exit\n");

	if(trackableKeyFrameSearch != 0) delete trackableKeyFrameSearch;
	if(constraintTracker != 0) delete constraintTracker;
	if(constraintSE3Tracker != 0) delete constraintSE3Tracker;
	if(newKFTrackingReference != 0) delete newKFTrackingReference;
	if(candidateTrackingReference != 0) delete candidateTrackingReference;

	delete mappingTrackingReference;
	delete map;
	delete trackingReference;
	delete tracker;

	// make shure to reset all shared pointers to all frames before deleting the keyframegraph!
	unmappedTrackedFrames.clear();
	latestFrameTriedForReloc.reset();
	latestTrackedFrame.reset();
	currentKeyFrame.reset();
	trackingReferenceFrameSharedPT.reset();

	// delte keyframe graph
	delete keyFrameGraph;

	FrameMemorySE3::getInstance().releaseBuffes();


	Util::closeAllWindows();
}

void CDLSDSlamSystem::setVisualization(Output3DWrapper* outputWrapper)
{
	this->outputWrapper = outputWrapper;
}

void CDLSDSlamSystem::mergeOptimizationOffset()
{
	// update all vertices that are in the graph!
	poseConsistencyMutex.lock();

	bool needPublish = false;
	if(haveUnmergedOptimizationOffset)
	{
		keyFrameGraph->keyframesAllMutex.lock_shared();
		for(unsigned int i=0;i<keyFrameGraph->keyframesAll.size(); i++)
			keyFrameGraph->keyframesAll[i]->pose->applyPoseGraphOptResult();
		keyFrameGraph->keyframesAllMutex.unlock_shared();

		haveUnmergedOptimizationOffset = false;
		needPublish = true;
	}

	poseConsistencyMutex.unlock();






	if(needPublish)
		publishKeyframeGraph();
}



void CDLSDSlamSystem::mappingThreadLoop()
{
	printf("Started mapping thread!\n");
	while(keepRunning)
	{
		if (!doMappingIteration())
		{
			boost::unique_lock<boost::mutex> lock(unmappedTrackedFramesMutex);
			unmappedTrackedFramesSignal.timed_wait(lock,boost::posix_time::milliseconds(200));	// slight chance of deadlock otherwise
			lock.unlock();
		}

		newFrameMappedMutex.lock();
		newFrameMappedSignal.notify_all();
		newFrameMappedMutex.unlock();
	}
	printf("Exited mapping thread \n");
}

void CDLSDSlamSystem::finalize()
{
	printf("Finalizing Graph... finding final constraints!!\n");

	lastNumConstraintsAddedOnFullRetrack = 1;
	while(lastNumConstraintsAddedOnFullRetrack != 0)
	{
		doFullReConstraintTrack = true;
		usleep(200000);
	}


	printf("Finalizing Graph... optimizing!!\n");
	doFinalOptimization = true;
	newConstraintMutex.lock();
	newConstraintAdded = true;
	newConstraintCreatedSignal.notify_all();
	newConstraintMutex.unlock();
	while(doFinalOptimization)
	{
		usleep(200000);
	}


	printf("Finalizing Graph... publishing!!\n");
	unmappedTrackedFramesMutex.lock();
	unmappedTrackedFramesSignal.notify_one();
	unmappedTrackedFramesMutex.unlock();
	while(doFinalOptimization)
	{
		usleep(200000);
	}
	boost::unique_lock<boost::mutex> lock(newFrameMappedMutex);
	newFrameMappedSignal.wait(lock);
	newFrameMappedSignal.wait(lock);

	usleep(200000);
	printf("Done Finalizing Graph.!!\n");
}

// only search previous several frames 
void CDLSDSlamSystem::constraintSearchThreadLoop_simple()
{
    printf("Started constraint search simple thread!\n");
	
    boost::unique_lock<boost::mutex> lock(newKeyFrameMutex);
    int failedToRetrack = 0;

    while(keepRunning)
    {
      if(newKeyFrames.size() == 0)
      {
        lock.unlock();
        // do nothing not to retrack
        usleep(1000);
        lock.lock(); 
        newKeyFrameCreatedSignal.timed_wait(lock, boost::posix_time::milliseconds(500));
      }
      else
      {
        FrameSE3* newKF = newKeyFrames.front();
        newKeyFrames.pop_front();
        lock.unlock();

        struct timeval tv_start, tv_end;
        gettimeofday(&tv_start, NULL);
        // findConstraintsForNewKeyFrames(newKF, true, true, 1.0);
        findConstraintsForNewKeyFrames_simple(newKF, true, true, 1.0);
        failedToRetrack=0;
        gettimeofday(&tv_end, NULL);
        msFindConstraintsItaration = 0.9*msFindConstraintsItaration + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
        nFindConstraintsItaration++;

        FrameMemory::getInstance().pruneActiveFrames();
        lock.lock();
      }
    }
    printf("Exited constraint search simple thread \n");
}

void CDLSDSlamSystem::findConstraintsForNewKeyFrames_simple(FrameSE3* newKeyFrame, bool forceParent, bool useFABMAP, float closeCandidatesTH)
{
  if(!newKeyFrame->hasTrackingParent())
  {
    newConstraintMutex.lock(); 
    keyFrameGraph->addKeyFrame(newKeyFrame);
    newConstraintAdded = true; 
    newConstraintCreatedSignal.notify_all(); 
    newConstraintMutex.unlock(); 
    return 0; 
  }
  
  if(!forceParent && (newKeyFrame->lastConstraintTrackedCamToWorld * newKeyFrame->getScaledCamToWorld().inverse()).log().norm()<0.01)
    return 0;
  
  FrameSE3* parent = newKeyFrame->hasTrackingParent()?newKeyFrame->getTrackingParent():0;
  FrameSE3* p_parent = 0 ; // = parent->hasTrackingParent()?parent->getTrackingParent(): 0;

  // =============== get only parent constraint . =================
  std::vector<KFConstraintStructSE3*, Eigen::aligned_allocator<KFConstraintStructSE3*> > constraints;
  std::map< FrameSE3*, SE3, std::less<FrameSE3*>, Eigen::aligned_allocator<std::pair<FrameSE3*, SE3> > > candidateToFrame_initialEstimateMap; 
  poseConsistencyMutex.lock_shared();
  SE3 candidateToFrame_initialEstimate = newKeyFrame->getScaledCamToWorld().inverse() * parent->getScaledCamToWorld();
  candidateToFrame_initialEstimateMap[parent] = candidateToFrame_initialEstimate; 
  if(p_parent !=0 )
  {
    candidateToFrame_initialEstimate = newKeyFrame->getScaledCamToWorld().inverse() * p_parent->getScaledCamToWorld();
    candidateToFrame_initialEstimateMap[p_parent] = candidateToFrame_initialEstimate; 
  }

  poseConsistencyMutex.unlock_shared();

  // make tracking reference for newKeyFrame 
  newKFTrackingReference->importFrame(newKeyFrame);

  if(parent != 0 && forceParent)
  {
    KFConstraintStructSE3* e1=0;
    KFConstraintStructSE3* e2=0;
    
    testConstraintSE3(
        parent, e1, e2,
        candidateToFrame_initialEstimateMap[parent],
        100);
    if(enablePrintDebugInfo && printConstraintSearchInfo)
      printf(" PARENT (0)\n");

    if( e1 != 0)
    {
       constraints.push_back(e1);
       constraints.push_back(e2);
    }
    else
    {
      float downweightFac = 5;
      const float kernelDelta = 5 * sqrt(6000*loopclosureStrictness) / downweightFac;
      printf("warning: reciprocal tracking on new frame failed badly, added odometry edge (Hacky).\n");

      poseConsistencyMutex.lock_shared();
      constraints.push_back(new KFConstraintStructSE3());
      constraints.back()->firstFrame = newKeyFrame;
      constraints.back()->secondFrame = newKeyFrame->getTrackingParent();
      constraints.back()->secondToFirst = constraints.back()->firstFrame->getScaledCamToWorld().inverse() * constraints.back()->secondFrame->getScaledCamToWorld();
      /*constraints.back()->information  <<
        0.8098,-0.1507,-0.0557, 0.1211, 0.7657, 0.0120, 0,
        -0.1507, 2.1724,-0.1103,-1.9279,-0.1182, 0.1943, 0,
        -0.0557,-0.1103, 0.2643,-0.0021,-0.0657,-0.0028, 0.0304,
        0.1211,-1.9279,-0.0021, 2.3110, 0.1039,-0.0934, 0.0005,
        0.7657,-0.1182,-0.0657, 0.1039, 1.0545, 0.0743,-0.0028,
        0.0120, 0.1943,-0.0028,-0.0934, 0.0743, 0.4511, 0,
        0,0, 0.0304, 0.0005,-0.0028, 0, 0.0228;*/
      constraints.back()->information  <<
        0.8098,-0.1507,-0.0557, 0.1211, 0.7657, 0.0120, 
        -0.1507, 2.1724,-0.1103,-1.9279,-0.1182, 0.1943, 
        -0.0557,-0.1103, 0.2643,-0.0021,-0.0657,-0.0028, 
        0.1211,-1.9279,-0.0021, 2.3110, 0.1039,-0.0934, 
        0.7657,-0.1182,-0.0657, 0.1039, 1.0545, 0.0743,
        0.0120, 0.1943,-0.0028,-0.0934, 0.0743, 0.4511;
      constraints.back()->information *= (1e9/(downweightFac*downweightFac));

      constraints.back()->robustKernel = new g2o::RobustKernelHuber();
      constraints.back()->robustKernel->setDelta(kernelDelta);

      constraints.back()->meanResidual = 10;
      constraints.back()->meanResidualD = 10;
      constraints.back()->meanResidualP = 10;
      constraints.back()->usage = 0;

      poseConsistencyMutex.unlock_shared();
    }
  }

  if(p_parent != 0)
  {
    KFConstraintStructSE3* e1=0;
    KFConstraintStructSE3* e2=0;
    
    testConstraintSE3(
        p_parent, e1, e2,
        candidateToFrame_initialEstimateMap[p_parent],
        100);
    if(e1 != 0)
    {
      constraints.push_back(e1);
      constraints.push_back(e2);
    }
  }

  newConstraintMutex.lock(); 
  keyFrameGraph->addKeyFrame(newKeyFrame);
  for(unsigned int i=0;i<constraints.size();i++)
    keyFrameGraph->insertConstraint(constraints[i]);
  newConstraintAdded = true; 
  newConstraintCreatedSignal.notify_all(); 
  newConstraintMutex.unlock();

  newKFTrackingReference->invalidate(); 
  candidateTrackingReference->invalidate(); 

  return constraints.size();

}

void CDLSDSlamSystem::testConstraintSE3(
    FrameSE3* candidate, KFConstraintStructSE3* &e1_out, KFConstraintStructSE3* &e2_out, 
    SE3 candidateToFrame_initialEstimate, 
    float strictness)
{
    candidateTrackingReference->importFrame(candidate); 
    SE3 FtoC = candidateToFrame_initialEstimate.inverse(); 
    SE3 CtoF = candidateToFrame_initialEstimate; 
    Matrix6x6 FtoCInfo, CtoFInfo; 

    // delete all the father constraints
    int id1 = candidateTrackingReference->frameID;
    int id2 = newKFTrackingReference->frameID;

    if( (id1<=id2 && id1+g_match_pre_k < id2) || (id2<=id1 && id2+g_match_pre_k < id1) )
    {
      e1_out = e2_out = 0;
      return ;
    }


    float err_level2 = tryTrackSE3(newKFTrackingReference, candidateTrackingReference, 2, 2, USESSE, FtoC, CtoF); 
    if(err_level2 > 4000*strictness)
    {
      printf("dpt_lsd_slam.cpp: Failed %d -> %d (lvl %d): errs: (%.1f)\n", 
          newKFTrackingReference->frameID, candidateTrackingReference->frameID, 2, sqrtf(err_level2));
      e1_out = e2_out = 0; 
      // newKFTrackingReference->keyframe->trackingFailed.insert(std::pair<Frame*, SE3>)
      return ;
    }
    
    e1_out = new KFConstraintStructSE3(); 
    e2_out = new KFConstraintStructSE3(); 

    float err_level1 = tryTrackSE3(newKFTrackingReference, candidateTrackingReference, 0, 0, USESSE, FtoC, CtoF, e1_out, e2_out);
    if(err_level1 > 6000*strictness)
    {
      printf("dpt_lsd_slam.cpp: Failed %d -> %d (lvl %d): errs: (%.1f)\n", 
          newKFTrackingReference->frameID, candidateTrackingReference->frameID, 0, sqrtf(err_level1));
      delete e1_out; 
      delete e2_out;
      e1_out = e2_out = 0;
      return ;
    }
    
    printf("dpt_lsd_slam.cpp: ADDED %d -> %d errs: (%.1f / %.1f)\n",   
        newKFTrackingReference->frameID, candidateTrackingReference->frameID, sqrtf(err_level2), sqrtf(err_level1));

    const float kernelDelta = 5 * sqrt(6000*loopclosureStrictness); 
    e1_out->robustKernel = new g2o::RobustKernelHuber(); 
    e1_out->robustKernel->setDelta(kernelDelta); 
    e2_out->robustKernel = new g2o::RobustKernelHuber(); 
    e2_out->robustKernel->setDelta(kernelDelta);
}

float CDLSDSlamSystem::tryTrackSE3(
    TrackingReferenceSE3* A, TrackingReferenceSE3* B, 
    int lvlStart, int lvlEnd, 
    bool useSSE, SE3& AtoB, SE3& BtoA, 
    KFConstraintStructSE3 *e1, KFConstraintStructSE3* e2)
{
    BtoA = constraintSE3Tracker->trackFrame(
        A, B->keyframe, BtoA, lvlStart, lvlEnd); 
    Matrix6x6 BtoAInfo = constraintSE3Tracker->lastSE3Hessian;
    float BtoA_meanResidual = constraintSE3Tracker->lastResidual; 
    float BtoA_meanPResidual = BtoA_meanResidual; 
    float BtoA_meanDResidual = 0; 
    float BtoA_usage = constraintSE3Tracker->pointUsage; 
  
    if(constraintSE3Tracker->diverged || 
        BtoAInfo(0, 0) == 0 )
    {
      return 1e20;
    }

    AtoB = constraintSE3Tracker->trackFrame(
        B, A->keyframe, AtoB, lvlStart, lvlEnd, false);  // true for debug, record matched frames
    Matrix6x6 AtoBInfo = constraintSE3Tracker->lastSE3Hessian; 
    float AtoB_meanResidual = constraintSE3Tracker->lastResidual; 
    float AtoB_meanPResidual = AtoB_meanResidual; 
    float AtoB_meanDResidual = 0; 
    float AtoB_usage = constraintSE3Tracker->pointUsage; 
    
    if(constraintSE3Tracker->diverged || 
        AtoBInfo(0, 0) == 0)
    {
      return 1e20;
    }
    
    // Propagate uncertainty (with d(a*b)/d(b) = Adj_a) and calculate Mahalanobis norm 
    Matrix6x6 datimesb_db = AtoB.cast<float>().Adj(); 
    Matrix6x6 diffHesse = ((AtoBInfo.inverse() + datimesb_db* BtoAInfo.inverse() * datimesb_db.transpose())).inverse(); 
    Vector6 diff = (AtoB * BtoA).log().cast<float>();

    float reciprocalConsistency = (diffHesse*diff).dot(diff); 

    if(e1 != 0 && e2 != 0)
    {
      e1->firstFrame = A->keyframe;
      e1->secondFrame = B->keyframe;
      e1->secondToFirst = BtoA;
      e1->information = BtoAInfo.cast<double>();
      e1->meanResidual = BtoA_meanResidual;
      e1->meanResidualD = BtoA_meanDResidual;
      e1->meanResidualP = BtoA_meanPResidual;
      e1->usage = BtoA_usage;

      e2->firstFrame = B->keyframe;
      e2->secondFrame = A->keyframe;
      e2->secondToFirst = AtoB;
      e2->information = AtoBInfo.cast<double>();
      e2->meanResidual = AtoB_meanResidual;
      e2->meanResidualD = AtoB_meanDResidual;
      e2->meanResidualP = AtoB_meanPResidual;
      e2->usage = AtoB_usage;

      e1->reciprocalConsistency = e2->reciprocalConsistency = reciprocalConsistency;
    }
  return reciprocalConsistency; 
}

void CDLSDSlamSystem::constraintSearchThreadLoop()
{
	printf("Started  constraint search thread!\n");
	
	boost::unique_lock<boost::mutex> lock(newKeyFrameMutex);
	int failedToRetrack = 0;

	while(keepRunning)
	{
		if(newKeyFrames.size() == 0)
		{
			lock.unlock();
			keyFrameGraph->keyframesForRetrackMutex.lock();
			bool doneSomething = false;
                        bool b_retrack_pre = false;
			if(b_retrack_pre && keyFrameGraph->keyframesForRetrack.size() > 10)
			{
				std::deque< FrameSE3* >::iterator toReTrack = keyFrameGraph->keyframesForRetrack.begin() + (rand() % (keyFrameGraph->keyframesForRetrack.size()/3));
				FrameSE3* toReTrackFrame = *toReTrack;

				keyFrameGraph->keyframesForRetrack.erase(toReTrack);
				keyFrameGraph->keyframesForRetrack.push_back(toReTrackFrame);

				keyFrameGraph->keyframesForRetrackMutex.unlock();

				int found = findConstraintsForNewKeyFrames(toReTrackFrame, false, false, 2.0);
				if(found == 0)
					failedToRetrack++;
				else
					failedToRetrack=0;

				if(failedToRetrack < (int)keyFrameGraph->keyframesForRetrack.size() - 5)
					doneSomething = true;
			}
			else
				keyFrameGraph->keyframesForRetrackMutex.unlock();

			lock.lock();

			if(!doneSomething)
			{
				if(enablePrintDebugInfo && printConstraintSearchInfo)
					printf("nothing to re-track... waiting.\n");
				newKeyFrameCreatedSignal.timed_wait(lock,boost::posix_time::milliseconds(500));

			}
                        if(saveGraphStructure)
                        {
                          if(keyFrameGraph!=0)
                          {
                            keyFrameGraph->saveGraph("./lsd_slam/lsd_graph.log");
                          }
                        }
		}
		else
		{
			FrameSE3* newKF = newKeyFrames.front();
			newKeyFrames.pop_front();
			lock.unlock();

			struct timeval tv_start, tv_end;
			gettimeofday(&tv_start, NULL);

			findConstraintsForNewKeyFrames(newKF, true, true, 1.0);
			failedToRetrack=0;
			gettimeofday(&tv_end, NULL);
			msFindConstraintsItaration = 0.9*msFindConstraintsItaration + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
			nFindConstraintsItaration++;

			FrameMemorySE3::getInstance().pruneActiveFrames();
			lock.lock();
		}


		if(doFullReConstraintTrack)
		{
			lock.unlock();
			printf("Optizing Full Map!\n");

			int added = 0;
			for(unsigned int i=0;i<keyFrameGraph->keyframesAll.size();i++)
			{
				if(keyFrameGraph->keyframesAll[i]->pose->isInGraph)
					added += findConstraintsForNewKeyFrames(keyFrameGraph->keyframesAll[i], false, false, 1.0);
			}

			printf("Done optizing Full Map! Added %d constraints.\n", added);

			doFullReConstraintTrack = false;

			lastNumConstraintsAddedOnFullRetrack = added;
			lock.lock();
		}



	}

	printf("Exited constraint search thread \n");
}

void CDLSDSlamSystem::optimizationThreadLoop()
{
	printf("Started optimization thread \n");

	while(keepRunning)
	{
		boost::unique_lock<boost::mutex> lock(newConstraintMutex);
		if(!newConstraintAdded)
			newConstraintCreatedSignal.timed_wait(lock,boost::posix_time::milliseconds(2000));	// slight chance of deadlock otherwise
		newConstraintAdded = false;
		lock.unlock();

		if(doFinalOptimization)
		{
			printf("doing final optimization iteration!\n");
			optimizationIteration(50, 0.001);
			doFinalOptimization = false;
		}
		while(optimizationIteration(5, 0.02));
	}

	printf("Exited optimization thread \n");
}

void CDLSDSlamSystem::publishKeyframeGraph()
{
	if (outputWrapper != nullptr)
		outputWrapper->publishKeyframeGraph(keyFrameGraph);
}

void CDLSDSlamSystem::requestDepthMapScreenshot(const std::string& filename)
{
	depthMapScreenshotFilename = filename;
	depthMapScreenshotFlag = true;
}

void CDLSDSlamSystem::finishCurrentKeyframe()
{
	if(enablePrintDebugInfo && printThreadingInfo)
		printf("FINALIZING KF %d\n", currentKeyFrame->id());

	map->finalizeKeyFrame();

	if(SLAMEnabled)
	{
		mappingTrackingReference->importFrame(currentKeyFrame.get());
		currentKeyFrame->setPermaRef(mappingTrackingReference);
		mappingTrackingReference->invalidate();

		if(currentKeyFrame->idxInKeyframes < 0)
		{
			keyFrameGraph->keyframesAllMutex.lock();
			currentKeyFrame->idxInKeyframes = keyFrameGraph->keyframesAll.size();
			keyFrameGraph->keyframesAll.push_back(currentKeyFrame.get());
			keyFrameGraph->totalPoints += currentKeyFrame->numPoints;
			keyFrameGraph->totalVertices ++;
			keyFrameGraph->keyframesAllMutex.unlock();

			newKeyFrameMutex.lock();
			newKeyFrames.push_back(currentKeyFrame.get());
			newKeyFrameCreatedSignal.notify_all();
			newKeyFrameMutex.unlock();
		}
	}

	if(outputWrapper!= 0)
		outputWrapper->publishKeyframe(currentKeyFrame.get());
}

void CDLSDSlamSystem::discardCurrentKeyframe()
{
	if(enablePrintDebugInfo && printThreadingInfo)
		printf("DISCARDING KF %d\n", currentKeyFrame->id());

	if(currentKeyFrame->idxInKeyframes >= 0)
	{
		printf("WARNING: trying to discard a KF that has already been added to the graph... finalizing instead.\n");
		finishCurrentKeyframe();
		return;
	}


	map->invalidate();

	keyFrameGraph->allFramePosesMutex.lock();
	for(FramePoseStructSE3* p : keyFrameGraph->allFramePoses)
	{
		if(p->trackingParent != 0 && p->trackingParent->frameID == currentKeyFrame->id())
			p->trackingParent = 0;
	}
	keyFrameGraph->allFramePosesMutex.unlock();


	keyFrameGraph->idToKeyFrameMutex.lock();
	keyFrameGraph->idToKeyFrame.erase(currentKeyFrame->id());
	keyFrameGraph->idToKeyFrameMutex.unlock();

}

void CDLSDSlamSystem::createNewCurrentKeyframe(std::shared_ptr<FrameSE3> newKeyframeCandidate)
{
	if(enablePrintDebugInfo && printThreadingInfo)
		printf("CREATE NEW KF %d from %d\n", newKeyframeCandidate->id(), currentKeyFrame->id());


	if(SLAMEnabled)
	{
		// add NEW keyframe to id-lookup
		keyFrameGraph->idToKeyFrameMutex.lock();
		keyFrameGraph->idToKeyFrame.insert(std::make_pair(newKeyframeCandidate->id(), newKeyframeCandidate));
		keyFrameGraph->idToKeyFrameMutex.unlock();
	}

	// propagate & make new.
	map->createKeyFrame(newKeyframeCandidate.get());

	if(printPropagationStatistics)
	{

		Eigen::Matrix<float, 20, 1> data;
		data.setZero();
		data[0] = runningStats.num_prop_attempts / ((float)width*height);
		data[1] = (runningStats.num_prop_created + runningStats.num_prop_merged) / (float)runningStats.num_prop_attempts;
		data[2] = runningStats.num_prop_removed_colorDiff / (float)runningStats.num_prop_attempts;

		outputWrapper->publishDebugInfo(data);
	}

	currentKeyFrameMutex.lock();
	currentKeyFrame = newKeyframeCandidate;
	currentKeyFrameMutex.unlock();
}
void CDLSDSlamSystem::loadNewCurrentKeyframe(FrameSE3* keyframeToLoad)
{
	if(enablePrintDebugInfo && printThreadingInfo)
		printf("RE-ACTIVATE KF %d\n", keyframeToLoad->id());

	map->setFromExistingKF(keyframeToLoad);

	if(enablePrintDebugInfo && printRegularizeStatistics)
		printf("re-activate frame %d!\n", keyframeToLoad->id());

	currentKeyFrameMutex.lock();
	currentKeyFrame = keyFrameGraph->idToKeyFrame.find(keyframeToLoad->id())->second;
	currentKeyFrame->depthHasBeenUpdatedFlag = false;
	currentKeyFrameMutex.unlock();
}

void CDLSDSlamSystem::changeKeyframe(bool noCreate, bool force, float maxScore)
{
	FrameSE3* newReferenceKF=0;
	std::shared_ptr<FrameSE3> newKeyframeCandidate = latestTrackedFrame;
        if(doKFReActivation)
        {
          printf("dpt_lsd_slam.cpp: what!!!!!! doKFReActivation == true!\n");
          doKFReActivation = false;
        }
	if(doKFReActivation && SLAMEnabled)
	{
		struct timeval tv_start, tv_end;
		gettimeofday(&tv_start, NULL);
		newReferenceKF = trackableKeyFrameSearch->findRePositionCandidate(newKeyframeCandidate.get(), maxScore);
		gettimeofday(&tv_end, NULL);
		msFindReferences = 0.9*msFindReferences + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
		nFindReferences++;
	}

	if(newReferenceKF != 0)
        {
           printf("dpt_lsd_slam.cpp: what? loadNewCurrentKeyframe?!\n"); 
	   loadNewCurrentKeyframe(newReferenceKF);
        }
	else
	{
		if(force)
		{
			if(noCreate)
			{
				trackingIsGood = false;
				nextRelocIdx = -1;
				printf("mapping is disabled & moved outside of known map. Starting Relocalizer!\n");
			}
			else
				createNewCurrentKeyframe(newKeyframeCandidate);
		}
	}


	createNewKeyFrame = false;
}

bool CDLSDSlamSystem::updateKeyframe()
{
	std::shared_ptr<FrameSE3> reference = nullptr;
	std::deque< std::shared_ptr<FrameSE3> > references;

	unmappedTrackedFramesMutex.lock();

	// remove frames that have a different tracking parent.
	while(unmappedTrackedFrames.size() > 0 &&
			(!unmappedTrackedFrames.front()->hasTrackingParent() ||
					unmappedTrackedFrames.front()->getTrackingParent() != currentKeyFrame.get()))
	{
		unmappedTrackedFrames.front()->clear_refPixelWasGood();
		unmappedTrackedFrames.pop_front();
	}

	// clone list
	if(unmappedTrackedFrames.size() > 0)
	{
		for(unsigned int i=0;i<unmappedTrackedFrames.size(); i++)
			references.push_back(unmappedTrackedFrames[i]);

		std::shared_ptr<FrameSE3> popped = unmappedTrackedFrames.front();
		unmappedTrackedFrames.pop_front();
		unmappedTrackedFramesMutex.unlock();

		if(enablePrintDebugInfo && printThreadingInfo)
			printf("MAPPING %d on %d to %d (%d frames)\n", currentKeyFrame->id(), references.front()->id(), references.back()->id(), (int)references.size());

		map->updateKeyframe(references);

		popped->clear_refPixelWasGood();
		references.clear();
	}
	else
	{
		unmappedTrackedFramesMutex.unlock();
		return false;
	}


	if(enablePrintDebugInfo && printRegularizeStatistics)
	{
		Eigen::Matrix<float, 20, 1> data;
		data.setZero();
		data[0] = runningStats.num_reg_created;
		data[2] = runningStats.num_reg_smeared;
		data[3] = runningStats.num_reg_deleted_secondary;
		data[4] = runningStats.num_reg_deleted_occluded;
		data[5] = runningStats.num_reg_blacklisted;

		data[6] = runningStats.num_observe_created;
		data[7] = runningStats.num_observe_create_attempted;
		data[8] = runningStats.num_observe_updated;
		data[9] = runningStats.num_observe_update_attempted;


		data[10] = runningStats.num_observe_good;
		data[11] = runningStats.num_observe_inconsistent;
		data[12] = runningStats.num_observe_notfound;
		data[13] = runningStats.num_observe_skip_oob;
		data[14] = runningStats.num_observe_skip_fail;

		outputWrapper->publishDebugInfo(data);
	}



	if(outputWrapper != 0 && continuousPCOutput && currentKeyFrame != 0)
		outputWrapper->publishKeyframe(currentKeyFrame.get());

	return true;
}


void CDLSDSlamSystem::addTimingSamples()
{
	map->addTimingSample();
	struct timeval now;
	gettimeofday(&now, NULL);
	float sPassed = ((now.tv_sec-lastHzUpdate.tv_sec) + (now.tv_usec-lastHzUpdate.tv_usec)/1000000.0f);
	if(sPassed > 1.0f)
	{
		nAvgTrackFrame = 0.8*nAvgTrackFrame + 0.2*(nTrackFrame / sPassed); nTrackFrame = 0;
		nAvgOptimizationIteration = 0.8*nAvgOptimizationIteration + 0.2*(nOptimizationIteration / sPassed); nOptimizationIteration = 0;
		nAvgFindReferences = 0.8*nAvgFindReferences + 0.2*(nFindReferences / sPassed); nFindReferences = 0;

		if(trackableKeyFrameSearch != 0)
		{
			trackableKeyFrameSearch->nAvgTrackPermaRef = 0.8*trackableKeyFrameSearch->nAvgTrackPermaRef + 0.2*(trackableKeyFrameSearch->nTrackPermaRef / sPassed); trackableKeyFrameSearch->nTrackPermaRef = 0;
		}
		nAvgFindConstraintsItaration = 0.8*nAvgFindConstraintsItaration + 0.2*(nFindConstraintsItaration / sPassed); nFindConstraintsItaration = 0;
		nAvgOptimizationIteration = 0.8*nAvgOptimizationIteration + 0.2*(nOptimizationIteration / sPassed); nOptimizationIteration = 0;

		lastHzUpdate = now;


		if(enablePrintDebugInfo && printOverallTiming)
		{
			printf("MapIt: %3.1fms (%.1fHz); Track: %3.1fms (%.1fHz); Create: %3.1fms (%.1fHz); FindRef: %3.1fms (%.1fHz); PermaTrk: %3.1fms (%.1fHz); Opt: %3.1fms (%.1fHz); FindConst: %3.1fms (%.1fHz);\n",
					map->msUpdate, map->nAvgUpdate,
					msTrackFrame, nAvgTrackFrame,
					map->msCreate+map->msFinalize, map->nAvgCreate,
					msFindReferences, nAvgFindReferences,
					trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->msTrackPermaRef : 0, trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->nAvgTrackPermaRef : 0,
					msOptimizationIteration, nAvgOptimizationIteration,
					msFindConstraintsItaration, nAvgFindConstraintsItaration);
		}
	}

}


void CDLSDSlamSystem::debugDisplayDepthMap()
{


	map->debugPlotDepthMap();
	double scale = 1;
	// if(currentKeyFrame != 0 && currentKeyFrame != 0)
	//	scale = currentKeyFrame->getScaledCamToWorld().scale();
	// debug plot depthmap
	char buf1[200];
	char buf2[200];


	snprintf(buf1,200,"Map: Upd %3.0fms (%2.0fHz); Trk %3.0fms (%2.0fHz); %d / %d / %d",
			map->msUpdate, map->nAvgUpdate,
			msTrackFrame, nAvgTrackFrame,
			currentKeyFrame->numFramesTrackedOnThis, currentKeyFrame->numMappedOnThis, (int)unmappedTrackedFrames.size());

	snprintf(buf2,200,"dens %2.0f%%; good %2.0f%%; scale %2.2f; res %2.1f/; usg %2.0f%%; Map: %d F, %d KF, %d E, %.1fm Pts",
			100*currentKeyFrame->numPoints/(float)(width*height),
			100*tracking_lastGoodPerBad,
			scale,
			tracking_lastResidual,
			100*tracking_lastUsage,
			(int)keyFrameGraph->allFramePoses.size(),
			keyFrameGraph->totalVertices,
			(int)keyFrameGraph->edgesAll.size(),
			1e-6 * (float)keyFrameGraph->totalPoints);


	if(onSceenInfoDisplay)
		printMessageOnCVImage(map->debugImageDepth, buf1, buf2);
	if (displayDepthMap)
		Util::displayImage( "DebugWindow DEPTH", map->debugImageDepth, false );

	int pressedKey = Util::waitKey(1);
	handleKey(pressedKey);
}


void CDLSDSlamSystem::takeRelocalizeResult()
{
    printf("dpt_lsd_slam.cpp: relocalizeReuslt has not been implemented!\n");
    /*
	// Frame* keyframe;
        FrameSE3 * keyframe;
	int succFrameID;
	SE3 succFrameToKF_init;
	std::shared_ptr<FrameSE3> succFrame;
	relocalizer.stop();
	relocalizer.getResult(keyframe, succFrame, succFrameID, succFrameToKF_init);
	assert(keyframe != 0);

	loadNewCurrentKeyframe(keyframe);

	currentKeyFrameMutex.lock();
	trackingReference->importFrame(currentKeyFrame.get());
	trackingReferenceFrameSharedPT = currentKeyFrame;
	currentKeyFrameMutex.unlock();

	tracker->trackFrame(
			trackingReference,
			succFrame.get(),
			succFrameToKF_init);

	if(!tracker->trackingWasGood || tracker->lastGoodCount / (tracker->lastGoodCount + tracker->lastBadCount) < 1-0.75f*(1-MIN_GOODPERGOODBAD_PIXEL))
	{
		if(enablePrintDebugInfo && printRelocalizationInfo)
			printf("RELOCALIZATION FAILED BADLY! discarding result.\n");
		trackingReference->invalidate();
	}
	else
	{
		keyFrameGraph->addFrame(succFrame.get());

		unmappedTrackedFramesMutex.lock();
		if(unmappedTrackedFrames.size() < 50)
			unmappedTrackedFrames.push_back(succFrame);
		unmappedTrackedFramesMutex.unlock();

		currentKeyFrameMutex.lock();
		createNewKeyFrame = false;
		trackingIsGood = true;
		currentKeyFrameMutex.unlock();
	}*/
}

bool CDLSDSlamSystem::doMappingIteration()
{
	if(currentKeyFrame == 0)
		return false;

	if(!doMapping && currentKeyFrame->idxInKeyframes < 0)
	{
		if(currentKeyFrame->numMappedOnThisTotal >= MIN_NUM_MAPPED)
			finishCurrentKeyframe();
		else
			discardCurrentKeyframe();

		map->invalidate();
		printf("Finished KF %d as Mapping got disabled!\n",currentKeyFrame->id());

		changeKeyframe(true, true, 1.0f);
	}

	mergeOptimizationOffset();
	addTimingSamples();

	if(dumpMap)
	{
		keyFrameGraph->dumpMap(packagePath+"/save");
		dumpMap = false;
	}


	// set mappingFrame
	if(trackingIsGood)
	{
		if(!doMapping)
		{
			//printf("tryToChange refframe, lastScore %f!\n", lastTrackingClosenessScore);
			if(lastTrackingClosenessScore > 1)
				changeKeyframe(true, false, lastTrackingClosenessScore * 0.75);

			if (displayDepthMap || depthMapScreenshotFlag)
				debugDisplayDepthMap();

			return false;
		}


		if (createNewKeyFrame)
		{
			finishCurrentKeyframe();
			changeKeyframe(false, true, 1.0f);


			if (displayDepthMap || depthMapScreenshotFlag)
				debugDisplayDepthMap();
		}
		else
		{
			bool didSomething = updateKeyframe();

			if (displayDepthMap || depthMapScreenshotFlag)
				debugDisplayDepthMap();
			if(!didSomething)
				return false;
		}

		return true;
	}
	else
	{
          printf("dpt_lsd_slam.cpp: what? tracking is not good?!\n");
		// invalidate map if it was valid.
		if(map->isValid())
		{
			if(currentKeyFrame->numMappedOnThisTotal >= MIN_NUM_MAPPED)
				finishCurrentKeyframe();
			else
				discardCurrentKeyframe();

			map->invalidate();
		}

		// start relocalizer if it isnt running already
                printf("dpt_lsd_slam.cpp: relocalizer start not implemented!\n");
		// if(!relocalizer.isRunning)
		//	relocalizer.start(keyFrameGraph->keyframesAll);

		// did we find a frame to relocalize with?
		if(relocalizer.waitResult(50))
			takeRelocalizeResult();


		return true;
	}
}


void CDLSDSlamSystem::gtDepthInit(uchar* image, float* depth, double timeStamp, int id)
{
	printf("Doing GT initialization!\n");

	currentKeyFrameMutex.lock();

	// currentKeyFrame.reset(new Frame(id, width, height, K, timeStamp, image));
        currentKeyFrame.reset(new FrameSE3(id, width, height, K, timeStamp, image));
	currentKeyFrame->setDepthFromGroundTruth(depth);

	map->initializeFromGTDepth(currentKeyFrame.get());
	keyFrameGraph->addFrame(currentKeyFrame.get());

	currentKeyFrameMutex.unlock();

	if(doSlam)
	{
		keyFrameGraph->idToKeyFrameMutex.lock();
		keyFrameGraph->idToKeyFrame.insert(std::make_pair(currentKeyFrame->id(), currentKeyFrame));
		keyFrameGraph->idToKeyFrameMutex.unlock();
	}
	if(continuousPCOutput && outputWrapper != 0) outputWrapper->publishKeyframe(currentKeyFrame.get());

	printf("Done GT initialization!\n");
}

void CDLSDSlamSystem::setCurrentKFGT(float * gt_v)
{
    currentKeyFrameMutex.lock();
      currentKeyFrame->setGT(gt_v);
    currentKeyFrameMutex.unlock();
}

void CDLSDSlamSystem::randomInit(uchar* image, double timeStamp, int id)
{
	printf("Doing Random initialization!\n");

	if(!doMapping)
		printf("WARNING: mapping is disabled, but we just initialized... THIS WILL NOT WORK! Set doMapping to true.\n");


	currentKeyFrameMutex.lock();

	currentKeyFrame.reset(new FrameSE3(id, width, height, K, timeStamp, image));
	map->initializeRandomly(currentKeyFrame.get());
	keyFrameGraph->addFrame(currentKeyFrame.get());

	currentKeyFrameMutex.unlock();

	if(doSlam)
	{
		keyFrameGraph->idToKeyFrameMutex.lock();
		keyFrameGraph->idToKeyFrame.insert(std::make_pair(currentKeyFrame->id(), currentKeyFrame));
		keyFrameGraph->idToKeyFrameMutex.unlock();
	}
	if(continuousPCOutput && outputWrapper != 0) outputWrapper->publishKeyframe(currentKeyFrame.get());


	if (displayDepthMap || depthMapScreenshotFlag)
		debugDisplayDepthMap();


	printf("Done Random initialization!\n");

}

// track without setup KF 
void CDLSDSlamSystem::trackVO(uchar* image, float* dpt, unsigned int frameID, bool blockUntilMapped, double timestamp)
{
  ROS_INFO("frame %d at %lf do not has gt, in trackVO", frameID, timestamp);
    // Create new frame
    std::shared_ptr<FrameSE3> trackingNewFrame(new FrameSE3(frameID, width, height, K, timestamp, image));
    trackingNewFrame.get()->setDepthFromGroundTruth(dpt);
    
    // get tracking reference pose 
    currentKeyFrameMutex.lock();
    if(trackingReference->keyframe != currentKeyFrame.get() || currentKeyFrame->depthHasBeenUpdatedFlag)
    {
      trackingReference->importFrame(currentKeyFrame.get());
      currentKeyFrame->depthHasBeenUpdatedFlag = false;
      trackingReferenceFrameSharedPT = currentKeyFrame;
    }

    FramePoseStructSE3* trackingReferencePose = trackingReference->keyframe->pose;
    currentKeyFrameMutex.unlock();

    poseConsistencyMutex.lock_shared();
    SE3 frameToReference_initialEstimate =  trackingReferencePose->getCamToWorld().inverse() * keyFrameGraph->allFramePoses.back()->getCamToWorld();
    poseConsistencyMutex.unlock_shared();

    struct timeval tv_start, tv_end;
    gettimeofday(&tv_start, NULL);

    // track VO 
    SE3 newRefToFrame_poseUpdate = tracker->trackFrame(
        trackingReference,
        trackingNewFrame.get(),
        frameToReference_initialEstimate);
    gettimeofday(&tv_end, NULL);
    msTrackFrame = 0.9*msTrackFrame + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
    nTrackFrame++;

    tracking_lastResidual = tracker->lastResidual;
    tracking_lastUsage = tracker->pointUsage;
    tracking_lastGoodPerBad = tracker->lastGoodCount / (tracker->lastGoodCount + tracker->lastBadCount);
    tracking_lastGoodPerTotal = tracker->lastGoodCount / (trackingNewFrame->width(SE3TRACKING_MIN_LEVEL)*trackingNewFrame->height(SE3TRACKING_MIN_LEVEL));

    if(plotTracking)
    {
      Eigen::Matrix<float, 20, 1> data;
      data.setZero();
      data[0] = tracker->lastResidual;

      data[3] = tracker->lastGoodCount / (tracker->lastGoodCount + tracker->lastBadCount);
      data[4] = 4*tracker->lastGoodCount / (width*height);
      data[5] = tracker->pointUsage;

      data[6] = tracker->affineEstimation_a;
      data[7] = tracker->affineEstimation_b;
      outputWrapper->publishDebugInfo(data);
    }


    // add pose into pose-graph
    keyFrameGraph->addFrame(trackingNewFrame.get());

    if (outputWrapper != 0)
    {
      outputWrapper->publishTrackedFrame(trackingNewFrame.get());
    }

    // Keyframe selection
    latestTrackedFrame = trackingNewFrame;

    // map it 
    unmappedTrackedFramesMutex.lock();
    if(unmappedTrackedFrames.size() < 50 || (unmappedTrackedFrames.size() < 100 && trackingNewFrame->getTrackingParent()->numMappedOnThisTotal < 10))
      unmappedTrackedFrames.push_back(trackingNewFrame);
    unmappedTrackedFramesSignal.notify_one();
    unmappedTrackedFramesMutex.unlock();

    // implement blocking
    if(blockUntilMapped && trackingIsGood)
    {
      boost::unique_lock<boost::mutex> lock(newFrameMappedMutex);
      while(unmappedTrackedFrames.size() > 0)
      {
        //printf("TRACKING IS BLOCKING, waiting for %d frames to finish mapping.\n", (int)unmappedTrackedFrames.size());
        newFrameMappedSignal.wait(lock);
      }
      lock.unlock();
    }

    return ;
}

void CDLSDSlamSystem::trackFrameGT(uchar* image, float* dpt, unsigned int frameID, bool blockUntilMapped, double timestamp, bool has_gt, float *gt_p)
{
  static int next_gt_kf = -1;
  if(!has_gt || gt_p == 0)
  {
    // return trackFrame(image, dpt, frameID, blockUntilMapped, timestamp); 
     return trackVO(image, dpt, frameID, blockUntilMapped, timestamp);
  }  


  // Create new frame
  std::shared_ptr<FrameSE3> trackingNewFrame(new FrameSE3(frameID, width, height, K, timestamp, image, gt_p));
  trackingNewFrame.get()->setDepthFromGroundTruth(dpt);
  
  currentKeyFrameMutex.lock(); 
  if(trackingReference->keyframe != currentKeyFrame.get() || currentKeyFrame->depthHasBeenUpdatedFlag)
  {
    trackingReference->importFrame(currentKeyFrame.get());
    currentKeyFrame->depthHasBeenUpdatedFlag = false;
    trackingReferenceFrameSharedPT = currentKeyFrame;
  }

  FramePoseStructSE3* trackingReferencePose = trackingReference->keyframe->pose;
  currentKeyFrameMutex.unlock();
 
  // initial transformation between tracking reference keyframe and current tracking frame
  poseConsistencyMutex.lock_shared();
  SE3 frameToReference_initialEstimate = trackingReferencePose->getCamToWorld().inverse() * keyFrameGraph->allFramePoses.back()->getCamToWorld();
  poseConsistencyMutex.unlock_shared();

  SE3 newRefToFrame_poseUpdate = tracker->trackFrame(
			trackingReference,
			trackingNewFrame.get(),
			frameToReference_initialEstimate);
  
  // add new frame, only record its pose
  keyFrameGraph->addFrame(trackingNewFrame.get());
  if (outputWrapper != 0)
  {
    outputWrapper->publishTrackedFrame(trackingNewFrame.get());
  }

  if(has_gt && gt_p != 0)
  {
    // return trackFrame(image, dpt, frameID, blockUntilMapped, timestamp); 
    // return trackVO(image, dpt, frameID, blockUntilMapped, timestamp);
      
    // keyframe selection, here, each frame with gt, set it as a keyframe 
    ROS_INFO("sr_frame at %lf has gt, create a KF", timestamp);
  }  

  // Keyframe selection
  latestTrackedFrame = trackingNewFrame;

   createNewKeyFrame = true; 

  // notify a new unmapped frame 
  unmappedTrackedFramesMutex.lock();
	if(unmappedTrackedFrames.size() < 50 || (unmappedTrackedFrames.size() < 100 && trackingNewFrame->getTrackingParent()->numMappedOnThisTotal < 10))
		unmappedTrackedFrames.push_back(trackingNewFrame);
  unmappedTrackedFramesSignal.notify_one();
  unmappedTrackedFramesMutex.unlock();

  // implement blocking
  if(blockUntilMapped && trackingIsGood)
  {
    boost::unique_lock<boost::mutex> lock(newFrameMappedMutex);
    while(unmappedTrackedFrames.size() > 0)
    {
      //printf("TRACKING IS BLOCKING, waiting for %d frames to finish mapping.\n", (int)unmappedTrackedFrames.size());
      newFrameMappedSignal.wait(lock);
    }
    lock.unlock();
  }
  
  return ;

}


void CDLSDSlamSystem::trackFrame(uchar* image, float* dpt, unsigned int frameID, bool blockUntilMapped, double timestamp)
{
        static int last_kf_ID = -1;
	// Create new frame
	std::shared_ptr<FrameSE3> trackingNewFrame(new FrameSE3(frameID, width, height, K, timestamp, image));
        trackingNewFrame.get()->setDepthFromGroundTruth(dpt);

	if(!trackingIsGood)
	{
		// relocalizer.updateCurrentFrame(trackingNewFrame);

                printf("dpt_lsd_slam.cpp: what trackingIsBad? relocalizer not implemented! \n");
		unmappedTrackedFramesMutex.lock();
		unmappedTrackedFramesSignal.notify_one();
		unmappedTrackedFramesMutex.unlock();
                last_kf_ID = frameID;
		return;
	}

	currentKeyFrameMutex.lock();
	bool my_createNewKeyframe = createNewKeyFrame;	// pre-save here, to make decision afterwards.
	if(trackingReference->keyframe != currentKeyFrame.get() || currentKeyFrame->depthHasBeenUpdatedFlag)
	{
		trackingReference->importFrame(currentKeyFrame.get());
		currentKeyFrame->depthHasBeenUpdatedFlag = false;
		trackingReferenceFrameSharedPT = currentKeyFrame;
	}

	FramePoseStructSE3* trackingReferencePose = trackingReference->keyframe->pose;
	currentKeyFrameMutex.unlock();

	// DO TRACKING & Show tracking result.
	if(enablePrintDebugInfo && printThreadingInfo)
		printf("TRACKING %d on %d\n", trackingNewFrame->id(), trackingReferencePose->frameID);


	poseConsistencyMutex.lock_shared();
	//SE3 frameToReference_initialEstimate = se3FromSim3(
	//		trackingReferencePose->getCamToWorld().inverse() * keyFrameGraph->allFramePoses.back()->getCamToWorld());
	SE3 frameToReference_initialEstimate =	trackingReferencePose->getCamToWorld().inverse() * keyFrameGraph->allFramePoses.back()->getCamToWorld();

	poseConsistencyMutex.unlock_shared();



	struct timeval tv_start, tv_end;
	gettimeofday(&tv_start, NULL);

	SE3 newRefToFrame_poseUpdate = tracker->trackFrame(
			trackingReference,
			trackingNewFrame.get(),
			frameToReference_initialEstimate);


	gettimeofday(&tv_end, NULL);
	msTrackFrame = 0.9*msTrackFrame + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
	nTrackFrame++;

	tracking_lastResidual = tracker->lastResidual;
	tracking_lastUsage = tracker->pointUsage;
	tracking_lastGoodPerBad = tracker->lastGoodCount / (tracker->lastGoodCount + tracker->lastBadCount);
	tracking_lastGoodPerTotal = tracker->lastGoodCount / (trackingNewFrame->width(SE3TRACKING_MIN_LEVEL)*trackingNewFrame->height(SE3TRACKING_MIN_LEVEL));


	if(manualTrackingLossIndicated || tracker->diverged || (keyFrameGraph->keyframesAll.size() > INITIALIZATION_PHASE_COUNT && !tracker->trackingWasGood))
	{
		printf("TRACKING LOST for frame %d (%1.2f%% good Points, which is %1.2f%% of available points, %s)!\n",
				trackingNewFrame->id(),
				100*tracking_lastGoodPerTotal,
				100*tracking_lastGoodPerBad,
				tracker->diverged ? "DIVERGED" : "NOT DIVERGED");

		trackingReference->invalidate();

		trackingIsGood = false;
		nextRelocIdx = -1;

		unmappedTrackedFramesMutex.lock();
		unmappedTrackedFramesSignal.notify_one();
		unmappedTrackedFramesMutex.unlock();

		manualTrackingLossIndicated = false;
		return;
	}

	if(plotTracking)
	{
		Eigen::Matrix<float, 20, 1> data;
		data.setZero();
		data[0] = tracker->lastResidual;

		data[3] = tracker->lastGoodCount / (tracker->lastGoodCount + tracker->lastBadCount);
		data[4] = 4*tracker->lastGoodCount / (width*height);
		data[5] = tracker->pointUsage;

		data[6] = tracker->affineEstimation_a;
		data[7] = tracker->affineEstimation_b;
		outputWrapper->publishDebugInfo(data);
	}

	keyFrameGraph->addFrame(trackingNewFrame.get());


	//Sim3 lastTrackedCamToWorld = mostCurrentTrackedFrame->getScaledCamToWorld();//  mostCurrentTrackedFrame->TrackingParent->getScaledCamToWorld() * sim3FromSE3(mostCurrentTrackedFrame->thisToParent_SE3TrackingResult, 1.0);
	if (outputWrapper != 0)
	{
		outputWrapper->publishTrackedFrame(trackingNewFrame.get());
	}


	// Keyframe selection
	latestTrackedFrame = trackingNewFrame;
	if (!my_createNewKeyframe && currentKeyFrame->numMappedOnThisTotal > MIN_NUM_MAPPED)
	{
		Sophus::Vector3d dist = newRefToFrame_poseUpdate.translation() * currentKeyFrame->meanIdepth;
		float minVal = fmin(0.2f + keyFrameGraph->keyframesAll.size() * 0.8f / INITIALIZATION_PHASE_COUNT,1.0f);

		if(keyFrameGraph->keyframesAll.size() < INITIALIZATION_PHASE_COUNT)	minVal *= 0.7;

		lastTrackingClosenessScore = trackableKeyFrameSearch->getRefFrameScore(dist.dot(dist), tracker->pointUsage);

		if (lastTrackingClosenessScore > minVal)
		{
			createNewKeyFrame = true;

			if(enablePrintDebugInfo && printKeyframeSelectionInfo)
				printf("SELECT %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",trackingNewFrame->id(),trackingNewFrame->getTrackingParent()->id(), dist.dot(dist), tracker->pointUsage, trackableKeyFrameSearch->getRefFrameScore(dist.dot(dist), tracker->pointUsage));
		}
		else
		{
			if(enablePrintDebugInfo && printKeyframeSelectionInfo)
				printf("SKIPPD %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",trackingNewFrame->id(),trackingNewFrame->getTrackingParent()->id(), dist.dot(dist), tracker->pointUsage, trackableKeyFrameSearch->getRefFrameScore(dist.dot(dist), tracker->pointUsage));

		}
	}

        // TODO: create a new keyframe at least k frames 
        int create_kf_at_least_k_frames = g_key_frame_k; //2;
 	if (frameID>MIN_NUM_MAPPED && frameID - last_kf_ID >= create_kf_at_least_k_frames) 
        {
          createNewKeyFrame = true; 
          // ROS_WARN("dpt_lsd_slam.cpp: createNewKF, current frameID = %d last_kf_ID = %d", frameID, last_kf_ID);
          last_kf_ID = frameID;
        }

	unmappedTrackedFramesMutex.lock();
	if(unmappedTrackedFrames.size() < 50 || (unmappedTrackedFrames.size() < 100 && trackingNewFrame->getTrackingParent()->numMappedOnThisTotal < 10))
		unmappedTrackedFrames.push_back(trackingNewFrame);
	unmappedTrackedFramesSignal.notify_one();
	unmappedTrackedFramesMutex.unlock();

	// implement blocking
	if(blockUntilMapped && trackingIsGood)
	{
		boost::unique_lock<boost::mutex> lock(newFrameMappedMutex);
		while(unmappedTrackedFrames.size() > 0)
		{
			//printf("TRACKING IS BLOCKING, waiting for %d frames to finish mapping.\n", (int)unmappedTrackedFrames.size());
			newFrameMappedSignal.wait(lock);
		}
		lock.unlock();
	}
}

/*
float CDLSDSlamSystem::tryTrackSim3(
		TrackingReference* A, TrackingReference* B,
		int lvlStart, int lvlEnd,
		bool useSSE,
		Sim3 &AtoB, Sim3 &BtoA,
		KFConstraintStruct* e1, KFConstraintStruct* e2 )
{
	BtoA = constraintTracker->trackFrameSim3(
			A,
			B->keyframe,
			BtoA,
			lvlStart,lvlEnd);
	Matrix7x7 BtoAInfo = constraintTracker->lastSim3Hessian;
	float BtoA_meanResidual = constraintTracker->lastResidual;
	float BtoA_meanDResidual = constraintTracker->lastDepthResidual;
	float BtoA_meanPResidual = constraintTracker->lastPhotometricResidual;
	float BtoA_usage = constraintTracker->pointUsage;


	if (constraintTracker->diverged ||
		BtoA.scale() > 1 / Sophus::SophusConstants<sophusType>::epsilon() ||
		BtoA.scale() < Sophus::SophusConstants<sophusType>::epsilon() ||
		BtoAInfo(0,0) == 0 ||
		BtoAInfo(6,6) == 0)
	{
		return 1e20;
	}


	AtoB = constraintTracker->trackFrameSim3(
			B,
			A->keyframe,
			AtoB,
			lvlStart,lvlEnd);
	Matrix7x7 AtoBInfo = constraintTracker->lastSim3Hessian;
	float AtoB_meanResidual = constraintTracker->lastResidual;
	float AtoB_meanDResidual = constraintTracker->lastDepthResidual;
	float AtoB_meanPResidual = constraintTracker->lastPhotometricResidual;
	float AtoB_usage = constraintTracker->pointUsage;


	if (constraintTracker->diverged ||
		AtoB.scale() > 1 / Sophus::SophusConstants<sophusType>::epsilon() ||
		AtoB.scale() < Sophus::SophusConstants<sophusType>::epsilon() ||
		AtoBInfo(0,0) == 0 ||
		AtoBInfo(6,6) == 0)
	{
		return 1e20;
	}

	// Propagate uncertainty (with d(a * b) / d(b) = Adj_a) and calculate Mahalanobis norm
	Matrix7x7 datimesb_db = AtoB.cast<float>().Adj();
	Matrix7x7 diffHesse = (AtoBInfo.inverse() + datimesb_db * BtoAInfo.inverse() * datimesb_db.transpose()).inverse();
	Vector7 diff = (AtoB * BtoA).log().cast<float>();


	float reciprocalConsistency = (diffHesse * diff).dot(diff);


	if(e1 != 0 && e2 != 0)
	{
		e1->firstFrame = A->keyframe;
		e1->secondFrame = B->keyframe;
		e1->secondToFirst = BtoA;
		e1->information = BtoAInfo.cast<double>();
		e1->meanResidual = BtoA_meanResidual;
		e1->meanResidualD = BtoA_meanDResidual;
		e1->meanResidualP = BtoA_meanPResidual;
		e1->usage = BtoA_usage;

		e2->firstFrame = B->keyframe;
		e2->secondFrame = A->keyframe;
		e2->secondToFirst = AtoB;
		e2->information = AtoBInfo.cast<double>();
		e2->meanResidual = AtoB_meanResidual;
		e2->meanResidualD = AtoB_meanDResidual;
		e2->meanResidualP = AtoB_meanPResidual;
		e2->usage = AtoB_usage;

		e1->reciprocalConsistency = e2->reciprocalConsistency = reciprocalConsistency;
	}

	return reciprocalConsistency;
}


void CDLSDSlamSystem::testConstraint(
		Frame* candidate,
		KFConstraintStruct* &e1_out, KFConstraintStruct* &e2_out,
		Sim3 candidateToFrame_initialEstimate,
		float strictness)
{
	candidateTrackingReference->importFrame(candidate);

	Sim3 FtoC = candidateToFrame_initialEstimate.inverse(), CtoF = candidateToFrame_initialEstimate;
	Matrix7x7 FtoCInfo, CtoFInfo;

	float err_level3 = tryTrackSim3(
			newKFTrackingReference, candidateTrackingReference,	// A = frame; b = candidate
			SIM3TRACKING_MAX_LEVEL-1, 3,
			USESSE,
			FtoC, CtoF);

	if(err_level3 > 3000*strictness)
	{
		if(enablePrintDebugInfo && printConstraintSearchInfo)
			printf("FAILE %d -> %d (lvl %d): errs (%.1f / - / -).",
				newKFTrackingReference->frameID, candidateTrackingReference->frameID,
				3,
				sqrtf(err_level3));

		e1_out = e2_out = 0;

		newKFTrackingReference->keyframe->trackingFailed.insert(std::pair<Frame*,Sim3>(candidate, candidateToFrame_initialEstimate));
		return;
	}

	float err_level2 = tryTrackSim3(
			newKFTrackingReference, candidateTrackingReference,	// A = frame; b = candidate
			2, 2,
			USESSE,
			FtoC, CtoF);

	if(err_level2 > 4000*strictness)
	{
		if(enablePrintDebugInfo && printConstraintSearchInfo)
			printf("FAILE %d -> %d (lvl %d): errs (%.1f / %.1f / -).",
				newKFTrackingReference->frameID, candidateTrackingReference->frameID,
				2,
				sqrtf(err_level3), sqrtf(err_level2));

		e1_out = e2_out = 0;
		newKFTrackingReference->keyframe->trackingFailed.insert(std::pair<Frame*,Sim3>(candidate, candidateToFrame_initialEstimate));
		return;
	}

	e1_out = new KFConstraintStruct();
	e2_out = new KFConstraintStruct();


	float err_level1 = tryTrackSim3(
			newKFTrackingReference, candidateTrackingReference,	// A = frame; b = candidate
			1, 1,
			USESSE,
			FtoC, CtoF, e1_out, e2_out);

	if(err_level1 > 6000*strictness)
	{
		if(enablePrintDebugInfo && printConstraintSearchInfo)
			printf("FAILE %d -> %d (lvl %d): errs (%.1f / %.1f / %.1f).",
					newKFTrackingReference->frameID, candidateTrackingReference->frameID,
					1,
					sqrtf(err_level3), sqrtf(err_level2), sqrtf(err_level1));

		delete e1_out;
		delete e2_out;
		e1_out = e2_out = 0;
		newKFTrackingReference->keyframe->trackingFailed.insert(std::pair<Frame*,Sim3>(candidate, candidateToFrame_initialEstimate));
		return;
	}


	if(enablePrintDebugInfo && printConstraintSearchInfo)
		printf("ADDED %d -> %d: errs (%.1f / %.1f / %.1f).",
			newKFTrackingReference->frameID, candidateTrackingReference->frameID,
			sqrtf(err_level3), sqrtf(err_level2), sqrtf(err_level1));


	const float kernelDelta = 5 * sqrt(6000*loopclosureStrictness);
	e1_out->robustKernel = new g2o::RobustKernelHuber();
	e1_out->robustKernel->setDelta(kernelDelta);
	e2_out->robustKernel = new g2o::RobustKernelHuber();
	e2_out->robustKernel->setDelta(kernelDelta);
}
*/
int CDLSDSlamSystem::findConstraintsForNewKeyFrames(FrameSE3* newKeyFrame, bool forceParent, bool useFABMAP, float closeCandidatesTH)
{
	if(!newKeyFrame->hasTrackingParent())
	{
		newConstraintMutex.lock();
		keyFrameGraph->addKeyFrame(newKeyFrame);
		newConstraintAdded = true;
		newConstraintCreatedSignal.notify_all();
		newConstraintMutex.unlock();
		return 0;
	}

	if(!forceParent && (newKeyFrame->lastConstraintTrackedCamToWorld * newKeyFrame->getScaledCamToWorld().inverse()).log().norm() < 0.01)
		return 0;


	newKeyFrame->lastConstraintTrackedCamToWorld = newKeyFrame->getScaledCamToWorld();

	// =============== get all potential candidates and their initial relative pose. =================
	std::vector<KFConstraintStructSE3*, Eigen::aligned_allocator<KFConstraintStructSE3*> > constraints;
	FrameSE3* fabMapResult = 0;
	std::unordered_set<FrameSE3*, std::hash<FrameSE3*>, std::equal_to<FrameSE3*>,
		Eigen::aligned_allocator< FrameSE3* > > candidates = trackableKeyFrameSearch->findCandidates(newKeyFrame, fabMapResult, useFABMAP, closeCandidatesTH);
	std::map< FrameSE3*, SE3, std::less<FrameSE3*>, Eigen::aligned_allocator<std::pair<FrameSE3*, SE3> > > candidateToFrame_initialEstimateMap;


	// erase the ones that are already neighbours.
	for(std::unordered_set<FrameSE3*>::iterator c = candidates.begin(); c != candidates.end();)
	{
		if(newKeyFrame->neighbors.find(*c) != newKeyFrame->neighbors.end())
		{
			if(enablePrintDebugInfo && printConstraintSearchInfo)
				printf("SKIPPING %d on %d cause it already exists as constraint.\n", (*c)->id(), newKeyFrame->id());
			c = candidates.erase(c);
		}
		else
			++c;
	}

	poseConsistencyMutex.lock_shared();
	for (FrameSE3* candidate : candidates)
	{
		SE3 candidateToFrame_initialEstimate = newKeyFrame->getScaledCamToWorld().inverse() * candidate->getScaledCamToWorld();
		candidateToFrame_initialEstimateMap[candidate] = candidateToFrame_initialEstimate;
	}

	std::unordered_map<FrameSE3*, int> distancesToNewKeyFrame;
	if(newKeyFrame->hasTrackingParent())
		keyFrameGraph->calculateGraphDistancesToFrame(newKeyFrame->getTrackingParent(), &distancesToNewKeyFrame);
	poseConsistencyMutex.unlock_shared();





	// =============== distinguish between close and "far" candidates in Graph =================
	// Do a first check on trackability of close candidates.
	std::unordered_set<FrameSE3*, std::hash<FrameSE3*>, std::equal_to<FrameSE3*>,
		Eigen::aligned_allocator< FrameSE3* > > closeCandidates;
	std::vector<FrameSE3*, Eigen::aligned_allocator<FrameSE3*> > farCandidates;
	FrameSE3* parent = newKeyFrame->hasTrackingParent() ? newKeyFrame->getTrackingParent() : 0;

	int closeFailed = 0;
	int closeInconsistent = 0;

	SO3 disturbance = SO3::exp(Sophus::Vector3d(0.05,0,0));

	for (FrameSE3* candidate : candidates)
	{
		if (candidate->id() == newKeyFrame->id())
			continue;
		if(!candidate->pose->isInGraph)
			continue;
		if(newKeyFrame->hasTrackingParent() && candidate == newKeyFrame->getTrackingParent())
			continue;
		if(candidate->idxInKeyframes < INITIALIZATION_PHASE_COUNT)
			continue;

		SE3 c2f_init = candidateToFrame_initialEstimateMap[candidate]; //se3FromSim3(candidateToFrame_initialEstimateMap[candidate].inverse()).inverse();
		c2f_init.so3() = c2f_init.so3() * disturbance;
		SE3 c2f = constraintSE3Tracker->trackFrameOnPermaref(candidate, newKeyFrame, c2f_init);
		if(!constraintSE3Tracker->trackingWasGood) {closeFailed++; continue;}


		SE3 f2c_init = candidateToFrame_initialEstimateMap[candidate].inverse();// se3FromSim3(candidateToFrame_initialEstimateMap[candidate]).inverse();
		f2c_init.so3() = disturbance * f2c_init.so3();
		SE3 f2c = constraintSE3Tracker->trackFrameOnPermaref(newKeyFrame, candidate, f2c_init);
		if(!constraintSE3Tracker->trackingWasGood) {closeFailed++; continue;}

		if((f2c.so3() * c2f.so3()).log().norm() >= 0.09) {closeInconsistent++; continue;}

		closeCandidates.insert(candidate);
	}



	int farFailed = 0;
	int farInconsistent = 0;
	for (FrameSE3* candidate : candidates)
	{
		if (candidate->id() == newKeyFrame->id())
			continue;
		if(!candidate->pose->isInGraph)
			continue;
		if(newKeyFrame->hasTrackingParent() && candidate == newKeyFrame->getTrackingParent())
			continue;
		if(candidate->idxInKeyframes < INITIALIZATION_PHASE_COUNT)
			continue;

		if(candidate == fabMapResult)
		{
			farCandidates.push_back(candidate);
			continue;
		}

		if(distancesToNewKeyFrame.at(candidate) < 4)
			continue;

		farCandidates.push_back(candidate);
	}




	int closeAll = closeCandidates.size();
	int farAll = farCandidates.size();

	// erase the ones that we tried already before (close)
      for(std::unordered_set<FrameSE3*>::iterator c = closeCandidates.begin(); c != closeCandidates.end();)
      {
		if(newKeyFrame->trackingFailed.find(*c) == newKeyFrame->trackingFailed.end())
		{
			++c;
			continue;
		}
		auto range = newKeyFrame->trackingFailed.equal_range(*c);

		bool skip = false;
		SE3 f2c = candidateToFrame_initialEstimateMap[*c].inverse();
		for (auto it = range.first; it != range.second; ++it)
		{
			if((f2c * it->second).log().norm() < 0.1)
			{
				skip=true;
				break;
			}
		}

		if(skip)
		{
			if(enablePrintDebugInfo && printConstraintSearchInfo)
				printf("SKIPPING %d on %d (NEAR), cause we already have tried it.\n", (*c)->id(), newKeyFrame->id());
			c = closeCandidates.erase(c);
		}
		else
			++c;
	}

	// erase the ones that are already neighbours (far)
	for(unsigned int i=0;i<farCandidates.size();i++)
	{
		if(newKeyFrame->trackingFailed.find(farCandidates[i]) == newKeyFrame->trackingFailed.end())
			continue;

		auto range = newKeyFrame->trackingFailed.equal_range(farCandidates[i]);

		bool skip = false;
		for (auto it = range.first; it != range.second; ++it)
		{
			if((it->second).log().norm() < 0.2)
			{
				skip=true;
				break;
			}
		}

		if(skip)
		{
			if(enablePrintDebugInfo && printConstraintSearchInfo)
				printf("SKIPPING %d on %d (FAR), cause we already have tried it.\n", farCandidates[i]->id(), newKeyFrame->id());
			farCandidates[i] = farCandidates.back();
			farCandidates.pop_back();
			i--;
		}
	}



	if (enablePrintDebugInfo && printConstraintSearchInfo)
		printf("Final Loop-Closure Candidates: %d / %d close (%d failed, %d inconsistent) + %d / %d far (%d failed, %d inconsistent) = %d\n",
				(int)closeCandidates.size(),closeAll, closeFailed, closeInconsistent,
				(int)farCandidates.size(), farAll, farFailed, farInconsistent,
				(int)closeCandidates.size() + (int)farCandidates.size());



	// =============== limit number of close candidates ===============
	// while too many, remove the one with the highest connectivity.
	while((int)closeCandidates.size() > maxLoopClosureCandidates)
	{
		FrameSE3* worst = 0;
		int worstNeighbours = 0;
		for(FrameSE3* f : closeCandidates)
		{
			int neightboursInCandidates = 0;
			for(FrameSE3* n : f->neighbors)
				if(closeCandidates.find(n) != closeCandidates.end())
					neightboursInCandidates++;

			if(neightboursInCandidates > worstNeighbours || worst == 0)
			{
				worst = f;
				worstNeighbours = neightboursInCandidates;
			}
		}

		closeCandidates.erase(worst);
	}


	// =============== limit number of far candidates ===============
	// delete randomly
	int maxNumFarCandidates = (maxLoopClosureCandidates +1) / 2;
	if(maxNumFarCandidates < 5) maxNumFarCandidates = 5;
	while((int)farCandidates.size() > maxNumFarCandidates)
	{
		int toDelete = rand() % farCandidates.size();
		if(farCandidates[toDelete] != fabMapResult)
		{
			farCandidates[toDelete] = farCandidates.back();
			farCandidates.pop_back();
		}
	}







	// =============== TRACK! ===============

	// make tracking reference for newKeyFrame.
	newKFTrackingReference->importFrame(newKeyFrame);


	for (FrameSE3* candidate : closeCandidates)
	{
		KFConstraintStructSE3* e1=0;
		KFConstraintStructSE3* e2=0;

		testConstraintSE3(
				candidate, e1, e2,
				candidateToFrame_initialEstimateMap[candidate],
				loopclosureStrictness);

		if(enablePrintDebugInfo && printConstraintSearchInfo)
			printf(" CLOSE (%d)\n", distancesToNewKeyFrame.at(candidate));

		if(e1 != 0)
		{
			constraints.push_back(e1);
			constraints.push_back(e2);

			// delete from far candidates if it's in there.
			for(unsigned int k=0;k<farCandidates.size();k++)
			{
				if(farCandidates[k] == candidate)
				{
					if(enablePrintDebugInfo && printConstraintSearchInfo)
						printf(" DELETED %d from far, as close was successful!\n", candidate->id());

					farCandidates[k] = farCandidates.back();
					farCandidates.pop_back();
				}
			}
		}
	}


	for (FrameSE3* candidate : farCandidates)
	{
		KFConstraintStructSE3* e1=0;
		KFConstraintStructSE3* e2=0;

		testConstraintSE3(
				candidate, e1, e2,
				SE3(),
				loopclosureStrictness);

		if(enablePrintDebugInfo && printConstraintSearchInfo)
			printf(" FAR (%d)\n", distancesToNewKeyFrame.at(candidate));

		if(e1 != 0)
		{
			constraints.push_back(e1);
			constraints.push_back(e2);
		}
	}



	if(parent != 0 && forceParent)
	{
		KFConstraintStructSE3* e1=0;
		KFConstraintStructSE3* e2=0;
		testConstraintSE3(
				parent, e1, e2,
				candidateToFrame_initialEstimateMap[parent],
				100);
		if(enablePrintDebugInfo && printConstraintSearchInfo)
			printf(" PARENT (0)\n");

		if(e1 != 0)
		{
			constraints.push_back(e1);
			constraints.push_back(e2);
		}
		else
		{
			float downweightFac = 5;
			const float kernelDelta = 5 * sqrt(6000*loopclosureStrictness) / downweightFac;
			printf("warning: reciprocal tracking on new frame failed badly, added odometry edge (Hacky).\n");

			poseConsistencyMutex.lock_shared();
			constraints.push_back(new KFConstraintStructSE3());
			constraints.back()->firstFrame = newKeyFrame;
			constraints.back()->secondFrame = newKeyFrame->getTrackingParent();
			constraints.back()->secondToFirst = constraints.back()->firstFrame->getScaledCamToWorld().inverse() * constraints.back()->secondFrame->getScaledCamToWorld();
			/*constraints.back()->information  <<
					0.8098,-0.1507,-0.0557, 0.1211, 0.7657, 0.0120, 0,
					-0.1507, 2.1724,-0.1103,-1.9279,-0.1182, 0.1943, 0,
					-0.0557,-0.1103, 0.2643,-0.0021,-0.0657,-0.0028, 0.0304,
					 0.1211,-1.9279,-0.0021, 2.3110, 0.1039,-0.0934, 0.0005,
					 0.7657,-0.1182,-0.0657, 0.1039, 1.0545, 0.0743,-0.0028,
					 0.0120, 0.1943,-0.0028,-0.0934, 0.0743, 0.4511, 0,
					0,0, 0.0304, 0.0005,-0.0028, 0, 0.0228;*/
			constraints.back()->information  <<
					0.8098,-0.1507,-0.0557, 0.1211, 0.7657, 0.0120, 
					-0.1507, 2.1724,-0.1103,-1.9279,-0.1182, 0.1943, 
					-0.0557,-0.1103, 0.2643,-0.0021,-0.0657,-0.0028, 
					 0.1211,-1.9279,-0.0021, 2.3110, 0.1039,-0.0934, 
					 0.7657,-0.1182,-0.0657, 0.1039, 1.0545, 0.0743,
					 0.0120, 0.1943,-0.0028,-0.0934, 0.0743, 0.4511;

			constraints.back()->information *= (1e9/(downweightFac*downweightFac));

			constraints.back()->robustKernel = new g2o::RobustKernelHuber();
			constraints.back()->robustKernel->setDelta(kernelDelta);

			constraints.back()->meanResidual = 10;
			constraints.back()->meanResidualD = 10;
			constraints.back()->meanResidualP = 10;
			constraints.back()->usage = 0;

			poseConsistencyMutex.unlock_shared();
		}
	}


	newConstraintMutex.lock();

	keyFrameGraph->addKeyFrame(newKeyFrame);
	for(unsigned int i=0;i<constraints.size();i++)
		keyFrameGraph->insertConstraint(constraints[i]);


	newConstraintAdded = true;
	newConstraintCreatedSignal.notify_all();
	newConstraintMutex.unlock();

	newKFTrackingReference->invalidate();
	candidateTrackingReference->invalidate();



	return constraints.size();
}



bool CDLSDSlamSystem::optimizationIteration(int itsPerTry, float minChange)
{
	struct timeval tv_start, tv_end;
	gettimeofday(&tv_start, NULL);



	g2oGraphAccessMutex.lock();

	// lock new elements buffer & take them over.
	newConstraintMutex.lock();
	keyFrameGraph->addElementsFromBuffer();
	newConstraintMutex.unlock();


	// Do the optimization. This can take quite some time!
	int its = keyFrameGraph->optimize(itsPerTry);
	

	// save the optimization result.
	poseConsistencyMutex.lock_shared();
	keyFrameGraph->keyframesAllMutex.lock_shared();
	float maxChange = 0;
	float sumChange = 0;
	float sum = 0;
	for(size_t i=0;i<keyFrameGraph->keyframesAll.size(); i++)
	{
		// set edge error sum to zero
		keyFrameGraph->keyframesAll[i]->edgeErrorSum = 0;
		keyFrameGraph->keyframesAll[i]->edgesNum = 0;

		if(!keyFrameGraph->keyframesAll[i]->pose->isInGraph) continue;



		// get change from last optimization
		// Sim3 a = keyFrameGraph->keyframesAll[i]->pose->graphVertex->estimate();
		// Sim3 b = keyFrameGraph->keyframesAll[i]->getScaledCamToWorld();
		// Sophus::Vector7f diff = (a*b.inverse()).log().cast<float>();

	        SE3 a = keyFrameGraph->keyframesAll[i]->pose->graphVertex->estimate();
		SE3 b = keyFrameGraph->keyframesAll[i]->getScaledCamToWorld();
		Sophus::Vector6f diff = (a*b.inverse()).log().cast<float>();


		for(int j=0;j<6;j++)
		{
			float d = fabsf((float)(diff[j]));
			if(d > maxChange) maxChange = d;
			sumChange += d;
		}
		sum +=6;

		// set change
		keyFrameGraph->keyframesAll[i]->pose->setPoseGraphOptResult(
				keyFrameGraph->keyframesAll[i]->pose->graphVertex->estimate());

		// add error
		for(auto edge : keyFrameGraph->keyframesAll[i]->pose->graphVertex->edges())
		{
			// keyFrameGraph->keyframesAll[i]->edgeErrorSum += ((EdgeSim3*)(edge))->chi2();
                        keyFrameGraph->keyframesAll[i]->edgeErrorSum += ((EdgeSE3*)edge)->chi2();
			keyFrameGraph->keyframesAll[i]->edgesNum++;
		}
	}

	haveUnmergedOptimizationOffset = true;
	keyFrameGraph->keyframesAllMutex.unlock_shared();
	poseConsistencyMutex.unlock_shared();

	g2oGraphAccessMutex.unlock();

	if(enablePrintDebugInfo && printOptimizationInfo)
		printf("did %d optimization iterations. Max Pose Parameter Change: %f; avgChange: %f. %s\n", its, maxChange, sumChange / sum,
				maxChange > minChange && its == itsPerTry ? "continue optimizing":"Waiting for addition to graph.");


	gettimeofday(&tv_end, NULL);
	msOptimizationIteration = 0.9*msOptimizationIteration + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
	nOptimizationIteration++;


	return maxChange > minChange && its == itsPerTry;
}


void CDLSDSlamSystem::saveKFGraph(std::string file)
{
  keyFrameGraph->saveGraph(file);
}

void CDLSDSlamSystem::optimizeGraph()
{
	boost::unique_lock<boost::mutex> g2oLock(g2oGraphAccessMutex);
	keyFrameGraph->optimize(1000);
	g2oLock.unlock();
	mergeOptimizationOffset();
}


SE3 CDLSDSlamSystem::getCurrentPoseEstimate()
{
	SE3 camToWorld = SE3();
	keyFrameGraph->allFramePosesMutex.lock_shared();
	if(keyFrameGraph->allFramePoses.size() > 0)
		// camToWorld = se3FromSim3(keyFrameGraph->allFramePoses.back()->getCamToWorld());
		camToWorld = keyFrameGraph->allFramePoses.back()->getCamToWorld();
	keyFrameGraph->allFramePosesMutex.unlock_shared();
	return camToWorld;
}

std::vector<FramePoseStructSE3*, Eigen::aligned_allocator<FramePoseStructSE3*> > CDLSDSlamSystem::getAllPoses()
{
	return keyFrameGraph->allFramePoses;
}
