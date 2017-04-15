/*
 *  July. 2, 2016 David Z
 *  
 *  the sr_lsd_slam is only a subclass of lsd-slam, 
 *  depth_lsd_slam is a new SLAM method that using depth information 
 *
 * */

#ifndef DPT_LSD_SLAM_H
#define DPT_LSD_SLAM_H

#include <vector>
#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/locks.hpp>
#include "util/settings.h"
#include "IOWrapper/Timestamp.h"
#include "opencv2/core/core.hpp"

#include "util/SophusUtil.h"

#include "Tracking/Relocalizer.h"


using namespace lsd_slam;

namespace lsd_slam{
class TrackingReference;
class TrackingReferenceSE3; 
class KeyFrameGraph;
class KeyFrameGraphSE3;
class SE3Tracker;
class Sim3Tracker;
// class DepthMap;
// class DepthMapSE3;
class Frame;
class FrameSE3;
class DataSet;
class LiveSLAMWrapper;
class Output3DWrapper;
class TrackableKeyFrameSearch;
class TrackableKeyFrameSearchSE3;
class FramePoseStruct;
class FramePoseStructSE3;
struct KFConstraintStruct;
struct KFConstraintStructSE3;
}

// class CDepthMap;
class CDepthMapSE3;
typedef Eigen::Matrix<float, 7, 7> Matrix7x7;
typedef Eigen::Matrix<float, 6, 6> Matrix6x6;

class CDLSDSlamSystem
{
  friend class IntegrationTest;
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // settings. Constant from construction onward.
    int width;
  int height;
  Eigen::Matrix3f K;
  const bool SLAMEnabled;

  bool trackingIsGood;


  CDLSDSlamSystem(int w, int h, Eigen::Matrix3f K, bool enableSLAM = true, int KF_every_k = 2, int search_previous = 3);
  CDLSDSlamSystem(const CDLSDSlamSystem&) = delete;
  CDLSDSlamSystem& operator=(const CDLSDSlamSystem&) = delete;
  ~CDLSDSlamSystem();

  void randomInit(uchar* image, double timeStamp, int id);
  void gtDepthInit(uchar* image, float* depth, double timeStamp, int id);
  void setCurrentKFGT(float* gt_v);

  // tracks a frame.
  // first frame will return Identity = camToWord.
  // returns camToWord transformation of the tracked frame.
  // frameID needs to be monotonically increasing.
  // void trackFrame(uchar* image, unsigned int frameID, bool blockUntilMapped, double timestamp);
  void trackFrame(uchar* image, float* dpt, unsigned int frameID, bool blockUntilMapped, double timestamp);

  // tracks a frame with gt 
  void trackFrameGT(uchar* image, float* dpt, unsigned int frameID, bool blockUntilMapped, double timestamp, bool has_gt = false, 
      float *gt_p = 0);
    
  // tracks a frame using VO, no KF is created 
  void trackVO(uchar* image, float* dpt, unsigned int frameID, bool blockUntilMapped, double timestamp);

  // finalizes the system, i.e. blocks and does all remaining loop-closures etc.
  void finalize();

  int g_key_frame_k;    // every at least k frame has a KF 
  int g_match_pre_k;    // for each KF, search match with previous frames 
  void saveKFGraph(std::string);

  /** Does an offline optimization step. */
  void optimizeGraph();

  inline FrameSE3* getCurrentKeyframe() {return currentKeyFrame.get();}	// not thread-safe!

  /** Returns the current pose estimate. */
  SE3 getCurrentPoseEstimate();

  /** Sets the visualization where point clouds and camera poses will be sent to. */
  void setVisualization(Output3DWrapper* outputWrapper);

  void requestDepthMapScreenshot(const std::string& filename);

  bool doMappingIteration();

  // int findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool forceParent=true, bool useFABMAP=true, float closeCandidatesTH=1.0);
  int findConstraintsForNewKeyFrames(FrameSE3* newKeyFrame, bool forceParent=true, bool useFABMAP=true, float closeCandidatesTH=1.0);

  bool optimizationIteration(int itsPerTry, float minChange);

  void publishKeyframeGraph();

 //  std::vector<FramePoseStruct*, Eigen::aligned_allocator<lsd_slam::FramePoseStruct*> > getAllPoses();

  std::vector<FramePoseStructSE3*, Eigen::aligned_allocator<lsd_slam::FramePoseStructSE3*> > getAllPoses();



  float msTrackFrame, msOptimizationIteration, msFindConstraintsItaration, msFindReferences;
  int nTrackFrame, nOptimizationIteration, nFindConstraintsItaration, nFindReferences;
  float nAvgTrackFrame, nAvgOptimizationIteration, nAvgFindConstraintsItaration, nAvgFindReferences;
  struct timeval lastHzUpdate;


  private:
  protected:

  // ============= EXCLUSIVELY TRACKING THREAD (+ init) ===============
  // TrackingReference* trackingReference; // tracking reference for current keyframe. only used by tracking.
  TrackingReferenceSE3 * trackingReference;
  SE3Tracker* tracker;



  // ============= EXCLUSIVELY MAPPING THREAD (+ init) =============
  // DepthMap* map;
  // CDepthMap* map;
  // DepthMapSE3* map;
  CDepthMapSE3* map;
  // TrackingReference* mappingTrackingReference;
  TrackingReferenceSE3* mappingTrackingReference;

  // during re-localization used
  // std::vector<Frame*> KFForReloc;
  std::vector<FrameSE3*> KFForReloc;
  int nextRelocIdx;
  // std::shared_ptr<Frame> latestFrameTriedForReloc;
  std::shared_ptr<FrameSE3> latestFrameTriedForReloc;


  // ============= EXCLUSIVELY FIND-CONSTRAINT THREAD (+ init) =============
  // TrackableKeyFrameSearch* trackableKeyFrameSearch;
  TrackableKeyFrameSearchSE3* trackableKeyFrameSearch;

  Sim3Tracker* constraintTracker;
  SE3Tracker* constraintSE3Tracker;
  // TrackingReference* newKFTrackingReference;
  // TrackingReference* candidateTrackingReference;
  TrackingReferenceSE3* newKFTrackingReference;
  TrackingReferenceSE3* candidateTrackingReference;



  // ============= SHARED ENTITIES =============
  float tracking_lastResidual;
  float tracking_lastUsage;
  float tracking_lastGoodPerBad;
  float tracking_lastGoodPerTotal;

  int lastNumConstraintsAddedOnFullRetrack;
  bool doFinalOptimization;
  float lastTrackingClosenessScore;

  // for sequential operation. Set in Mapping, read in Tracking.
  boost::condition_variable  newFrameMappedSignal;
  boost::mutex newFrameMappedMutex;



  // USED DURING RE-LOCALIZATION ONLY
  Relocalizer relocalizer;



  // Individual / no locking
  Output3DWrapper* outputWrapper;	// no lock required
  // KeyFrameGraph* keyFrameGraph;	// has own locks
  KeyFrameGraphSE3* keyFrameGraph;


  // Tracking: if (!create) set candidate, set create.
  // Mapping: if (create) use candidate, reset create.
  // => no locking required.
  // std::shared_ptr<Frame> latestTrackedFrame;
  std::shared_ptr<FrameSE3> latestTrackedFrame;
  bool createNewKeyFrame;



  // PUSHED in tracking, READ & CLEARED in mapping
  // std::deque< std::shared_ptr<Frame> > unmappedTrackedFrames;
  std::deque< std::shared_ptr<FrameSE3> > unmappedTrackedFrames;
  boost::mutex unmappedTrackedFramesMutex;
  boost::condition_variable  unmappedTrackedFramesSignal;


  // PUSHED by Mapping, READ & CLEARED by constraintFinder
  // std::deque< Frame* > newKeyFrames;
  std::deque< FrameSE3* > newKeyFrames;
  boost::mutex newKeyFrameMutex;
  boost::condition_variable newKeyFrameCreatedSignal;


  // SET & READ EVERYWHERE
  // std::shared_ptr<Frame> currentKeyFrame;	// changed (and, for VO, maybe deleted)  only by Mapping thread within exclusive lock.
  // std::shared_ptr<Frame> trackingReferenceFrameSharedPT;	// only used in odometry-mode, to keep a keyframe alive until it is deleted. ONLY accessed whithin currentKeyFrameMutex lock.

  std::shared_ptr<FrameSE3> currentKeyFrame;	// changed (and, for VO, maybe deleted)  only by Mapping thread within exclusive lock.
  std::shared_ptr<FrameSE3> trackingReferenceFrameSharedPT;	// only used in odometry-mode, to keep a keyframe alive until it is deleted. ONLY accessed whithin currentKeyFrameMutex lock.
  boost::mutex currentKeyFrameMutex;



  // threads
  boost::thread thread_mapping;
  boost::thread thread_constraint_search;
  boost::thread thread_optimization;
  bool keepRunning; // used only on destruction to signal threads to finish.



  // optimization thread
  bool newConstraintAdded;
  boost::mutex newConstraintMutex;
  boost::condition_variable newConstraintCreatedSignal;
  boost::mutex g2oGraphAccessMutex;



  // optimization merging. SET in Optimization, merged in Mapping.
  bool haveUnmergedOptimizationOffset;

  // mutex to lock frame pose consistency. within a shared lock of this, *->getScaledCamToWorld() is
  // GUARANTEED to give the same result each call, and to be compatible to each other.
  // locked exclusively during the pose-update by Mapping.
  boost::shared_mutex poseConsistencyMutex;



  bool depthMapScreenshotFlag;
  std::string depthMapScreenshotFilename;


  /** Merges the current keyframe optimization offset to all working entities. */
  void mergeOptimizationOffset();


  void mappingThreadLoop();

  void finishCurrentKeyframe();
  void discardCurrentKeyframe();

  void changeKeyframe(bool noCreate, bool force, float maxScore);
  // void createNewCurrentKeyframe(std::shared_ptr<Frame> newKeyframeCandidate);
  // void loadNewCurrentKeyframe(Frame* keyframeToLoad);
  void createNewCurrentKeyframe(std::shared_ptr<FrameSE3> newKeyframeCandidate);
  void loadNewCurrentKeyframe(FrameSE3* keyframeToLoad);


  bool updateKeyframe();

  void addTimingSamples();

  void debugDisplayDepthMap();

  void takeRelocalizeResult();

  void constraintSearchThreadLoop();

  // only match with several previous frames 
  void constraintSearchThreadLoop_simple();  
  // void findConstraintsForNewKeyFrames_simple(Frame* newKeyFrame, bool forceParent, bool useFABMAP, float closeCandidatesTH);
  void findConstraintsForNewKeyFrames_simple(FrameSE3* newKeyFrame, bool forceParent, bool useFABMAP, float closeCandidatesTH);


  /** Calculates a scale independent error norm for reciprocal tracking results a and b with associated information matrices. */
  float tryTrackSim3(
      TrackingReference* A, TrackingReference* B,
      int lvlStart, int lvlEnd,
      bool useSSE,
      Sim3 &AtoB, Sim3 &BtoA,
      KFConstraintStruct* e1=0, KFConstraintStruct* e2=0);

  float tryTrackSE3(
      TrackingReferenceSE3* A, TrackingReferenceSE3* B,
      int lvlStart, int lvlEnd,
      bool useSSE,
      SE3 &AtoB, SE3 &BtoA,
      KFConstraintStructSE3* e1=0, KFConstraintStructSE3* e2=0);

  void testConstraint(
      Frame* candidate,
      KFConstraintStruct* &e1_out, KFConstraintStruct* &e2_out,
      Sim3 candidateToFrame_initialEstimate,
      float strictness);

  void testConstraintSE3(
      FrameSE3* candidate, KFConstraintStructSE3* &e1_out, KFConstraintStructSE3* &e2_out, 
      SE3 candidateToFrame_initialEstimate_sim3, 
      float strictness);

  void optimizationThreadLoop();
};




#endif
