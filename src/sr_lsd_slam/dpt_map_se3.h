#ifndef DPT_MAP_SE3_H
#define DPT_MAP_SE3_H
#include "util/EigenCoreInclude.h"
#include "opencv2/core/core.hpp"
#include "util/settings.h"
#include "util/IndexThreadReduce.h"
#include "util/SophusUtil.h"

namespace lsd_slam
{

class DepthMapPixelHypothesis;
class FrameSE3;
class KeyFrameGraphSE3;

}

using namespace lsd_slam; 

/**
 * Keeps a detailed depth map (consisting of CDepthMapPixelHypothesis) and does
 * stereo comparisons and regularization to update it.
 */
class CDepthMapSE3
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CDepthMapSE3(int w, int h, const Eigen::Matrix3f& K);
	CDepthMapSE3(const CDepthMapSE3&) = delete;
	CDepthMapSE3& operator=(const CDepthMapSE3&) = delete;
	~CDepthMapSE3();

	/** Resets everything. */
	void reset();
	
	/**
	 * does obervation and regularization only.
	 **/
	void updateKeyframe(std::deque< std::shared_ptr<FrameSE3> > referenceFrames);

	/**
	 * does propagation and whole-filling-regularization (no observation, for that need to call updateKeyframe()!)
	 **/
	void createKeyFrame(FrameSE3* new_keyframe);
	
	/**
	 * does one fill holes iteration
	 */
	void finalizeKeyFrame();

	void invalidate();
	inline bool isValid() {return activeKeyFrame!=0;};

	int debugPlotDepthMap();

	// ONLY for debugging, their memory is managed (created & deleted) by this object.
	cv::Mat debugImageHypothesisHandling;
	cv::Mat debugImageHypothesisPropagation;
	cv::Mat debugImageStereoLines;
	cv::Mat debugImageDepth;

	void initializeFromGTDepth(FrameSE3* new_frame);
	void initializeRandomly(FrameSE3* new_frame);

	void setFromExistingKF(FrameSE3* kf);

	void addTimingSample();
	float msUpdate, msCreate, msFinalize;
	float msObserve, msRegularize, msPropagate, msFillHoles, msSetDepth;
	int nUpdate, nCreate, nFinalize;
	int nObserve, nRegularize, nPropagate, nFillHoles, nSetDepth;
	struct timeval lastHzUpdate;
	float nAvgUpdate, nAvgCreate, nAvgFinalize;
	float nAvgObserve, nAvgRegularize, nAvgPropagate, nAvgFillHoles, nAvgSetDepth;



	// pointer to global keyframe graph
	IndexThreadReduce threadReducer;

private:
protected:
	// camera matrix etc.
	Eigen::Matrix3f K, KInv;
	float fx,fy,cx,cy;
	float fxi,fyi,cxi,cyi;
	int width, height;


	// ============= parameter copies for convenience ===========================
	// these are just copies of the pointers given to this function, for convenience.
	// these are NOT managed by this object!
	FrameSE3* activeKeyFrame;
	boost::shared_lock<boost::shared_mutex> activeKeyFramelock;
	const float* activeKeyFrameImageData;
	bool activeKeyFrameIsReactivated;

	FrameSE3* oldest_referenceFrame;
	FrameSE3* newest_referenceFrame;
	std::vector<FrameSE3*> referenceFrameByID;
	int referenceFrameByID_offset;

	// ============= internally used buffers for intermediate calculations etc. =============
	// for internal depth tracking, their memory is managed (created & deleted) by this object.
	DepthMapPixelHypothesis* otherDepthMap;
	DepthMapPixelHypothesis* currentDepthMap;
	int* validityIntegralBuffer;

	

	// ============ internal functions ==================================================
	// does the line-stereo seeking.
	// takes a lot of parameters, because they all have been pre-computed before.
	inline float doLineStereo(
			const float u, const float v, const float epxn, const float epyn,
			const float min_idepth, const float prior_idepth, float max_idepth,
			const FrameSE3* const referenceFrame, const float* referenceFrameImage,
			float &result_idepth, float &result_var, float &result_eplLength,
			RunningStats* const stats);


	void propagateDepth(FrameSE3* new_keyframe);
	

	void observeDepth();
	void observeDepthRow(int yMin, int yMax, RunningStats* stats);
	bool observeDepthCreate(const int &x, const int &y, const int &idx, RunningStats* const &stats);
	bool observeDepthUpdate(const int &x, const int &y, const int &idx, const float* keyFrameMaxGradBuf, RunningStats* const &stats);
	bool makeAndCheckEPL(const int x, const int y, const FrameSE3* const ref, float* pepx, float* pepy, RunningStats* const stats);


	void regularizeDepthMap(bool removeOcclusion, int validityTH);
	template<bool removeOcclusions> void regularizeDepthMapRow(int validityTH, int yMin, int yMax, RunningStats* stats);


	void buildRegIntegralBuffer();
	void buildRegIntegralBufferRow1(int yMin, int yMax, RunningStats* stats);
	void regularizeDepthMapFillHoles();
	void regularizeDepthMapFillHolesRow(int yMin, int yMax, RunningStats* stats);


	void resetCounters();

	//float clocksPropagate, clocksPropagateKF, clocksObserve, msObserve, clocksReg1, clocksReg2, msReg1, msReg2, clocksFinalize;
};


#endif

