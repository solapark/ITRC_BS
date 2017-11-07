#pragma once
#ifndef _CarSnukt_H_
#define _CarSnukt_H_

#include "Common.h"

/*
	@Brief: OpenCV - Window mouse interupt service function
*/
Void CallBackFunc(int event, int x, int y, int flags, void *userdata);

class CarSnukt {
private:
	// ============================================
	// ================	Variables  ================
	// ============================================

	Mat S[BGM_N];						// Statistic BGM candidates
	uint8_t S_Idx;						// The index of the static BGM candidates
	uint8_t S_U_Idx;					// The index of the oldest BGM candidate
	bool isFilledS;						// The flag that indicates the statistic BGM candidates are fully filled

	Mat Bs;								// The statistic BGM
	Mat B;								// The final background model	
	bool isBsAvai;						// The flag that indicates the statistic BGM is available or not
	bool isBAvai;						// The flag that indicates the final BGM is available or not
	bool isBLoaded;						// The flag that indicates the BGM is loaded or not

	Ptr<RTrees> RTMVOClassifier;		// Random tree MVO classifier model pointer
	Ptr<ANN_MLP> MLPMVOClassifier;		// Neural net MVO classifier model pointer
	int32_t ImHorSize;					// The horizontal size of the input image
	int32_t ImVerSize;					// The vertical size of the input image

	Mat FG;								// Foreground 						
	Mat MVOSH;							// MVO related mask: MVO and MVO'shadow
	Mat MVO;							// MVO mask
	Mat SH;								// MVO'shadow mask
	Mat MVOGSH;							// Ghost related mask: ghost and ghost's shadow

	Mat ROI;							// ROI mask
	Point2i ROI_BL;						// The Bottom-Left point of the ROI mask
	Point2i ROI_BR;						// The Bottom-Right point of the ROI mask
	Point2i ROI_TR;						// The Top-Right point of the ROI mask
	Point2i ROI_TL;						// The Top-Left point of the ROI mask

	Point2i ROI_iBL;					// The Bottom-Left point of the inROI mask
	Point2i ROI_iBR;					// The Bottom-Right point of the inROI mask
	Point2i ROI_iTR;					// The Top-Right point of the inROI mask
	Point2i ROI_iTL;					// The Top-Left point of the inROI mask

	float ROI_iBL_BR_Param[3];			// The parameter of the line formed by BL and BR points
	float ROI_iBR_TR_Param[3];			// The parameter of the line formed by BR and TR points
	float ROI_iTR_TL_Param[3];			// The parameter of the line formed by TR and TL points
	float ROI_iTL_BL_Param[3];			// The parameter of the line formed by TL and BL points

	int NewTrackObj;					// New track object ID
	int32_t	vID;						//virtual MVO ID for ITRC project

	// ============================================
	// ================= Structs  =================
	// ============================================


	struct TrackedObject				// The structs that store all principle information of tracked large-MVOs
	{
		KalmanFilter KF;
		Mat PosMeasure;
		Mat ROI;
		Mat b_hist, g_hist, r_hist;
		Point2i HisPos[HIS_POS_SIZE];
		vector<Point2i>CriticPntsVec;
		Point2i CenterImgPlane;
		Point2f CenterTrans;
		Point2i Head;
		Point2i Tail;
		Mat TransROI;
		uint8_t SoftID;
		uint8_t HardID;
		int32_t vID;
		uint32_t NumOfHisPt;

		camToCar dataCamToCar;
	};


	Mat TransBGM;
	Mat TransMat;

	timeStamp time;

	// ============================================
	// ================	Functions  ================
	// ============================================

	/*
		@Brief: update the statistic BGM candidates
		*/
	inline Void UpdateS(Mat &I);

	/*
		@Brief: update the statistic BGM candidates
		*/
	inline Void UpdateBs(void);

	/*
		@Brief: Roughly segment the nonzero pixels after background substraction into groups of moving objects
		*/
	inline Bool NonZeroSeg(Mat &BinImg, vector<Mat> &ROI, vector<Mat> &SEG, vector<int> &SumNonZeroPix);

	/*
		@Brief: Densely segment the nonzero pixels after background substraction into groups of moving objects
		*/
	inline Bool NonZeroSegTwice(Mat &BinImg, vector<Mat> &ROI, vector<Mat> &SEG, vector<int> &SumNonZeroPix);

	/*
		@Brief: The refinement process applied for a segmented MVO, which remove all zero colums and zero rows at the outer
		parts of the MVO matrices
		*/
	inline Bool RefineDetecedMVO(Mat &MVO_SEG, Mat &MVO_ROI);

	/*
		@Brief: Apply the background subtraction, ROI filter mask and a Gaussian filter. After this step, a foreground
		matrix which contains ghosts, real moving objects and their shadows
		*/
	inline Void BGSuppress(Mat &I, Mat &B, Mat &FG);

	/*
		@Brief: Remove all ghost objects of the foreground mask
		*/
	inline Void GhostDet(Mat &I, Mat &lastI, Mat &FG, Mat &MVOSH, Mat &MVOGSH);

	/*
		@Brief: Remove all shadow pixels of the detected MVOs
		*/
	inline Void ShadowDet(Mat &I, Mat &B, Mat &MVOSH, vector<Mat> &MVO_ROI, vector<Mat> &MVO_SEG);

	/*
		@Brief: Once all objects in the foreground mask are determined, using this background update technique
		could improve the accuracy of the final BGM noticeablely but it demands quite large amount of computation.
		*/
	inline Void BGMKnowledgedBasedUpdate(Mat &I, Mat &Bs, bool &isBAvai, Mat &B, Mat &MVOGSH);

	/*
		@Brief: Estimate the orientation of the MVO then detect critical points that represent the object bottom
		*/
	inline Void DetectCriticalPoint(Mat &MVO_SEG, Mat &MVO_ROI, vector<Point2i> &CriticalPnts, Point2f &BaseLineVec);

	/*
		@Brief:
		*/
	inline Float FindPhanGiacY(Float a[3], Float b[3], Float c[3], float x);

	/*
		@Brief: Check a MVO is within the ROI or not
		*/
	inline Bool CheckInsideROI(Mat &MVO_ROI);

	/*
		@Brief: 1. Check the MVO is in the ROI or not
		2. If yes, classify this MVO (small-scale or large-scale MVOs)
		*/
	inline Void LargeMVODetection(Mat &I, vector<Mat> &MVO_SEG, vector<Mat> &MVO_ROI, vector<bool> &isLargeObject);

	/*
		@Brief: Update the historical position of a tracking object
		*/
	inline Void UpdateHisPos(Point2i HisPos[HIS_POS_SIZE], Point2i NewPoint);

	/*
		@Brief: Calculate the color histogram of a object
		*/
	inline Void MVOCalColorHistogram(Mat &MVO, Mat &b_hist, Mat &g_hist, Mat &r_hist);

	/*
		@Brief: Create a new track object
		*/
	inline Void CreateNewTrackObjt(Mat &I, Mat &curSeg, Mat &curROI);

	/*
		@Brief: Transform the critical points to a map
		*/
	inline Void TransformCriticalPoints(vector<Point2i> &CriticPntsVec, Mat &TransROI);

	/*
		@Brief: Refine the MVO's ROIs of the small objects after tracking process
		*/
	inline Void SmallROIRefine(vector<bool> &isLargeObject, vector<Mat> &MVO_ROI, vector<Mat> &SmallObjectROI);

	/*
		@Brief:
		*/
	inline Point2f TransformPoint(Point2f curPoint);

	/*
		@Brief:	LargeMVOTracking
		1. Perform tracking for large MVOs
		2. Also refine detection and classification results
		*/
	inline Void LargeMVOTracking(Mat &I,
		vector<Mat> &MVO_SEG,
		vector<Mat> &MVO_ROI,
		vector<bool> &isLargeObject,
		vector<int> &hardIdCode);

	/*
		@Brief:	Annotation
		*/
	inline Void Annotation(Mat &I, vector<Mat> &SmallObjectROI);

	inline Void prepareSendData();


protected:
public:

	// ============================================
	// ================	Variables  ================
	// ============================================

	TrackedObject TrackObj[LIVE_OBJECT_SIZE];		// The structs that store all priciple information of tracked large MVOs
	Mat LiveObjList;								// The indices of current live tracking objects
	vector<Mat> SmallObjectROI;						// The bounding boxes of the samll objects
	
	// ============================================
	// ================	Functions  ================
	// ============================================

	/*
		Init
		*/
	CarSnukt(void);

	/*
		@Brief: store the background image
		*/
	Void StoreBG(void);

	/*
		@Brief: load the background from a saved image
		*/
	Void LoadBG(Mat firstFrame);
	Void LoadBG(void);
	bool isBGLoaded(void){ return isBLoaded; }

	/*
		@Brief: Update the background model
		*/
	Void UpdateBGM(Mat &I);

	/*
		@Brief: Get the BGM status
		*/
	Bool GetBGMStatus(void) { return (isBsAvai || isBAvai); }

	/*
		@Brief: Get the statistic BGM
		*/
	Mat GetBs(void) { return Bs; }

	/*
		@Brief: Initialize the CarSnukt detector
		*/
	Void Initialize(uint32_t ImageVerSize, uint32_t ImageHorSize);

	/*
		@Brief: Picking the ROI region as well as the perspective transformation region
		*/
	Void InputROIandPersMap(Mat &tmpI);

	/*
		@Brief: Form the ROI parameters given by 4 polar points
		*/
	Void FormROI(Point2i BL, Point2i BR, Point2i TR, Point2i TL);

	/*
		@Brief: Form the top-down perspective background for displaying
		*/
	Void FormTransBGM(const Mat &I, Point2f PerBL, Point2f PerBR, Point2f PerTR, Point2f PerTL, int w, int h);

	/*
		@Brief: The packed function for CarSnukt-based detector
		It's output is the structure: CarSnuktOutp that is used for the mapping process.
		*/
	Void CarSnuktDet(Mat &I, Mat &lastI);

	Void changeToPers(Mat &I);

	Int getDataToSend(vector<camToCar> & vecCamToCar) const;

	//time stamp
	Void updateT(const SYSTEMTIME &t);
};
#endif

