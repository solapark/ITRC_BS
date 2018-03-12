#include "stdafx.h"
#include "CarSnukt.h"
Void CallBackFunc(int event, int x, int y, int flags, void *userdata)
{
	if (event == EVENT_LBUTTONDOWN){
		cout << "Left button clicked pos x: " << x << ", pos y: " << y << endl;
		//vector<Point2i> *p = (vector<Point2i> *) userdata;
		//p->push_back(Point2i(x, y));
		Point2i *p = (Point2i *)userdata;
		p->x = x;
		p->y = y;
	}

	if (event == EVENT_LBUTTONDOWN && flags == EVENT_FLAG_CTRLKEY){
		cout << "Left button clicked pos x: " << x << ", pos y: " << y << endl;
		//vector<Point2i> *p = (vector<Point2i> *) userdata;
		//p->push_back(Point2i(x, y));
		Point2i *p = (Point2i *)userdata;
		p->x = x;
		p->y = y;
	}

	if (event == EVENT_RBUTTONDOWN){
		cout << "Right button clicked " << endl;
		//vector<Point2i> *p = (vector<Point2i> *) userdata;
		//p->push_back(Point2i(x, y));
	}
}

CarSnukt::CarSnukt() 
#if AUTO_CAR_DETECTION
	: colDet(LOW_H, HIGH_H, LOW_S, HIGH_S, LOW_V, HIGH_V, REFINE)
#endif
{
#if DETECTOR_YOLO
	pYolo = new Detector(CFG_FILE, WEIGHT_FILE);
#endif

	// BGM
	isBsAvai = false;
	isBAvai = false;
	isBLoaded = false;
	isFilledS = false;
	S_Idx = 0;
	S_U_Idx = 0;

	// MVO Tracking
	LiveObjList = Mat::zeros(1, LIVE_OBJECT_SIZE, CV_8UC1);

	NewTrackObj = 0;
	vID = MVO_VIRTUAL_ID_START;

	for (int i = 0; i < LIVE_OBJECT_SIZE; i++) {
		TrackObj[i].gpsVel.setGpsVelocity(LAT_PRECISION, LON_PRECISION, VEL_PRECISION, VELOCITY_LIMIT);
	}

#if AUTO_CAR_DETECTION
	// Auto Car Tracking
	LiveAutoCarList = Mat::zeros(1, LIVE_AUTO_CAR_SIZE, CV_8UC1);
	
	NewAutoTrackObj = 0;
	autoVID = AUTO_CAR_VIRTUAL_ID_START;
	for (int i = 0; i < LIVE_AUTO_CAR_SIZE; i++) {
		autoCar[i].gpsVel.setGpsVelocity(LAT_PRECISION, LON_PRECISION, VEL_PRECISION, VELOCITY_LIMIT);
	}
#endif
}

//need to study -sola
inline Void CarSnukt::UpdateS(Mat &I)
{ 
#if BGM_KNOWLEDGE
	if (S_Idx < (BGM_N - 1))
	{
		S[S_Idx] = I;
		S_Idx++;
		isFilledS = false;
	}
	else
	{
		// the last candidate of S is the current BGM		
		if ((!isBAvai) && (!isBsAvai)) {
			S[BGM_N - 1] = S[0];
		}
		else if (isBsAvai) {
			S[BGM_N - 1] = Bs;
		}
		else if (isBAvai) {
			S[BGM_N - 1] = B;
		}

		// update the oldest candidate 
		S[S_U_Idx] = I;
		S_U_Idx++;
		S_U_Idx = S_U_Idx % (BGM_N - 1);
		isFilledS = true;
	}
#else
	if (S_Idx < BGM_N)
	{
		S[S_Idx] = I;
		S_Idx++;
		isFilledS = false;
	}
	else
	{
		// the last candidate of S is the current BGM		
		if ((!isBAvai) && (!isBsAvai)) {
			S[BGM_N - 1] = S[0];
		}
		else if (isBsAvai) {
			S[BGM_N - 1] = Bs;
		}
		else if (isBAvai) {
			S[BGM_N - 1] = B;
		}

		// update the oldest candidate 
		S[S_U_Idx] = I;
		S_U_Idx++;

		S_U_Idx = S_U_Idx % (BGM_N - 1);

		isFilledS = true;
	}
#endif
}

//need to study -sola
Void CarSnukt::UpdateBs()
{
	// Update the Statistic BGM
	if (isFilledS)
	{
		// calculate the sum of distances from a BGM Bs[i] to other BGM Bs[j] candidates
		Mat SumDist[BGM_N];
		for (uint8_t i = 0; i < BGM_N; i++)
		{
			Mat Bs_i = S[i];
			Mat tmpSumDist = Mat::zeros(S[0].rows, S[0].cols, CV_32FC1);
			//imshow("Bs_i",Bs_i);
			//waitKey();
			for (uint8_t j = 0; j < BGM_N; j++)
			{
				if (i != j) {
					Mat Bs_j = S[j];
					Mat diff = Bs_i - Bs_j;
					Mat channels[3];
					diff = diff.mul(diff);
					if (j == (BGM_N - 1))
					{
						diff = diff.mul(BGM_WB);
					}
					split(diff, channels);
					tmpSumDist = tmpSumDist + channels[0] + channels[1] + channels[2];
				}
			}
			SumDist[i] = tmpSumDist;
		}

		// Pixel-wise statistic BGM update
		Mat tmpBs = Mat::zeros(S[0].rows, S[0].cols, CV_32FC3);
		for (int i = 0; i < S[0].rows; i++) {
			for (int j = 0; j < S[0].cols; j++) {
				// find the minimum distance
				float tmpMinPixDist = (float)LONG_MAX;
				uint8_t minIdx = 0;
				for (uint8_t k = 0; k < BGM_N; k++) {
					float curDist = SumDist[k].at<float>(i, j);
					if (curDist < tmpMinPixDist) {
						tmpMinPixDist = curDist;
						minIdx = k;
					}
				}
				// use the pixel with the minimum distance to update the statistical BGM
				tmpBs.at<Vec3f>(i, j) = S[minIdx].at<Vec3f>(i, j);
			}
		}

		// Gaussian filter applied for the statistic BGM
		GaussianBlur(tmpBs, tmpBs, cv::Size(5, 5), 0.3);

#if DEBUG_BKG_UPDATE
		imshow("Bs", tmpBs);
		printf("Updated new Bs, press any key ...\n");
		waitKey();
#endif

		// coppy to the class statistic BGM and update the Bs available flag
		Bs = tmpBs;
		isBsAvai = true;
	}
}

Void CarSnukt::StoreBG()
{
#if SAVE_NEW_BG
	bool do_once = false;
#else
	static bool do_once = false;
#endif

	if (!do_once){
//		printf("Do you want to store the current background ? y/n \n");
#if SAVE_NEW_BG
		imshow("BS save", Bs);
		waitKey();
#endif
		string input;
		cin >> input;
		if (input == "y"){
			Mat tmpBs = Bs;
			tmpBs.convertTo(tmpBs, CV_32FC3, 255.0);
			imwrite(BG_FILE, tmpBs);
			B = Bs;
		}
		do_once = true;
	}
}

Void CarSnukt::LoadBG(Mat firstFrame)
{
	Mat BG = firstFrame;
	if (BG.rows != SIZE_VER || BG.cols != SIZE_HOR) {
		cout << "BG size != img size" << endl;
		resize(BG, BG, Size(SIZE_HOR, SIZE_VER));
		//resize(BG, BG, Size(SIZE_HOR, SIZE_VER));
	}
	if (!BG.empty())
	{
//		imshow("BackGround Img", BG);
//		printf("Do you want to load BG image ? y/n \n");
//		destroyWindow("BackGround Img");
		string input = "y";
		//cin >> input;
		if (input == "y")
		{
			BG.convertTo(BG, CV_32FC3, 1 / 255.0);
			Bs = BG;
			isBsAvai = true;
			isBLoaded = true;
		}
	}
}

Void CarSnukt::LoadBG()
{
	Mat BG = imread(BG_FILE);
	if (BG.rows != SIZE_VER || BG.cols != SIZE_HOR) {
		cout << "BG size != img size" << endl;
		resize(BG, BG, Size(SIZE_HOR, SIZE_VER));
		//resize(BG, BG, Size(SIZE_HOR, SIZE_VER));
	}
	if (!BG.empty())
	{
		cv::imshow("BackGround Img", BG);
		cv::waitKey();
		std::printf("Do you want to load BG image ? y/n \n");
		destroyWindow("BackGround Img");
		string input = "y";
		//cin >> input;
		if (input == "y")
		{
			BG.convertTo(BG, CV_32FC3, 1 / 255.0);
			Bs = BG;
			isBsAvai = true;
			isBLoaded = true;
		}
	}
}

Void CarSnukt::FormTransBGM(const Mat &I, Point2f PerBL, Point2f PerBR, Point2f PerTR, Point2f PerTL, int w, int h)
{
	Point2f dst_p[4], map_p1[4];

	dst_p[0] = PerBL;
	dst_p[1] = PerBR;
	dst_p[2] = PerTR;
	dst_p[3] = PerTL;

	map_p1[0] = Point2f(0, 0);
	map_p1[1] = Point2f((float)w, 0);
	map_p1[2] = Point2f((float)w, (float)h);
	map_p1[3] = Point2f(0, (float)h);

	TransMat = getPerspectiveTransform(dst_p, map_p1);

	warpPerspective(I, TransBGM, TransMat, Size(w, h));
	//cout << TransBGM.size() << endl;
	//imshow("TransBGM", TransBGM);
	//waitKey();
}

inline Bool CarSnukt::NonZeroSeg(Mat &BinImg, vector<Mat> &ROI, vector<Mat> &SEG, vector<int> &SumNonZeroPix)
{
	/*
		Non-zero segmentation for binary image
		*/
	vector<Point2i> VerSeg;
	vector<Mat> Segment;
	vector<Mat> SegmentROI;
	vector<int> SumOfNonZeroPix;
	bool isValid = false;
	Mat locations;
	int NonZero = countNonZero(BinImg);
	if (NonZero > 0)
	{
		findNonZero(BinImg, locations);
		int LocSize = locations.total();
		//for (int i = 0; i < LocSize; i++) {
		//	//circle(BinImg, locations.at(i), 1, Scalar(0, 0, 0));
		//	cout << locations.at(i).x << ", " << locations.at(i).y << endl;
		//	//imshow("location", BinImg);
		////	waitKey();
		//}

		// Find the vertical segmentations
		Point2i tmpVerSeg(locations.at<Point2i>(0).y, 0);
		for (int i = 0; i < LocSize - 1; i++)
		{
			int curY = locations.at<Point2i>(i).y;
			int nextY = locations.at<Point2i>(i + 1).y;
			if ((curY + SEGMIN) < nextY) // new segmentation condition
			{
				//cout << "new seg gen" << endl;
				tmpVerSeg.y = curY;
//				if ((tmpVerSeg.x + SEGMIN) < tmpVerSeg.y)
				if ((tmpVerSeg.x + BBMIN) < tmpVerSeg.y)
				{
					//cout << "push tmpVerseg" << endl;
					VerSeg.push_back(tmpVerSeg);
					assert(countNonZero(BinImg.row(tmpVerSeg.x)) > 0);		// ok
					assert(countNonZero(BinImg.row(tmpVerSeg.y)) > 0);		// ok					
				}
				tmpVerSeg.x = nextY;
			}
		}
		tmpVerSeg.y = locations.at<Point2i>(LocSize - 1).y;
		if ((tmpVerSeg.x + SEGMIN) < tmpVerSeg.y)
		{
			VerSeg.push_back(tmpVerSeg);
			assert(countNonZero(BinImg.row(tmpVerSeg.x)) > 0);		// ok
			assert(countNonZero(BinImg.row(tmpVerSeg.y)) > 0);		// ok					
		}

		// Find the horizontal segmentations inside the detected vertical segmentation
		if (!VerSeg.empty())
		{
#if DEBUG_NONZ_SEG
			cout << "VerSeg.size() : "<< VerSeg.size() << endl;
#endif
		
			for (int verSegIdx = 0; verSegIdx < VerSeg.size(); verSegIdx++)
			{
				Point2i curVerSeg = VerSeg.at(verSegIdx);
				// split to the smaller images				
				Mat tVerSeg = BinImg(Range(curVerSeg.x, curVerSeg.y + 1), Range::all()); // Caution: Range[a,a+x]: a start index, x the number of elements
				assert(countNonZero(tVerSeg.row(0))>0);						// ok
				assert(countNonZero(tVerSeg.row(tVerSeg.rows - 1)) > 0);	// ok
				// sort the horizontal indices
				Mat VerSegLocs;
				findNonZero(tVerSeg.t(), VerSegLocs);
				// complete segmentation inside the current vertical segmentation
				Point2i tmpHorSeg(VerSegLocs.at<Point2i>(0).y, 0);
				for (int HorPosIdx = 0; HorPosIdx < VerSegLocs.total() - 1; HorPosIdx++)
				{
					int curX = VerSegLocs.at<Point2i>(HorPosIdx).y;
					int nextX = VerSegLocs.at<Point2i>(HorPosIdx + 1).y;
					if ((curX + SEGMIN) < nextX)		// new segmentation condition
					{
						tmpHorSeg.y = curX;
						int w = tmpHorSeg.y - tmpHorSeg.x + 1;
						int h = curVerSeg.y - curVerSeg.x + 1;
						if (h > BBMIN && w > BBMIN)
						{
							//printf("new seg = [%d %d %d %d]\n", tmpHorSeg.x, tmpHorSeg.y, curVerSeg.x, curVerSeg.y);
							Mat tmpSeg = BinImg(Range(curVerSeg.x, curVerSeg.y + 1), Range(tmpHorSeg.x, tmpHorSeg.y + 1));
							Mat tmpSegROI = (Mat_<int>(1, 4) << tmpHorSeg.x, tmpHorSeg.y, curVerSeg.x, curVerSeg.y);
							uint32_t SegNonZ = countNonZero(tmpSeg);
							// check the horizontal segmentation
							assert(countNonZero(BinImg.col(tmpHorSeg.x)) > 0);		// ok
							assert(countNonZero(BinImg.col(tmpHorSeg.y)) > 0);		// ok
							assert(countNonZero(tmpSeg.col(0)) > 0);				// ok
							assert(countNonZero(tmpSeg.col(tmpSeg.cols - 1)) > 0);	// ok
							if (SegNonZ > OT) {
								Segment.push_back(tmpSeg);
								SegmentROI.push_back(tmpSegROI);
								SumOfNonZeroPix.push_back(SegNonZ);
							}
						}
						tmpHorSeg.x = nextX;
					}
				}
				tmpHorSeg.y = VerSegLocs.at<Point2i>(VerSegLocs.total() - 1).y;
				int w = tmpHorSeg.y - tmpHorSeg.x + 1;
				int h = curVerSeg.y - curVerSeg.x + 1;
				if (h > BBMIN && w > BBMIN)
				{
					//printf("new seg = [%d %d %d %d]\n", tmpHorSeg.x, tmpHorSeg.y, curVerSeg.x, curVerSeg.y);
					Mat tmpSeg = BinImg(Range(curVerSeg.x, curVerSeg.y + 1), Range(tmpHorSeg.x, tmpHorSeg.y + 1));
					Mat tmpSegROI = (Mat_<int>(1, 4) << tmpHorSeg.x, tmpHorSeg.y, curVerSeg.x, curVerSeg.y);
					uint32_t SegNonZ = countNonZero(tmpSeg);
					// check the horizontal segmentation
					assert(countNonZero(BinImg.col(tmpHorSeg.x)) > 0);		// ok
					assert(countNonZero(BinImg.col(tmpHorSeg.y)) > 0);		// ok
					assert(countNonZero(tmpSeg.col(0)) > 0);				// ok
					assert(countNonZero(tmpSeg.col(tmpSeg.cols - 1)) > 0);	// ok
					if (SegNonZ > OT) {
						Segment.push_back(tmpSeg);
						SegmentROI.push_back(tmpSegROI);
						SumOfNonZeroPix.push_back(SegNonZ);
					}
				}
			}

			if (!Segment.empty())
			{
				ROI = SegmentROI;
				SEG = Segment;
				SumNonZeroPix = SumOfNonZeroPix;
#if DEBUG_NONZ_SEG
				Mat3b grayRGB;
				cvtColor(BinImg, grayRGB, COLOR_GRAY2BGR);

				// draw all the detected bounding boxes
				for (int i = 0; i < SEG.size(); i++)
				{
					Mat CurROI = ROI.at(i);
					Rect rect(CurROI.at<int>(0),
						CurROI.at<int>(2),
						CurROI.at<int>(1) - CurROI.at<int>(0),
						CurROI.at<int>(3) - CurROI.at<int>(2)
						);
					rectangle(grayRGB, rect, Scalar(0, 255, 0), 1);
				}
				imshow("grayRGB", grayRGB);
				waitKey();
#endif
				isValid = true;
			}
#if DEBUG_NONZ_SEG
			else {
				cout << "non_zero_seg() : horseg empty" << endl;
				imshow("non_zero_seg() : horseg empty", BinImg);
				waitKey();
			}
#endif
		}
#if DEBUG_NONZ_SEG
		else {
			cout << "non_zero_seg() : verseg empty" << endl;
			imshow("non_zero_seg() : verseg empty", BinImg);
			waitKey();
		}
#endif
	}
#if DEBUG_NONZ_SEG
	else {
		cout << "non_zero_seg() : countNonZero<0 " << endl;
		imshow("non_zero_seg() : countNonZero<0 ", BinImg);
		waitKey();
	}
#endif
	return isValid;
}

inline Bool CarSnukt::NonZeroSegTwice(Mat &BinImg, vector<Mat>& ROI, vector<Mat>& SEG, vector<int> &SumNonZeroPix)
{
	//vector<Mat> ROI;
	//vector<Mat> SEG;
	vector<Mat> ROI1;
	vector<Mat> SEG1;
	vector<int> SNonZero1;
	vector<Mat> ROI2;
	vector<Mat> SEG2;
	vector<int> SNonZero2;
	bool isValid = false;
	if (NonZeroSeg(BinImg, ROI1, SEG1, SNonZero1))
	{
		//cout << "NonZeroSeg done" << endl;
		for (int i = 0; i < ROI1.size(); i++)
		{
			Mat curROI1 = ROI1.at(i);
			Mat curSeg = SEG1.at(i);
			ROI2.clear();
			SEG2.clear();
			SNonZero2.clear();
			if (NonZeroSeg(curSeg, ROI2, SEG2, SNonZero2))
			{
				// Shift to the Image co-ordinate
				for (int j = 0; j < ROI2.size(); j++)
				{
					Mat curROI2 = ROI2.at(j);
					Mat curSEG2 = SEG2.at(j);
					int curNonZPixel = SNonZero2.at(j);

					int xMinI = MAX(curROI1.at<int>(0) + curROI2.at<int>(0), 0);
					int xMaxI = MIN(curROI1.at<int>(0) + curROI2.at<int>(1), BinImg.cols - 1);
					int yMinI = MAX(curROI1.at<int>(2) + curROI2.at<int>(2), 0);
					int yMaxI = MIN(curROI1.at<int>(2) + curROI2.at<int>(3), BinImg.rows - 1);

					Mat tmpROI = (Mat_<int>(1, 4) << xMinI, xMaxI, yMinI, yMaxI);
					ROI.push_back(tmpROI);
					SEG.push_back(curSEG2);
					SumNonZeroPix.push_back(curNonZPixel);
					//printf("Global ROI = [%d %d %d %d]\n", xMinI, xMaxI, yMinI, yMaxI);
				}
			}
#if DEBUG_NONZ_SEG
			else {
				cout << "second non-zero seg = 0" << endl;
			}
#endif
		}
	}
#if DEBUG_NONZ_SEG
	else {
		cout << "first non-zero seg = 0" << endl;
	}
#endif

	// Debug
	if (!SEG.empty())
	{
		isValid = true;
#if DEBUG_NONZ_SEG_TWICE
		Mat3b grayRGB;
		cvtColor(BinImg, grayRGB, COLOR_GRAY2BGR);

		// draw all the detected bounding boxes
		for (int i = 0; i < SEG.size(); i++)
		{
			Mat CurROI = ROI.at(i);
			Mat CurSeg = SEG.at(i);
			Rect rect(MAX(CurROI.at<int>(0) - 2, 0),
				MAX(CurROI.at<int>(2) - 2, 0),
				MIN(CurROI.at<int>(1) - CurROI.at<int>(0) + 4, BinImg.cols - 1),
				MIN(CurROI.at<int>(3) - CurROI.at<int>(2) + 4, BinImg.rows - 1)
				);
			rectangle(grayRGB, rect, Scalar(0, 255, 0), 1);
			char str[200];
			sprintf(str, "[%d , %d, %d]", rect.height, rect.width, countNonZero(CurSeg));
			putText(grayRGB, str, Point2i(rect.x, rect.y), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255, 255), 1);
		}
		imshow("Segmented objects", grayRGB);
		waitKey(1);
#endif
	}
	return isValid;
}

inline Void CarSnukt::BGSuppress(Mat &I, Mat &B, Mat &FG)
{
//	imshow("B", B);
//	waitKey();
//	cout << "I.rows I.cols B.rows B.cols : "<< I.rows << ", " << I.cols << ", " << B.rows << ", " << B.cols << endl;
	Mat DB = abs(I - B);
	DB = DB.mul(DB);
	//vector<Mat> channels(3);
	Mat channels[3];
	split(DB, channels);
	FG = channels[0] + channels[1] + channels[2];
	//imshow("FG bf roi check", FG);
	FG = FG.mul(ROI);
	GaussianBlur(FG, FG, cv::Size(5, 5), 0.8);
	inRange(FG, TL_MIN, TL_MAX, FG);
#if DEBUG_SUB
	imshow("FG", FG);
	waitKey(1);
#endif
}

//need to study -sola
inline Void CarSnukt::GhostDet(Mat &I, Mat &lastI, Mat &FG, Mat &MVOSH, Mat &MVOGSH)
{
	//vector<Mat> channels(3);
	Mat channels[3];
	Mat MS = abs(I - lastI);
	MS = MS.mul(MS);
	split(MS, channels);
	Mat NonGH = channels[0] + channels[1] + channels[2];
	inRange(NonGH, MVT_MIN, MVT_MAX, NonGH);
	MVOSH = FG.mul(NonGH);	// For shadow detection
	MVOGSH = FG - MVOSH;	// Ghost related mask (For background update)
	GaussianBlur(MVOSH, MVOSH, Size(5, 5), 0.3);
	inRange(MVOSH, 100, SHRT_MAX, MVOSH);
#if DEBUG_SUB
	imshow("FG af ghst rm", MVOSH);
	waitKey();
#endif
}

//need to study -sola
inline Void CarSnukt::ShadowDet(Mat &I, Mat &B, Mat &MVOSH, vector<Mat> &MVO_ROI, vector<Mat> &MVO_SEG)
{
	/*
		Shadow detection
		@Brief:		Find the shadow pixel of the detected MVOGS bin image
		@Input:		The detected MVOGS segmentations resulted after the background suppresion method
		@Output:	The ROI of MVOs and their binary image segmentation
		The MVO's shadow mask
		*/
	vector<Mat> MVOSH_ROI;
	vector<Mat> MVOSH_SEG;
	vector<int> MVOSH_nonZPixl;
#if	DEBUG_RAW_SEG_SAL_N_SIZE
	Mat Icopy = I;
#endif
	if (NonZeroSegTwice(MVOSH, MVOSH_ROI, MVOSH_SEG, MVOSH_nonZPixl))
	{
		//cout << "NonZeroSegTwice done" << endl;
		//cout << "NonZeroSegTwice size : " << MVOSH_ROI.size() << endl;
		for (int segIdx = 0; segIdx < MVOSH_ROI.size(); segIdx++)
		{
			Mat curROI = MVOSH_ROI.at(segIdx);
			Mat curSeg = MVOSH_SEG.at(segIdx);
			// check the saliency and insideROI conditions

			if (MVOSH_nonZPixl.at(segIdx) > SAL &&
				curSeg.cols > SIZE &&
				curSeg.rows > SIZE)
			{
#if DEBUG_SHADOW_DET
				// debug
				namedWindow("curSeg", WINDOW_NORMAL);
				imshow("curSeg", curSeg);
#endif
#if SHADOW_REMOVAL
				if (MVOSH_nonZPixl.at(segIdx) > SHT && ((curSeg.cols > (SIZE << 1)) || (curSeg.rows > (SIZE << 1))))
				{
					int xMinI = curROI.at<int>(0); assert(xMinI > 0 || xMinI == 0);
					int xMaxI = curROI.at<int>(1); assert(xMaxI > xMinI); assert(xMaxI < I.cols);
					int yMinI = curROI.at<int>(2); assert(yMinI > 0 || yMinI == 0);
					int yMaxI = curROI.at<int>(3); assert(yMaxI > yMinI); assert(yMaxI < I.rows);
					Mat tmpSH = Mat::zeros(curSeg.rows, curSeg.cols, CV_32FC1);
					Mat segI_RGB = I(Range(yMinI, yMaxI + 1), Range(xMinI, xMaxI + 1));
					Mat segB_RGB = B(Range(yMinI, yMaxI + 1), Range(xMinI, xMaxI + 1));
					Mat SegI_HSV; cvtColor(segI_RGB, SegI_HSV, CV_32F, CV_RGB2HSV);
					Mat SegB_HSV; cvtColor(segB_RGB, SegB_HSV, CV_32F, CV_RGB2HSV);
					// find the shadow pixels 
					Mat NonZPixel;
					findNonZero(curSeg, NonZPixel);
					for (int nonZPixIdx = 0; nonZPixIdx < NonZPixel.total(); nonZPixIdx++)
					{
						Point2i p = NonZPixel.at<Point2i>(nonZPixIdx);
						Vec3f pI_HSV = SegI_HSV.at<Vec3f>(p);
						Vec3f pB_HSV = SegB_HSV.at<Vec3f>(p);
						tmpSH.at<float>(p) = (pI_HSV[0] + 0.0001f) / (pB_HSV[0] + 0.0001f);
					}
					inRange(tmpSH, ALPHA, BETA, tmpSH);
					curSeg = curSeg - tmpSH;

					// smoothen the non-shadow objects					
					GaussianBlur(curSeg, curSeg, cv::Size(3, 3), 0.5);
					inRange(curSeg, GFTHR_LOW, GFTHR_HIGH, curSeg);
				}
#endif

				// refine the MVO
				bool isRefined = RefineDetecedMVO(curSeg, curROI);
				//cout << "isRefined : "<< isRefined << endl;
				// check inside the ROI
#if CHECK_INSIDE_ROI
				bool isInROI = CheckInsideROI(curROI);
#else
				bool isInROI = 1;
#endif

#if DEBUG_SHADOW_DET
				// debug
				namedWindow("outCurSeg", WINDOW_NORMAL);
				imshow("outCurSeg", curSeg);	
				line(I, ROI_iBL, ROI_iBR, Scalar(0, 0, 255), 2, 8, 0);
				line(I, ROI_iBR, ROI_iTR, Scalar(0, 0, 255), 2, 8, 0);
				line(I, ROI_iTR, ROI_iTL, Scalar(0, 0, 255), 2, 8, 0);
				line(I, ROI_iTL, ROI_iBL, Scalar(0, 0, 255), 2, 8, 0);
				circle(I, ROI_iBL, 2, Scalar(0, 0, 255), 2, 8, 0);
				circle(I, ROI_iBR, 2, Scalar(0, 0, 255), 2, 8, 0);
				circle(I, ROI_iTR, 2, Scalar(0, 0, 255), 2, 8, 0);
				circle(I, ROI_iTL, 2, Scalar(0, 0, 255), 2, 8, 0);
//				imshow("I", I);
				waitKey();
				destroyWindow("curSeg");
				destroyWindow("outCurSeg");		
				imshow("I", I);
				waitKey();

#endif

				if (isInROI && isRefined)
				{
					MVO_SEG.push_back(curSeg);
					MVO_ROI.push_back(curROI);

#if STORE_MVO				
					// Store images for learning	
					uint32_t static imgIdx = 0;
					char ImgNameBin[100];
					sprintf(ImgNameBin, "../../../Dataset/MVOTrainData/TrainImgs/%0.6d.jpg", imgIdx);
					imwrite(ImgNameBin, curSeg);
					imgIdx++;
#endif
				}
			}
			else {
#if	DEBUG_RAW_SEG_SAL_N_SIZE
				std::cout << "raw seg's sal, size : " << MVOSH_nonZPixl.at(segIdx) << ", " << curSeg.cols << ", " << curSeg.rows << endl;
				Mat CurROI = curROI;
				Rect rect(MAX(CurROI.at<int>(0) - 2, 0),
					MAX(CurROI.at<int>(2) - 2, 0),
					MIN(CurROI.at<int>(1) - CurROI.at<int>(0) + 4, Icopy.cols - 1),
					MIN(CurROI.at<int>(3) - CurROI.at<int>(2) + 4, Icopy.rows - 1)
				);
				rectangle(Icopy, rect, Scalar(0, 255, 0), 1);

				imshow("failed raw seg", Icopy);
				waitKey();
#endif
			}
		}
	}
#if DEBUG_NONZ_SEG_TWICE
		else {
			cout << "NONZ SEG TWICE = 0" << endl;
		}
#endif
#if	DEBUG_RAW_SEG_SAL_N_SIZE
	imshow("failed raw seg", Icopy);
	waitKey();
#endif
}

//need to study -sola
inline Void CarSnukt::BGMKnowledgedBasedUpdate(Mat &I, Mat &Bs, bool &isBAvai, Mat &B, Mat &MVOGSH)
{
	for (int i = 0; i < I.rows; i++)
	{
		for (int j = 0; j < I.cols; j++)
		{
			bool isInMVOGSH = MVOGSH.at<bool>(i, j);
			if (isInMVOGSH)
			{
				// in the Ghost related mask, use the pixel in the current image to update
				B.at<Vec3f>(i, j) = I.at<Vec3f>(i, j);
			}
			else
			{
				// in the MVO related mask, use the pixel of the previous background model
				B.at<Vec3f>(i, j) = Bs.at<Vec3f>(i, j);
			}
		}
	}
	isBAvai = true;
}

//need to study -sola
inline Bool CarSnukt::RefineDetecedMVO(Mat &MVO_SEG, Mat &MVO_ROI) // Has bugs !
{
	Mat tmpROI = MVO_ROI;
	Mat tmpSEG = MVO_SEG;
	if (countNonZero(MVO_SEG) > SAL &&
		MVO_SEG.cols > SIZE &&
		MVO_SEG.rows > SIZE)
	{
		//printf("Dens = %d Cols = %d Rows = %d = \n", countNonZero(MVO_SEG), MVO_SEG.cols, MVO_SEG.rows);
		Mat tmpTransposeSeg = tmpSEG.t();
		Mat NonZeroLocs, NonZeroLocsT;
		findNonZero(tmpSEG, NonZeroLocs);
		findNonZero(tmpTransposeSeg, NonZeroLocsT);

		assert(!NonZeroLocs.empty() && "Cannot find nonzero points");

		uint32_t minX = NonZeroLocsT.at<Point2i>(0).y; // here
		uint32_t maxX = NonZeroLocsT.at<Point2i>(NonZeroLocsT.total() - 1).y;
		uint32_t minY = NonZeroLocs.at<Point2i>(0).y;
		uint32_t maxY = NonZeroLocs.at<Point2i>(NonZeroLocs.total() - 1).y;
		tmpSEG = tmpSEG(Range(minY, maxY + 1), Range(minX, maxX + 1));

		minX = tmpROI.at<int>(0) + minX;
		maxX = tmpROI.at<int>(0) + maxX;
		minY = tmpROI.at<int>(2) + minY;
		maxY = tmpROI.at<int>(2) + maxY;
		tmpROI = (Mat_<int>(1, 4) << minX, maxX, minY, maxY);

		NonZeroLocsT = NonZeroLocsT;
		MVO_ROI = tmpROI;
		MVO_SEG = tmpSEG;

		assert(countNonZero(MVO_SEG.col(0)) > 0);
		assert(countNonZero(MVO_SEG.row(0)) > 0);
		assert(countNonZero(MVO_SEG.col(MVO_SEG.cols - 1)) > 0);
		assert(countNonZero(MVO_SEG.row(MVO_SEG.rows - 1)) > 0);

		return true;
	}
	else
	{
		return false;
	}
}

//need to study -sola
inline Void CarSnukt::DetectCriticalPoint(Mat &MVO_SEG, Mat &MVO_ROI, vector<Point2i> &CriticalPnts, Point2f &BaseLineVec)
{
	/*
	Determine the critical points of the given detected object
	*/
	Mat locs;
	vector<Point2i> Seg[8];
	// Get the centre point O
	Point2i O(MVO_SEG.cols >> 1, MVO_SEG.rows >> 1);
	// Get A and B	
	findNonZero(MVO_SEG.row(0), locs);
	Point2i A(locs.at<Point2i>(0).x, 0);
	Point2i B(locs.at<Point2i>(locs.total()-1).x, 0);
	// Get C and D
	findNonZero(MVO_SEG.col(MVO_SEG.cols - 1), locs);
	Point2i C(MVO_SEG.cols - 1, locs.at<Point2i>(0).y);
	Point2i D(MVO_SEG.cols - 1, locs.at<Point2i>(locs.total() - 1).y);
	// Get E and F
	findNonZero(MVO_SEG.row(MVO_SEG.rows - 1), locs);
	Point2i E(locs.at<Point2i>(locs.total() - 1).x, MVO_SEG.rows - 1);
	Point2i F(locs.at<Point2i>(0).x, MVO_SEG.rows - 1);
	// Get H and G
	findNonZero(MVO_SEG.col(0), locs);
	Point2i G(0, locs.at<Point2i>(locs.total() - 1).y);
	Point2i H(0, locs.at<Point2i>(0).y);

	// Calculate the Euler distances between each pair of interesting points
	uint32_t static Dis[8];
	Dis[0] = (A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y); Seg[0].push_back(A);	Seg[0].push_back(B);	// d_AB
	Dis[1] = (B.x - C.x)*(B.x - C.x) + (B.y - C.y)*(B.y - C.y); Seg[1].push_back(B);	Seg[1].push_back(C);	// d_BC
	Dis[2] = (C.x - D.x)*(C.x - D.x) + (C.y - D.y)*(C.y - D.y); Seg[2].push_back(C);	Seg[2].push_back(D);	// d_CD
	Dis[3] = (D.x - E.x)*(D.x - E.x) + (D.y - E.y)*(D.y - E.y); Seg[3].push_back(D);	Seg[3].push_back(E);	// d_DE
	Dis[4] = (E.x - F.x)*(E.x - F.x) + (E.y - F.y)*(E.y - F.y); Seg[4].push_back(E);	Seg[4].push_back(F);	// d_EF
	Dis[5] = (F.x - G.x)*(F.x - G.x) + (F.y - G.y)*(F.y - G.y); Seg[5].push_back(F);	Seg[5].push_back(G);	// d_FG	
	Dis[6] = (G.x - H.x)*(G.x - H.x) + (G.y - H.y)*(G.y - H.y); Seg[6].push_back(G);	Seg[6].push_back(H);	// d_GH
	Dis[7] = (H.x - A.x)*(H.x - A.x) + (H.y - A.y)*(H.y - A.y); Seg[7].push_back(H);	Seg[7].push_back(A);	// d_HA

	// Find the two longest lines, which may represent the orientation of the vehicle
	uint8_t maxIdx = distance(Dis, max_element(Dis, Dis + 8)); Dis[maxIdx] = 0;
	uint8_t _2ndmaxIdx = distance(Dis, max_element(Dis, Dis + 8));

	// Form the line equation of the two longest lines
	Point2f p0, p1;
	float a[3], b[3], c[3];
	//the first line	
	p0 = Seg[maxIdx].at(0);
	p1 = Seg[maxIdx].at(1);
	a[1] = p0.y - p1.y;	// yo-y1
	b[1] = p1.x - p0.x;	// x1-x0
	c[1] = (p1.y - p0.y)*p0.x + (p0.x - p1.x)*p0.y; //(y1-y0)x0 + (x0-x1)y0
	//the second line
	p0 = Seg[_2ndmaxIdx].at(0);
	p1 = Seg[_2ndmaxIdx].at(1);
	a[2] = p0.y - p1.y;	// yo-y1
	b[2] = p1.x - p0.x;	// x1-x0
	c[2] = (p1.y - p0.y)*p0.x + (p0.x - p1.x)*p0.y; //(y1-y0)x0 + (x0-x1)y0

	// Find the "phan giac" line of the two longest lines
	Point2f I[2];
	I[0].x = (float)(0.8*O.x);
	I[0].y = FindPhanGiacY(a, b, c, I[0].x);
	I[1].x = (float)(1.2*O.x);
	I[1].y = FindPhanGiacY(a, b, c, I[1].x);

	// Form the direction vector of the vehicle 
	BaseLineVec = Point2f(I[1].x - I[0].x, I[1].y - I[0].y);
	float ArgDirVec = sqrt(BaseLineVec.x*BaseLineVec.x + BaseLineVec.y*BaseLineVec.y);
	BaseLineVec.x = BaseLineVec.x / ArgDirVec;
	BaseLineVec.y = BaseLineVec.y / ArgDirVec;

	// Base on the angle between the dirVec and the positive direction of the Ox, determine the critical points
	//vector<Point2i> CriticalPnts;
	Point2i Origin(MVO_ROI.at<int>(0), MVO_ROI.at<int>(2));
	//Point2i Origin(0, 0);
	CriticalPnts.clear();
	float AbsCosAlpha = abs(BaseLineVec.x);
	if ((AbsCosAlpha <= 0.173) ||			// relatively vertical direction (>80 degrees)
		(AbsCosAlpha >= 0.985))				// relatively horizontal direction (<10 degrees)
	{
		CriticalPnts.push_back(A + Origin); CriticalPnts.push_back(B + Origin);
		CriticalPnts.push_back(C + Origin); CriticalPnts.push_back(D + Origin);
		CriticalPnts.push_back(E + Origin); CriticalPnts.push_back(F + Origin);
		CriticalPnts.push_back(G + Origin); CriticalPnts.push_back(H + Origin);
	}
	else
	{
		CriticalPnts.push_back(C + Origin); CriticalPnts.push_back(D + Origin);
		CriticalPnts.push_back(E + Origin); CriticalPnts.push_back(F + Origin);
		CriticalPnts.push_back(G + Origin); CriticalPnts.push_back(H + Origin);
	}


	/*
		Debug part
		*/
#if DEBUG_CRITICAL_POINT
	Mat clMVO_SEG;
	cvtColor(MVO_SEG, clMVO_SEG, COLOR_GRAY2BGR);

	// Draw the lines formed by the interesting points
	line(clMVO_SEG, A, B, Scalar(0, 255, 0), 2, 8);
	line(clMVO_SEG, B, C, Scalar(0, 255, 0), 2, 8);
	line(clMVO_SEG, C, D, Scalar(0, 255, 0), 2, 8);
	line(clMVO_SEG, D, E, Scalar(0, 255, 0), 2, 8);
	line(clMVO_SEG, E, F, Scalar(0, 255, 0), 2, 8);
	line(clMVO_SEG, F, G, Scalar(0, 255, 0), 2, 8);
	line(clMVO_SEG, G, H, Scalar(0, 255, 0), 2, 8);
	line(clMVO_SEG, H, A, Scalar(0, 255, 0), 2, 8);

	// Draw the base lines of the vehicle
	line(clMVO_SEG, I[0], I[1], Scalar(0, 0, 255), 2, 8);

	// Draw the critical points
	for (uint8_t i = 0; i < CriticalPnts.size(); i++)
	{
		circle(clMVO_SEG, Point2i(CriticalPnts.at(i) - Origin), 2, Scalar(0, 0, 255), 3, 4, 0);
	}

	namedWindow("Test", 0);
	imshow("Test", clMVO_SEG);
	waitKey(0);
	destroyWindow("Test");
#endif


	//Mat Test[6];
	//Test[0] = imread("../../Bin/1I.jpg");		cvtColor(Test[0], Test[0], CV_BGR2GRAY);
	//Test[1] = imread("../../Bin/1nI.jpg");	cvtColor(Test[1], Test[1], CV_BGR2GRAY);
	//Test[2] = imread("../../Bin/2I.jpg");		cvtColor(Test[2], Test[2], CV_BGR2GRAY);
	//Test[3] = imread("../../Bin/2nI.jpg");	cvtColor(Test[3], Test[3], CV_BGR2GRAY);
	//Test[4] = imread("../../Bin/3I.jpg");		cvtColor(Test[4], Test[4], CV_BGR2GRAY);
	//Test[5] = imread("../../Bin/3nI.jpg");	cvtColor(Test[5], Test[5], CV_BGR2GRAY);

	//vector<Point2i> CriticPnts[6];
	//for (int i = 0; i < 6; i++)
	//{
	//	DetectCriticalPoint(Test[i], CriticPnts[i],true);
	//}
}

//need to study -sola
inline Float CarSnukt::FindPhanGiacY(Float a[3], Float b[3], Float c[3], float x)
{
	float d[3], ab[3], y;
	d[1] = a[1] * x + c[1];
	d[2] = a[2] * x + c[2];
	ab[1] = sqrt(a[1] * a[1] + b[1] * b[1]);
	ab[2] = sqrt(a[2] * a[2] + b[2] * b[2]);
	while (1)
	{
		// case 1 and 4
		y = (d[2] * ab[1] - d[1] * ab[2]) / (ab[2] * b[1] - ab[1] * b[2]);
		if (((b[1] * y + d[1] >= 0) && (b[2] * y + d[2] >= 0)) ||
			((b[1] * y + d[1] < 0) && (b[2] * y + d[2] < 0)))
		{
			break;
		}

		// case 2 and 3
		y = -(d[2] * ab[1] + d[1] * ab[2]) / (ab[2] * b[1] + ab[1] * b[2]);
		if (((b[1] * y + d[1] >= 0) && (b[2] * y + d[2] < 0)) ||
			((b[1] * y + d[1] < 0) && (b[2] * y + d[2] >= 0)))
		{
			break;
		}
	}
	return y;
}

#if RANTREE || ANN
#if RANTREE
inline Ptr<RTrees> CarSnukt::MVOClassifier()
#elif ANN
inline Ptr<ANN_MLP> CarSnukt::MVOClassifier()
#elif 
#endif
{
#if TRAIN_MODEL		
	Ptr<TrainData> trainData = TrainData::loadFromCSV("../../../Dataset/MVOTrainData/TrainData.csv", 0, -1, -1);
	assert(!trainData.empty());
#if RANTREE
	// Create a random forrest model
	printf("Train a random forest model.\n");
	Ptr<RTrees> rtrees = RTrees::create();
	rtrees->setMaxDepth(10);
	rtrees->setMinSampleCount(2);
	rtrees->setRegressionAccuracy(0);
	rtrees->setUseSurrogates(false);
	rtrees->setMaxCategories(16);
	rtrees->setPriors(Mat());
	rtrees->setCalculateVarImportance(true);
	rtrees->setActiveVarCount(0);
	rtrees->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 200, 0));
	rtrees->train(trainData);
	rtrees->save("../../Bin/RandomTrees.xml");
#elif ANN
	printf("Train a Neural Network model.\n");
	Ptr<ANN_MLP> ann = ANN_MLP::create();		
	Mat TrainDataMat = trainData->getSamples();
	Mat TrainLabelMat = trainData->getResponses();
	Mat layer_sizes = (Mat_<double>(4, 1) << TrainDataMat.cols, 20, 20, TrainLabelMat.cols);
	ann->setLayerSizes(layer_sizes);
	ann->setActivationFunction(ANN_MLP::SIGMOID_SYM, 1, 1);
	ann->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 2000, FLT_EPSILON));
	ann->setTrainMethod(ANN_MLP::BACKPROP, 0.0001);
	ann->train(TrainDataMat, SampleTypes::ROW_SAMPLE, TrainLabelMat);
	ann->save("../../Bin/NeuralNet.xml");
#endif
	printf("Model is saved.\n");		
#else
	printf("Load model from file.\n");

#if RANTREE		
	Ptr<RTrees> rtrees = Algorithm::load<RTrees>("../../Bin/RandomTrees.xml");
	assert(!rtrees->empty());
#elif ANN
	Ptr<ANN_MLP> ann = Algorithm::load<ANN_MLP>("../../Bin/NeuralNet.xml");
	assert(!ann->empty());
#endif

#endif		

#if RANTREE || ANN
	// Test the trained model
	Ptr<TrainData> testData = TrainData::loadFromCSV("../../../Dataset/MVOTrainData/TestData.csv", 0, -1, -1);
	assert(!testData.empty());
	Mat ref_labels = testData->getResponses();
	Mat predict_labels;
	double ClassError;
#endif

#if RANTREE
	rtrees->predict(testData->getSamples(), predict_labels);
	ClassError = (double)countNonZero(ref_labels - predict_labels) / (double)(ref_labels.rows);
	printf("ClassError Random Forest = %0.5f\n", ClassError);
	return rtrees;
#elif ANN
	ann->predict(testData->getSamples(), predict_labels);
	threshold(predict_labels, predict_labels, 0.5, 1, THRESH_BINARY);
	ClassError = (double)countNonZero(ref_labels - predict_labels) / (double)(ref_labels.rows);
	printf("ClassError Neural Net = %0.5f\n", ClassError);
	return ann;
#endif	
}
#endif


inline Bool CarSnukt::CheckInsideROI(Mat &MVO_ROI)
{
	// the isInsideROI should be verified more generally when the ROI is 
	// determined by the representation of 4 points
	Point2f BL((float)MVO_ROI.at<int>(0), (float)MVO_ROI.at<int>(2));	// Object's Bottom-Left point
	Point2f BR((float)MVO_ROI.at<int>(1), (float)MVO_ROI.at<int>(2));	// Object's Bottom-Right point
	Point2f TR((float)MVO_ROI.at<int>(1), (float)MVO_ROI.at<int>(3));	// Object's Top-Right point
	Point2f TL((float)MVO_ROI.at<int>(0), (float)MVO_ROI.at<int>(3));	// Object's Top-Left point

	bool isBLSatisfied = false;
	bool isBRSatisfied = false;
	bool isTRSatisfied = false;
	bool isTLSatisfied = false;

	float a[2], b[2], c[2], x, y;

	// for the BL point of the detected object
	a[0] = ROI_iTL_BL_Param[0];
	b[0] = ROI_iTL_BL_Param[1];
	c[0] = ROI_iTL_BL_Param[2];
	a[1] = ROI_iBL_BR_Param[0];
	b[1] = ROI_iBL_BR_Param[1];
	c[1] = ROI_iBL_BR_Param[2];

	x = BL.x;
	y = BL.y;

	if (((a[0] * x + b[0] * y + c[0]) < 0) &&	// TL_BL
		((a[1] * x + b[1] * y + c[1]) > 0))		// BL_BR
		isBLSatisfied = true;

	// for the BR point of the detected object			
	a[0] = ROI_iBL_BR_Param[0];
	b[0] = ROI_iBL_BR_Param[1];
	c[0] = ROI_iBL_BR_Param[2];
	a[1] = ROI_iBR_TR_Param[0];
	b[1] = ROI_iBR_TR_Param[1];
	c[1] = ROI_iBR_TR_Param[2];
	x = BR.x;
	y = BR.y;
	if (((a[0] * x + b[0] * y + c[0]) > 0) &&	// BL_BR
		((a[1] * x + b[1] * y + c[1]) > 0))		// BR_TR
		isBRSatisfied = true;

	// for the TR point of the detected object			
	a[0] = ROI_iBR_TR_Param[0];
	b[0] = ROI_iBR_TR_Param[1];
	c[0] = ROI_iBR_TR_Param[2];
	a[1] = ROI_iTR_TL_Param[0];
	b[1] = ROI_iTR_TL_Param[1];
	c[1] = ROI_iTR_TL_Param[2];
	x = TR.x;
	y = TR.y;
	if (((a[0] * x + b[0] * y + c[0]) > 0) &&	// BR_TR
		((a[1] * x + b[1] * y + c[1]) < 0))		// TR_TL
		isTRSatisfied = true;

	// for the TR point of the detected object			
	a[0] = ROI_iTR_TL_Param[0];
	b[0] = ROI_iTR_TL_Param[1];
	c[0] = ROI_iTR_TL_Param[2];
	a[1] = ROI_iTL_BL_Param[0];
	b[1] = ROI_iTL_BL_Param[1];
	c[1] = ROI_iTL_BL_Param[2];
	x = TL.x;
	y = TL.y;
	if (((a[0] * x + b[0] * y + c[0]) < 0) &&	// TR_TL
		((a[1] * x + b[1] * y + c[1]) < 0))		// TL_BL
		isTLSatisfied = true;

	bool isInsideROI = isBLSatisfied &
		isBRSatisfied &
		isTRSatisfied &
		isTLSatisfied;

	return isInsideROI;
}

inline Void CarSnukt::LargeMVODetection(Mat &I, vector<Mat> &MVO_SEG, vector<Mat> &MVO_ROI, vector<bool> &isLargeObject)
{
	int numLarge = 0;
	for (uint32_t i = 0; i < MVO_SEG.size(); i++)
	{
		Mat tmpMVO_SEG = MVO_SEG.at(i);
		bool isLrgObjt = false;
#if RANTREE
		Mat Label;
		Mat FetVec = FetureExtract(tmpMVO_SEG);
		RTMVOClassifier->predict(FetVec, Label);
		isLrgObjt = (bool)Label.at<float>(0);
#elif ANN
		Mat Label;
		Mat FetVec = FetureExtract(tmpMVO_SEG);
		MLPMVOClassifier->predict(FetVec, Label);
		threshold(Label, Label, 0.5, 1, THRESH_BINARY);
		isLrgObjt = (bool)Label.at<float>(0);
#elif SIZE_SALIENCY				
		if (countNonZero(tmpMVO_SEG) > LARGE_OBJ_DENS &&
			tmpMVO_SEG.cols > LARGE_OBJECT_SIZE &&
			tmpMVO_SEG.rows > LARGE_OBJECT_SIZE) 
		{
			isLrgObjt = true;
			numLarge++;
		}
#endif				
		isLargeObject.push_back(isLrgObjt);

#if DEBUG_MVO_CLASSSIFY
		cout << "isLargeObject " << isLrgObjt << "\n";
		for (size_t j = 0; j < MVO_ROI.size(); j++)
		{
			// Draw the bounding boxes
			Mat CurROI = MVO_ROI.at(j);
			Rect rect(MAX(CurROI.at<int>(0) - 2, 0),
				MAX(CurROI.at<int>(2) - 2, 0),
				MIN(CurROI.at<int>(1) - CurROI.at<int>(0) + 4, I.cols - 1),
				MIN(CurROI.at<int>(3) - CurROI.at<int>(2) + 4, I.rows - 1)
				);
			if (isLrgObjt)
				rectangle(I, rect, Scalar(0, 0, 255), 1);
			else
				rectangle(I, rect, Scalar(0, 255, 0), 1);
		}
		line(I, ROI_iBL, ROI_iBR, Scalar(0, 0, 255), 2, 8, 0);
		line(I, ROI_iBR, ROI_iTR, Scalar(0, 0, 255), 2, 8, 0);
		line(I, ROI_iTR, ROI_iTL, Scalar(0, 0, 255), 2, 8, 0);
		line(I, ROI_iTL, ROI_iBL, Scalar(0, 0, 255), 2, 8, 0);
		circle(I, ROI_iBL, 2, Scalar(0, 0, 255), 2, 8, 0);
		circle(I, ROI_iBR, 2, Scalar(0, 0, 255), 2, 8, 0);
		circle(I, ROI_iTR, 2, Scalar(0, 0, 255), 2, 8, 0);
		circle(I, ROI_iTL, 2, Scalar(0, 0, 255), 2, 8, 0);
		imshow("I", I);
		waitKey();
#endif		
	}

	//cout << "LargeMVODetection # : " << numLarge << endl;
}

inline Void CarSnukt::UpdateHisPos(Point2i HisPos[HIS_POS_SIZE], Point2i NewPoint)
{
	for (int i = 0; i < HIS_POS_SIZE - 1; i++)
	{
		HisPos[i] = HisPos[i + 1];
	}
	HisPos[HIS_POS_SIZE - 1] = NewPoint;
}

inline Void CarSnukt::MVOCalColorHistogram(Mat &MVO, Mat &b_hist, Mat &g_hist, Mat &r_hist)
{
	/// Separate the image in 3 places ( B, G and R )
	Mat bgr_planes[3];
	split(MVO, bgr_planes);
	/// Establish the number of bins
	int histSize = HISTOGRAM_BIN_SIZE;
	/// Set the ranges ( for B,G,R) )
	float range[] = { 0, 1 };
	const float* histRange = { range };
	bool uniform = true; bool accumulate = false;
	/// Compute the histograms:
	calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);
	calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
	calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);
	/// Normalize the result to [ 0, 1 ]
	normalize(b_hist, b_hist, 0, 1, NORM_MINMAX, -1, Mat());
	normalize(g_hist, g_hist, 0, 1, NORM_MINMAX, -1, Mat());
	normalize(r_hist, r_hist, 0, 1, NORM_MINMAX, -1, Mat());
}

inline Void CarSnukt::TransformCriticalPoints(vector<Point2i> &CriticPntsVec, Mat &TransROI)
{
	Mat tmpTransROI = (Mat_<uint32_t>(1, 4) << INT_MAX, 0, INT_MAX, 0);
	for (size_t i = 0; i < CriticPntsVec.size(); i++)
	{
		Point2f curPoint = CriticPntsVec.at(i);
		Mat A(1, 1, CV_32FC2, Scalar(curPoint.x, curPoint.y));
		Mat	B;
		perspectiveTransform(A, B, TransMat);
		//B.convertTo(B, CV_32SC2);
		Point2i TransP = (Point2i)B.at<Point2f>(0);

		// find minX
		if (tmpTransROI.at<int>(0) > TransP.x)
			tmpTransROI.at<int>(0) = TransP.x;
		// find maxX
		if (tmpTransROI.at<int>(1) < TransP.x)
			tmpTransROI.at<int>(1) = TransP.x;
		// find minY
		if (tmpTransROI.at<int>(2) > TransP.y)
			tmpTransROI.at<int>(2) = TransP.y;
		// find maxY
		if (tmpTransROI.at<int>(3) < TransP.y)
			tmpTransROI.at<int>(3) = TransP.y;

		//cout << "A = " << A << "\n";
		//cout << "B = " << B << "\n";
	}
	TransROI = tmpTransROI;
	//tmpTransROI.copyTo(TransROI);
	//cout << "tmpTransROI = " << tmpTransROI << "\n";
	//cout << "TransROI = " << TransROI << "\n";
}

inline Void CarSnukt::CreateNewTrackObjt(Mat &I, Mat &curSeg, Mat &curROI)
{
	// Update the live object list
	LiveObjList.at<uint8_t>(0, NewTrackObj) = 1;

	// Update the history positions
	Point2i CurPos = Point2i((curROI.at<int>(0) + curROI.at<int>(1)) >> 1,
		(curROI.at<int>(2) + curROI.at<int>(3)) >> 1);
	UpdateHisPos(TrackObj[NewTrackObj].HisPos, CurPos);
	TrackObj[NewTrackObj].NumOfHisPt = 1;

	// Update the current ROI
	TrackObj[NewTrackObj].ROI = curROI;

	// Update the 2D image of the detected object
	Mat SEG = I(Range(curROI.at<int>(2), curROI.at<int>(3) + 1),
		Range(curROI.at<int>(0), curROI.at<int>(1) + 1));

	// Update the soft ID of the current object
	TrackObj[NewTrackObj].SoftID = NewTrackObj;

	// Update the hard_ID
	TrackObj[NewTrackObj].HardID = UNKNOW_HARD_ID;

	// Update the virtual_ID for ITRC projcet
	if (vID > MOV_VIRTUAL_ID_END) {
		vID = MVO_VIRTUAL_ID_START;
	}
	TrackObj[NewTrackObj].vID = vID++;
 
	// Calculate the color histograms of the current object					
	MVOCalColorHistogram(SEG,
		TrackObj[NewTrackObj].b_hist,
		TrackObj[NewTrackObj].g_hist,
		TrackObj[NewTrackObj].r_hist);

	// update the center in the image plane
	TrackObj[NewTrackObj].CenterImgPlane = TrackObj[NewTrackObj].HisPos[HIS_POS_SIZE - 1];

	// to calculate the gps Velocity
	TrackObj[NewTrackObj].gpsVel.resetIsFirstMoment();


#if IS_USE_PER_TRANS && TRANSFORM_CENTER_POINT
	// The center for the transformed p
	TrackObj[NewTrackObj].CenterTrans = TransformPoint(TrackObj[NewTrackObj].CenterImgPlane);
#endif
	// Transforms those detected critical points
#if IS_USE_PER_TRANS && TRANSFORM_CRITICAL_POINT
	// The center for the transformed p
	TrackObj[NewTrackObj].CenterTrans = TransformPoint(TrackObj[NewTrackObj].CenterImgPlane);
	// Find the critical points
	DetectCriticalPoint(curSeg, curROI, TrackObj[NewTrackObj].CriticPntsVec, TrackObj[NewTrackObj].BaseLineVec);
	TransformCriticalPoints(TrackObj[NewTrackObj].CriticPntsVec, TrackObj[NewTrackObj].TransROI);
#endif

#if KALMAN_TRACK
	// Initialize the Kalman filter parameters for each track object
	TrackObj[NewTrackObj].KF.init(4, 2, 0);
	TrackObj[NewTrackObj].KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);

	TrackObj[NewTrackObj].PosMeasure = (Mat_<float>(2, 1) << CurPos.x, CurPos.y);
	TrackObj[NewTrackObj].PosMeasure.setTo(Scalar(0));

	TrackObj[NewTrackObj].KF.statePre.at<float>(0) = (float)CurPos.x;
	TrackObj[NewTrackObj].KF.statePre.at<float>(1) = (float)CurPos.y;
	TrackObj[NewTrackObj].KF.statePre.at<float>(2) = 0;
	TrackObj[NewTrackObj].KF.statePre.at<float>(3) = 0;

	setIdentity(TrackObj[NewTrackObj].KF.measurementMatrix);
	setIdentity(TrackObj[NewTrackObj].KF.processNoiseCov, Scalar::all(1e-3));
	setIdentity(TrackObj[NewTrackObj].KF.measurementNoiseCov, Scalar::all(1e-2));
	//setIdentity(TrackObj[NewTrackObj].KF.errorCovPost, Scalar::all(0.2));
#endif

	// Update the ID for the new detected objects	
	if (countNonZero(LiveObjList) == 0)
	{
		NewTrackObj++;
	}
	else
	{
		Mat NonZ;
		findNonZero(LiveObjList, NonZ);
		bool isConflict = true;
		int NewIDFindIter = 0;
		while (isConflict)
		{
			// assume that the new assign ID is a non-conflict ID
			isConflict = false;
			NewTrackObj++;
			NewTrackObj = NewTrackObj % LIVE_OBJECT_SIZE;
			// test its satisfactory
			for (size_t i = 0; i < NonZ.total(); i++)
			{
				if (NewTrackObj == NonZ.at<Point2i>(i).x)
				{
					isConflict = true;
				}
			}
			// assert if there are too many objects
			NewIDFindIter++;
			if (NewIDFindIter >= LIVE_OBJECT_SIZE)
			{
				std::printf("There are too many objects\n");
				NewTrackObj = 0;
			}
			//assert(NewIDFindIter <= LIVE_OBJECT_SIZE && "Cant not find a new ID");
		}
	}

}

inline Void CarSnukt::SmallROIRefine(vector<bool> &isLargeObject, vector<Mat> &MVO_ROI, vector<Mat> &SmallObjectROI)
{
	SmallObjectROI.clear();
	for (size_t i = 0; i < MVO_ROI.size(); i++)
	{
		if (isLargeObject.at(i) == 0)
			SmallObjectROI.push_back(MVO_ROI.at(i));
	}
}

inline Void CarSnukt::LargeMVOTracking(Mat &I,
	vector<Mat> &MVO_SEG,
	vector<Mat> &MVO_ROI,
	vector<bool> &isLargeObject,
	vector<int> &hardIdCode)
{
	// create new track objects if currently there is no live trackObject				
	if (countNonZero(LiveObjList) == 0)
	{
		for (size_t i = 0; i < MVO_SEG.size(); i++)
		{
			if (isLargeObject.at(i))
			{
				//printf("TrackObjectID = %d is created\n", NewTrackObj);
				CreateNewTrackObjt(I, MVO_SEG.at(i), MVO_ROI.at(i));
			}
		}
	}
	else // track the current existing live objects		
	{
		// temporal variables should be placed here
		Mat b_hist_obj[LIVE_OBJECT_SIZE];
		Mat g_hist_obj[LIVE_OBJECT_SIZE];
		Mat r_hist_obj[LIVE_OBJECT_SIZE];
		Point2i CurDetPos[LIVE_OBJECT_SIZE];
		Point2i CurDetSize[LIVE_OBJECT_SIZE];
		Mat MatchedObjList = Mat::zeros(1, LIVE_OBJECT_SIZE, CV_8UC1);
		Mat DetectedObjList = Mat::zeros(1, LIVE_OBJECT_SIZE, CV_8UC1);

		// Calculate the histograms and current position values for all detected objects
		for (size_t j = 0; j < MVO_SEG.size(); j++)
		{
#if	EXCLUDE_SMALL_MVOS_IN_TRK
			if (!isLargeObject.at(j)) {
				continue;
			}
#endif
			Mat curROI = MVO_ROI.at(j);
			Mat SEG = I(Range(curROI.at<int>(2), curROI.at<int>(3) + 1),
				Range(curROI.at<int>(0), curROI.at<int>(1) + 1));
			MVOCalColorHistogram(SEG, b_hist_obj[j], g_hist_obj[j], r_hist_obj[j]);
			CurDetPos[j] = Point2i((curROI.at<int>(0) + curROI.at<int>(1)) >> 1,
				(curROI.at<int>(2) + curROI.at<int>(3)) >> 1);
			CurDetSize[j] = Point2i(curROI.at<int>(1) - curROI.at<int>(0),
				curROI.at<int>(3) - curROI.at<int>(2));
			if (isLargeObject.at(j))
			{
				DetectedObjList.at<uint8_t>(0, j) = 1;
			}
		}

		// For each live track object, try to find its' most likely brother :D
		Mat NonZIdx;
		findNonZero(LiveObjList, NonZIdx);
		for (size_t i = 0; i < NonZIdx.total(); i++)
		{
			int TrackObjIdx = NonZIdx.at<Point2i>(i).x;
			bool isMatched = false;
			double MinDiffSum = (float)LONG_MAX;
			double Min_Diff_Hist = (float)LONG_MAX;
			double Min_Diff_Pos = (float)LONG_MAX;
			double Min_Diff_Size = (float)LONG_MAX;
			int TmpMatchedID = INT_MAX;

			//find its' most likely brother in all detected objects (including small obj)
			for (size_t j = 0; j < MVO_SEG.size(); j++)
			{
#if	EXCLUDE_SMALL_MVOS_IN_TRK
				if (!isLargeObject.at(j)) {
					continue;
				}
#endif
				if (MatchedObjList.at<uint8_t>(0, j) == 0)
				{
					Mat dif_b_hist = TrackObj[TrackObjIdx].b_hist - b_hist_obj[j];
					Mat dif_g_hist = TrackObj[TrackObjIdx].g_hist - g_hist_obj[j];
					Mat dif_r_hist = TrackObj[TrackObjIdx].r_hist - r_hist_obj[j];
					dif_b_hist = dif_b_hist.mul(dif_b_hist);
					dif_g_hist = dif_g_hist.mul(dif_g_hist);
					dif_r_hist = dif_r_hist.mul(dif_r_hist);
					Point2i Diff_Point = (TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 1] - CurDetPos[j]);
					Point2i CurSize = Point2i(TrackObj[TrackObjIdx].ROI.at<int>(1) - TrackObj[TrackObjIdx].ROI.at<int>(0),
						TrackObj[TrackObjIdx].ROI.at<int>(3) - TrackObj[TrackObjIdx].ROI.at<int>(2));

					double Diff_Hist = sum(dif_b_hist)[0] + sum(dif_g_hist)[0] + sum(dif_r_hist)[0];
					double Diff_Pos = Diff_Point.x*Diff_Point.x + Diff_Point.y*Diff_Point.y;
					double Diff_Size = (CurSize.x - CurDetSize[j].x)*(CurSize.x - CurDetSize[j].x) +
						(CurSize.y - CurDetSize[j].y)*(CurSize.y - CurDetSize[j].y);
					double DiffSum = Diff_Hist + Diff_Pos + Diff_Size;
					//printf("Diff_Hist = %0.5f Diff_Pos = %0.5f Diff_Size = %0.5f\n", Diff_Hist, Diff_Pos, Diff_Size);	

					if (DiffSum < MinDiffSum)
					{
						MinDiffSum = DiffSum;
						Min_Diff_Hist = Diff_Hist;
						Min_Diff_Pos = Diff_Pos;
						Min_Diff_Size = Diff_Size;
						TmpMatchedID = j;
					}
				}
			}

			if (Min_Diff_Hist < HISTOGRAM_THRES && Min_Diff_Pos < DISTANCE_THRES && Min_Diff_Size < SIZE_THRES) // tracking sucessfull
			{
				//printf("TrackObjIdx %d is EXPLICITLY matched\n", TrackObjIdx);

				// to narrow down the candidates for the next matching round	
				DetectedObjList.at<uint8_t>(0, TmpMatchedID) = 1; // important
				MatchedObjList.at<uint8_t>(0, TmpMatchedID) = 1;

				// update the large object flag if there is a miss classification
				isLargeObject.at(TmpMatchedID) = true;

				// update the newly matched object's features							
				TrackObj[TrackObjIdx].b_hist = b_hist_obj[TmpMatchedID];
				TrackObj[TrackObjIdx].g_hist = g_hist_obj[TmpMatchedID];
				TrackObj[TrackObjIdx].r_hist = r_hist_obj[TmpMatchedID];
				TrackObj[TrackObjIdx].ROI = MVO_ROI.at(TmpMatchedID);
				UpdateHisPos(TrackObj[TrackObjIdx].HisPos, CurDetPos[TmpMatchedID]);
				TrackObj[TrackObjIdx].NumOfHisPt++;
				isMatched = true;

				// update the center in the image plane 
				TrackObj[TrackObjIdx].CenterImgPlane = Point2i((TrackObj[TrackObjIdx].ROI.at<int>(0) + TrackObj[TrackObjIdx].ROI.at<int>(1)) >> 1,
					(TrackObj[TrackObjIdx].ROI.at<int>(2) + TrackObj[TrackObjIdx].ROI.at<int>(3)) >> 1);

#if TRANSFORM_CRITICAL_POINT
				// centerpoint update in the transformed plane
				TrackObj[TrackObjIdx].CenterTrans = TransformPoint(TrackObj[TrackObjIdx].CenterImgPlane);

				// update the critical points and the estimated orientation of the vehicle
				DetectCriticalPoint(MVO_SEG.at(TmpMatchedID), MVO_ROI.at(TmpMatchedID), TrackObj[TrackObjIdx].CriticPntsVec, TrackObj[TrackObjIdx].BaseLineVec);

				// update the direction of the large object		
				Point2i HisVec[3];
				HisVec[2] = TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 1] - TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 2];
				HisVec[1] = TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 2] - TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 3];
				HisVec[0] = TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 3] - TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 4];
				Point2i OverallHisVec = (HisVec[0] + HisVec[1] + HisVec[2]);
				OverallHisVec.x = OverallHisVec.x / 3;
				OverallHisVec.y = OverallHisVec.y / 3;
				int ArgHisVec = HisVec[0].x*HisVec[0].x + HisVec[0].y*HisVec[0].y +
					HisVec[1].x*HisVec[1].x + HisVec[1].y*HisVec[1].y +
					HisVec[2].x*HisVec[2].x + HisVec[2].y*HisVec[2].y;

				if ((OverallHisVec.x*TrackObj[TrackObjIdx].BaseLineVec.x + OverallHisVec.y*TrackObj[TrackObjIdx].BaseLineVec.y) < 0)
				{
					TrackObj[TrackObjIdx].Direction = Point2f(0, 0) - TrackObj[TrackObjIdx].BaseLineVec;
				}
				else
				{
					TrackObj[TrackObjIdx].Direction = TrackObj[TrackObjIdx].BaseLineVec;
				}
				TrackObj[TrackObjIdx].Direction = (Point2f)TrackObj[TrackObjIdx].CenterImgPlane +
					Point2f(VECTOR_UNIT_LENGTH*TrackObj[TrackObjIdx].Direction.x, VECTOR_UNIT_LENGTH*TrackObj[TrackObjIdx].Direction.y);

				// update the transformed ROI, center and direction
				TransformCriticalPoints(TrackObj[TrackObjIdx].CriticPntsVec, TrackObj[TrackObjIdx].TransROI);
				TrackObj[TrackObjIdx].DirectionTrans = TransformPoint(TrackObj[TrackObjIdx].Direction);

				// update the moving state of the vehicle
				Point2f DiffCenterTrans1 = TrackObj[TrackObjIdx].CenterTrans - TransformPoint(TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 2]);
				Point2f DiffCenterTrans2 = TransformPoint(TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 2]) - TransformPoint(TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 3]);
				float MovedDist = DiffCenterTrans1.x*DiffCenterTrans1.x + DiffCenterTrans1.y*DiffCenterTrans1.y +
					DiffCenterTrans2.x*DiffCenterTrans2.x + DiffCenterTrans2.y*DiffCenterTrans2.y;
				if (MovedDist < MOVING_THRESH)
				{
					TrackObj[TrackObjIdx].isMoving = false;
				}
				else
				{
					TrackObj[TrackObjIdx].isMoving = true;
				}
#endif
			}
#if DEBUG_TRACKING	
			else {
				cout << "LargeMVOTracking () : Min_Diff < THR ; HIST: " << Min_Diff_Hist << " POS: " << Min_Diff_Pos << " SIZE: " << Min_Diff_Size << endl;
			}
#endif

#if DEBUG_TRACK_DIFF
			else {
				cout << "Min_Diff_Hist , Min_Diff_Pos , Min_Diff_Size "<<Min_Diff_Hist <<" "<< Min_Diff_Pos << " " << Min_Diff_Size << endl;
			}

#endif

			// track an object using a predictive model
#if PREDICTIVE_TRACK
			if (!isMatched && TrackObj[TrackObjIdx].NumOfHisPt > 3)
			{
				// predict the new center of the lost-track object
				Point2i HisPos_1 = TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 1];
				Point2i HisPos_2 = TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 2];
				Point2i HisPos_3 = TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 3];
				Point2i PredPos = HisPos_1 + HisPos_2 - HisPos_3;
				// predict the ROI of the lost-track object
				Point2i Diff_Point = PredPos - HisPos_1;
				Mat PredROI = TrackObj[TrackObjIdx].ROI;
				PredROI.at<int>(0) += Diff_Point.x;
				PredROI.at<int>(1) += Diff_Point.x;
				PredROI.at<int>(2) += Diff_Point.y;
				PredROI.at<int>(3) += Diff_Point.y;
				// check if the predicted ROI is totally inside the global ROI or not
				Bool isInROI = CheckInsideROI(PredROI);
				// perform the matching when isInROI is true					
				if (isInROI)
				{
					Mat predSEG = I(Range(PredROI.at<int>(2), PredROI.at<int>(3) + 1),
						Range(PredROI.at<int>(0), PredROI.at<int>(1) + 1));
					Mat pred_b_hist_obj, pred_g_hist_obj, pred_r_hist_obj;
					MVOCalColorHistogram(predSEG, pred_b_hist_obj, pred_g_hist_obj, pred_r_hist_obj);
					Mat dif_b_hist = TrackObj[TrackObjIdx].b_hist - pred_b_hist_obj;
					Mat dif_g_hist = TrackObj[TrackObjIdx].g_hist - pred_g_hist_obj;
					Mat dif_r_hist = TrackObj[TrackObjIdx].r_hist - pred_r_hist_obj;
					dif_b_hist = dif_b_hist.mul(dif_b_hist);
					dif_g_hist = dif_g_hist.mul(dif_g_hist);
					dif_r_hist = dif_r_hist.mul(dif_r_hist);
					double Diff_Hist = sum(dif_b_hist)[0] + sum(dif_g_hist)[0] + sum(dif_r_hist)[0];
					double Diff_Pos = Diff_Point.x*Diff_Point.x + Diff_Point.y*Diff_Point.y;
					if (Diff_Hist < HISTOGRAM_THRES && Diff_Pos < DISTANCE_THRES)
					{
						//printf("TrackObjIdx %d PREDICTIVELY matched\n", TrackObjIdx);

						// update the newly matched object							
						TrackObj[TrackObjIdx].b_hist = pred_b_hist_obj;
						TrackObj[TrackObjIdx].g_hist = pred_g_hist_obj;
						TrackObj[TrackObjIdx].r_hist = pred_r_hist_obj;
						TrackObj[TrackObjIdx].ROI = PredROI;
						UpdateHisPos(TrackObj[TrackObjIdx].HisPos, PredPos);
						TrackObj[TrackObjIdx].NumOfHisPt++;
						TrackObj[TrackObjIdx].NumOfHisPt %= HIS_POS_SIZE;
						isMatched = true;
					}

					// update the center in the image plane and the transformed plane
					TrackObj[TrackObjIdx].CenterImgPlane = TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 1];

#if TRANSFORM_CRITICAL_POINT
					TrackObj[TrackObjIdx].CenterTrans = TransformPoint(TrackObj[TrackObjIdx].CenterImgPlane);
					TrackObj[TrackObjIdx].Direction = (Point2f)TrackObj[TrackObjIdx].CenterImgPlane - (Point2f)TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 2] +
						Point2f(VECTOR_UNIT_LENGTH*TrackObj[TrackObjIdx].Direction.x, VECTOR_UNIT_LENGTH*TrackObj[TrackObjIdx].Direction.y);

					// update the transformed ROI, center and direction
					TransformCriticalPoints(TrackObj[TrackObjIdx].CriticPntsVec, TrackObj[TrackObjIdx].TransROI);
					TrackObj[TrackObjIdx].DirectionTrans = TransformPoint(TrackObj[TrackObjIdx].Direction);

					// update the moving state of the vehicle
					Point2f DiffCenterTrans1 = TrackObj[TrackObjIdx].CenterTrans - TransformPoint(TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 2]);
					Point2f DiffCenterTrans2 = TransformPoint(TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 2]) - TransformPoint(TrackObj[TrackObjIdx].HisPos[HIS_POS_SIZE - 3]);
					float MovedDist = DiffCenterTrans1.x*DiffCenterTrans1.x + DiffCenterTrans1.y*DiffCenterTrans1.y +
						DiffCenterTrans2.x*DiffCenterTrans2.x + DiffCenterTrans2.y*DiffCenterTrans2.y;
					if (MovedDist < MOVING_THRESH)
					{
						TrackObj[TrackObjIdx].isMoving = false;
					}
					else
					{
						TrackObj[TrackObjIdx].isMoving = true;
					}
#endif
				}
			}
#endif

#if DETECT_DIRECTION
			if (isMatched)
			{
				// get the extracted image of the object and double its size
				Mat curROI = MVO_ROI.at(i);
				Mat ObjectRGB = I(Range(curROI.at<int>(2), curROI.at<int>(3) + 1),
					Range(curROI.at<int>(0), curROI.at<int>(1) + 1));
				Size doubleSize(8 * ObjectRGB.size().width, 8 * ObjectRGB.size().height);
				resize(ObjectRGB, ObjectRGB, doubleSize);
				// access the red channel
				Mat RedChannel;
				inRange(ObjectRGB, Scalar(0.05,0.05,0.4), Scalar(0.3,0.3,1), RedChannel);
				GaussianBlur(RedChannel, RedChannel, Size(5, 5), 0.8);
				inRange(RedChannel, 150, 255, RedChannel);
				vector<Mat> Marker_ROI;
				vector<Mat> Marker_SEG; 
				vector<int> Marker_nonZPixl;
				NonZeroSegTwice(RedChannel, Marker_ROI, Marker_SEG, Marker_nonZPixl);	
				assert(Marker_ROI.size() >= 2 && "It must have at least 2-red-color segments.");				
				// determine the head and tail
				int MaxIdx = 0;
				int SecondMaxIdx = 0;
				int Maxval = 0;
				int SecondMaxVal = 0;
				for (int i = 0; i < Marker_nonZPixl.size(); i++)
				{
					if (Marker_nonZPixl.at(i) > Maxval)
					{
						SecondMaxVal = Maxval;
						SecondMaxIdx = MaxIdx;
						Maxval = Marker_nonZPixl.at(i);
						MaxIdx = i;
					}
					else if (Marker_nonZPixl.at(i) > SecondMaxVal)
					{
						SecondMaxVal = Marker_nonZPixl.at(i);
						SecondMaxIdx = i;
					}
				}
				Point2i Head, Tail;
				Mat MarkerROI = Marker_ROI.at(MaxIdx);
				Head.y = (MarkerROI.at<int>(3) + MarkerROI.at<int>(2)) >> 1;
				Head.x = (MarkerROI.at<int>(1) + MarkerROI.at<int>(0)) >> 1;
				MarkerROI = Marker_ROI.at(SecondMaxIdx);
				Tail.y = (MarkerROI.at<int>(3) + MarkerROI.at<int>(2)) >> 1;       
				Tail.x = (MarkerROI.at<int>(1) + MarkerROI.at<int>(0)) >> 1;

				TrackObj[TrackObjIdx].Head = Point2i(Head.x >> 3, Head.y >> 3) + Point2i(curROI.at<int>(0), curROI.at<int>(2));
				TrackObj[TrackObjIdx].Tail = Point2i(Tail.x >> 3, Tail.y >> 3) + Point2i(curROI.at<int>(0), curROI.at<int>(2));
				// debug   
				circle(ObjectRGB, Head, 3, Scalar(0, 255, 0), 2, 4, 0);
				circle(ObjectRGB, Tail, 3, Scalar(255, 0, 0), 2, 4, 0);
				imshow("ObjectRGB", ObjectRGB);								
				imshow("RedChannel", RedChannel);
				waitKey(0);
			}
			else// terminate the object if there is no match for it
			{
				//printf("TrackObjIdx %d is terminated\n", TrackObjIdx);
				LiveObjList.at<uint8_t>(0, TrackObjIdx) = 0;
			}
#else
			// predict the future position using Kalman filter for the next several iterations
			// this step is used for the control algorithm
			if (isMatched)
			{

			}
			else // terminate the object if there is no match for it
			{
				//printf("TrackObjIdx %d is terminated\n", TrackObjIdx);
				LiveObjList.at<uint8_t>(0, TrackObjIdx) = 0;
			}
#endif
		}

		// add a new tracking object
		DetectedObjList = DetectedObjList - MatchedObjList;
		//cout << countNonZero(DetectedObjList) << "\n";
		if (countNonZero(DetectedObjList) > 0) // bugs are here
		{
			Mat NewDetIdx;
			findNonZero(DetectedObjList, NewDetIdx);
			for (size_t j = 0; j < NewDetIdx.total(); j++)
			{
				//printf("TrackObjectID = %d is created\n", NewTrackObj);
				CreateNewTrackObjt(I, MVO_SEG.at(NewDetIdx.at<Point2i>(j).x), MVO_ROI.at(NewDetIdx.at<Point2i>(j).x));
			}
		}
	}

#if 0	
//#if DEBUG_TRACKING	
	if (countNonZero(LiveObjList) > 0)
	{
		vector<Point2i> NonZ;
		findNonZero(LiveObjList, NonZ);
		for (size_t i = 0; i < NonZ.size(); i++)
		{
			uint8_t ID = NonZ.at(i).x;
			// Draw the bounding boxes
			Mat CurROI = TrackObj[ID].ROI;
			Rect rect(MAX(CurROI.at<int>(0) - 2, 0),
				MAX(CurROI.at<int>(2) - 2, 0),
				MIN(CurROI.at<int>(1) - CurROI.at<int>(0) + 4, I.cols - 1),
				MIN(CurROI.at<int>(3) - CurROI.at<int>(2) + 4, I.rows - 1)
				);
			rectangle(I, rect, Scalar(0, 0, 255), 1);
			// Draw the object ID
			uint8_t ObjectID = TrackObj[ID].SoftID;
			char str[200];
			sprintf(str, "ID = %d", ObjectID);
			putText(I, str, Point2f(CurROI.at<int>(0), CurROI.at<int>(2)), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255, 255), 2);
		}
	}

	// Draw the ROI
	line(I, ROI_iBL, ROI_iBR, Scalar(0, 0, 255), 2, 8, 0);
	line(I, ROI_iBR, ROI_iTR, Scalar(0, 0, 255), 2, 8, 0);
	line(I, ROI_iTR, ROI_iTL, Scalar(0, 0, 255), 2, 8, 0);
	line(I, ROI_iTL, ROI_iBL, Scalar(0, 0, 255), 2, 8, 0);
	circle(I, ROI_iBL, 2, Scalar(0, 0, 255), 2, 8, 0);
	circle(I, ROI_iBR, 2, Scalar(0, 0, 255), 2, 8, 0);
	circle(I, ROI_iTR, 2, Scalar(0, 0, 255), 2, 8, 0);
	circle(I, ROI_iTL, 2, Scalar(0, 0, 255), 2, 8, 0);

	imshow("I", I);
	waitKey(1);
#endif
}

inline Point2f CarSnukt::TransformPoint(Point2f curPoint)
{
	Mat A(1, 1, CV_32FC2, Scalar(curPoint.x, curPoint.y));
	Mat	B;
	perspectiveTransform(A, B, TransMat);
	//B.convertTo(B, CV_32SC2);
	Point2f TransP = (Point2i)B.at<Point2f>(0);
	return TransP;
}

Void CarSnukt::Initialize(uint32_t ImageVerSize, uint32_t ImageHorSize)
{
	// Train or load the classifier
#if RANTREE
	RTMVOClassifier = MVOClassifier();
#elif ANN
	MLPMVOClassifier = MVOClassifier();
#endif
	// Get the image size
	ImVerSize = ImageVerSize;
	ImHorSize = ImageHorSize;
	// Initialize the masks
	Mat FG = Mat::zeros(ImVerSize, ImHorSize, CV_32FC1); // Foreground 						
	Mat MVOSH = Mat::zeros(ImVerSize, ImHorSize, CV_32FC1); // MVO related mask: MVO and MVO'shadow
	Mat MVO = Mat::zeros(ImVerSize, ImHorSize, CV_32FC1); // MVO mask
	Mat SH = Mat::zeros(ImVerSize, ImHorSize, CV_32FC1); // MVO'shadow mask
	Mat MVOGSH = Mat::zeros(ImVerSize, ImHorSize, CV_32FC1); // Ghost related mask: ghost and ghost's shadow
}

Void CarSnukt::FormROI(Point2i BL, Point2i BR, Point2i TR, Point2i TL)
{
	//		   Image matrix 
	//0--------------------------> hor
	//|
	//|  BL _______________ BR  
	//|    |			   |
	//|    |			   |
	//|    |	 A MVO     |
	//|    |		       |
	//|    |			   |
	//|    |_______________|
	//|  TL					TR
	//|
	// \/
	//   ver

	// Form the outer ROI mask
	ROI_BL = BL;
	ROI_BR = BR;
	ROI_TR = TR;
	ROI_TL = TL;

	ROI = Mat::zeros(ImVerSize, ImHorSize, CV_32FC1);
	Point poly[4];
	poly[0] = ROI_TL;
	poly[1] = ROI_TR;
	poly[2] = ROI_BR;
	poly[3] = ROI_BL;
	fillConvexPoly(ROI, poly, 4, Scalar::all(1));
	/*imshow("ROI", ROI);
	waitKey();*/

	assert(ROI_TL.x >= 0 && ROI_TL.x < ImHorSize);
	assert(ROI_TR.x >= 0 && ROI_TR.x < ImHorSize);
	assert(ROI_BL.x >= 0 && ROI_BL.x < ImHorSize);
	assert(ROI_BR.x >= 0 && ROI_BR.x < ImHorSize);

	assert(ROI_TL.y >= 0 && ROI_TL.y < ImVerSize);
	assert(ROI_TR.y >= 0 && ROI_TR.y < ImVerSize);
	assert(ROI_BL.y >= 0 && ROI_BL.y < ImVerSize);
	assert(ROI_BR.y >= 0 && ROI_BR.y < ImVerSize);

	// Form a virtual ROI which is inside the current ROI
	ROI_iBL = BL + Point2i(+ROI_VEC, +ROI_VEC);
	ROI_iBR = BR + Point2i(-ROI_VEC, +ROI_VEC);
	ROI_iTR = TR + Point2i(-ROI_VEC, -ROI_VEC);
	ROI_iTL = TL + Point2i(+ROI_VEC, -ROI_VEC);

	assert(ROI_iTL.x >= 0 && ROI_iTL.x < ImHorSize);
	assert(ROI_iTR.x >= 0 && ROI_iTR.x < ImHorSize);
	assert(ROI_iBL.x >= 0 && ROI_iBL.x < ImHorSize);
	assert(ROI_iBR.x >= 0 && ROI_iBR.x < ImHorSize);

	assert(ROI_iTL.y >= 0 && ROI_iTL.y < ImVerSize);
	assert(ROI_iTR.y >= 0 && ROI_iTR.y < ImVerSize);
	assert(ROI_iBL.y >= 0 && ROI_iBL.y < ImVerSize);
	assert(ROI_iBR.y >= 0 && ROI_iBR.y < ImVerSize);

	Point2f p0, p1;
	p0 = (Point2f)ROI_iBL;
	p1 = (Point2f)ROI_iBR;
	ROI_iBL_BR_Param[0] = p0.y - p1.y;	// yo-y1
	ROI_iBL_BR_Param[1] = p1.x - p0.x;	// x1-x0
	ROI_iBL_BR_Param[2] = (p1.y - p0.y)*p0.x + (p0.x - p1.x)*p0.y; //(y1-y0)x0 + (x0-x1)y0

	p0 = (Point2f)ROI_iBR;
	p1 = (Point2f)ROI_iTR;
	ROI_iBR_TR_Param[0] = p0.y - p1.y;	// a = yo-y1
	ROI_iBR_TR_Param[1] = p1.x - p0.x;	// b = x1-x0
	ROI_iBR_TR_Param[2] = (p1.y - p0.y)*p0.x + (p0.x - p1.x)*p0.y; // c = (y1-y0)x0 + (x0-x1)y0

	p0 = (Point2f)ROI_iTL;
	p1 = (Point2f)ROI_iTR;
	ROI_iTR_TL_Param[0] = p0.y - p1.y;	// yo-y1
	ROI_iTR_TL_Param[1] = p1.x - p0.x;	// x1-x0
	ROI_iTR_TL_Param[2] = (p1.y - p0.y)*p0.x + (p0.x - p1.x)*p0.y; //(y1-y0)x0 + (x0-x1)y0

	p0 = (Point2f)ROI_iBL;
	p1 = (Point2f)ROI_iTL;
	ROI_iTL_BL_Param[0] = p0.y - p1.y;	// yo-y1
	ROI_iTL_BL_Param[1] = p1.x - p0.x;	// x1-x0
	ROI_iTL_BL_Param[2] = (p1.y - p0.y)*p0.x + (p0.x - p1.x)*p0.y; //(y1-y0)x0 + (x0-x1)y0
}

Void CarSnukt::InputROIandPersMap(Mat &tmpI)
{
	// Temperal variables for the ROI and perspective map
	bool isSettingDone = false;
	bool isSatisfied = false;
	int Per_Point_Cnt = 0;
	Mat sI;
	Rect rect;
	char str[200];
	Point2i p;
	vector<Point2i> ROI_P;
	vector<Point2i> PER_P;

	// ROI setting here (for non-static ROI)
	cv::namedWindow("ROI_Set", WINDOW_AUTOSIZE);
	while (!isSettingDone)
	{
		// Bottom-left
		isSatisfied = false;
		std::printf("Chose the Bottom-Left point of the ROI\n");

		while (!isSatisfied)
		{
			tmpI.copyTo(sI);
			rect = Rect(0, 0, tmpI.cols / 2, tmpI.rows / 2);
			rectangle(sI, rect, Scalar(0, 0, 255), 2);
			sprintf(str, "Chose the Bottom-Left point of the ROI");
			putText(sI, str, Point2i((int)(tmpI.cols / 10), (int)(tmpI.rows / 1.5)), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255, 255), 2);
			cv::setMouseCallback("ROI_Set", CallBackFunc, &p);
			if ((p.x < tmpI.cols / 2) && (p.y < tmpI.rows / 2) && (p.x>0) && (p.y>0))
//			if ((p.x < tmpI.cols) && (p.y < tmpI.rows) && (p.x>0) && (p.y>0))
			{
				isSatisfied = true;
				ROI_P.push_back(p);
			}
			imshow("ROI_Set", sI);
			cv::waitKey(1);
		}

		// Bottom-right
		isSatisfied = false;
		std::printf("Chose the Bottom-Right point of the ROI\n");
		while (!isSatisfied)
		{
			tmpI.copyTo(sI);
			rect = Rect(tmpI.cols / 2, 0, tmpI.cols / 2, tmpI.rows / 2);
			rectangle(sI, rect, Scalar(0, 0, 255), 2);
			sprintf(str, "Chose the Bottom-Right point of the ROI");
			putText(sI, str, Point2i((int)(tmpI.cols / 10), (int)(tmpI.rows / 1.5)), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255, 255), 2);
			cv::setMouseCallback("ROI_Set", CallBackFunc, &p);
			if ((p.x < tmpI.cols) && (p.y < tmpI.rows / 2) && (p.x>tmpI.cols / 2) && (p.y>0))
			{
				isSatisfied = true;
				ROI_P.push_back(p);
			}
			imshow("ROI_Set", sI);
			cv::waitKey(1);
		}

		// Top-right
		isSatisfied = false;
		std::printf("Chose the Top-Right point of the ROI\n");
		while (!isSatisfied)
		{
			tmpI.copyTo(sI);
			rect = Rect(tmpI.cols / 2, tmpI.rows / 2, tmpI.cols / 2, tmpI.rows / 2);
			rectangle(sI, rect, Scalar(0, 0, 255), 2);
			sprintf(str, "Chose the Top-Right point of the ROI");
			putText(sI, str, Point2i((int)(tmpI.cols / 10), int(tmpI.rows / 2.5)), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255, 255), 2);
			cv::setMouseCallback("ROI_Set", CallBackFunc, &p);
			if ((p.x < tmpI.cols) && (p.y < tmpI.rows) && (p.x>tmpI.cols / 2) && (p.y>tmpI.rows / 2))
			{
				isSatisfied = true;
				ROI_P.push_back(p);
			}
			imshow("ROI_Set", sI);
			cv::waitKey(1);
		}

		// Top-left
		isSatisfied = false;
		std::printf("Chose the Top-Left point of the ROI\n");
		while (!isSatisfied)
		{
			tmpI.copyTo(sI);
			rect = Rect(0, tmpI.rows / 2, tmpI.cols / 2, tmpI.rows / 2);
			rectangle(sI, rect, Scalar(0, 0, 255), 2);
			sprintf(str, "Chose the Top-Left point of the ROI");
			putText(sI, str, Point2i((int)(tmpI.cols / 10), (int)(tmpI.rows / 2.5)), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255, 255), 2);
			cv::setMouseCallback("ROI_Set", CallBackFunc, &p);
			if ((p.x < tmpI.cols / 2) && (p.y < tmpI.rows) && (p.x>0) && (p.y>tmpI.rows / 2))
			{
				isSatisfied = true;
				ROI_P.push_back(p);
			}
			imshow("ROI_Set", sI);
			cv::waitKey(1);
		}
		destroyWindow("ROI_Set");
		isSettingDone = true;
	}
#if PERS_VIEW
	ImHorSize = Trans_W;
	ImVerSize = Trans_H;
	FormROI(Point2i(1, 1), Point2i(Trans_W - 2, 1), Point2i(Trans_W - 2, Trans_H - 2), Point2i(1, Trans_H - 2));
#else
	FormROI(ROI_P.at(0), ROI_P.at(1), ROI_P.at(2), ROI_P.at(3));
#endif

#if IS_USE_PER_TRANS
	// chose 4 marker points of the top-down perspective 
	isSettingDone = false;
	isSatisfied = false;
	printf("Choose 4 points for the perspective transformation\n");
	namedWindow("PerspectiveMap_Set", WINDOW_AUTOSIZE);
	while (!isSettingDone)
	{
		// Draw the current IMG and the ROI
		tmpI.copyTo(sI);
		// Draw the outer ROI
		line(tmpI, ROI_BL, ROI_BR, Scalar(255, 0, 255), 2, 4, 0);
		line(tmpI, ROI_BR, ROI_TR, Scalar(255, 0, 255), 2, 4, 0);
		line(tmpI, ROI_TR, ROI_TL, Scalar(255, 0, 255), 2, 4, 0);
		line(tmpI, ROI_TL, ROI_BL, Scalar(255, 0, 255), 2, 4, 0);

		// Draw the inner ROI
		line(tmpI, ROI_iBL, ROI_iBR, Scalar(255, 0, 0), 2, 4, 0);
		line(tmpI, ROI_iBR, ROI_iTR, Scalar(255, 0, 0), 2, 4, 0);
		line(tmpI, ROI_iTR, ROI_iTL, Scalar(255, 0, 0), 2, 4, 0);
		line(tmpI, ROI_iTL, ROI_iBL, Scalar(255, 0, 0), 2, 4, 0);

		// chose 4 points for the perspective transformation
		p = Point2i(0, 0);
		while (Per_Point_Cnt < 4)
		{
			tmpI.copyTo(sI);
			sprintf(str, "Chose the %d-th point of the perspective transformation, inside the inner ROI", Per_Point_Cnt);
			putText(sI, str, Point2i((int)(tmpI.cols / 20), (int)(tmpI.rows / 1.5)), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255, 255), 2);
			setMouseCallback("PerspectiveMap_Set", CallBackFunc, &p);
			Mat PointArea = (Mat_<int>(1, 4) << p.x, p.x + 1, p.y, p.y + 1);
			//cout << p << "\n";
			if (CheckInsideROI(PointArea) && p.x != 0 && p.y != 0)
			{
				PER_P.push_back(p);
				isSatisfied = true;
				Per_Point_Cnt++;
				p = Point2i(0, 0);
			}
			imshow("PerspectiveMap_Set", sI);
			waitKey(1);
		}
		destroyWindow("PerspectiveMap_Set");
		isSettingDone = true;
	}

	tmpI.copyTo(sI);
	FormTransBGM(sI, PER_P.at(0), PER_P.at(1), PER_P.at(2), PER_P.at(3), Trans_W, Trans_H);

#endif
}

Void CarSnukt::UpdateBGM(Mat &I)
{
	UpdateS(I);
	UpdateBs();
}

Void CarSnukt::CarSnuktDet(Mat &I, Mat &lastI)
{
	vector<Mat> MVO_ROI;
	vector<Mat> MVO_SEG;
	vector<bool> isLargeObject;
	vector<int> hardIdCode; //redundant 

#if AUTO_CAR_DETECTION
	vector<bool> isAutoCar;
#endif

	MOVDetector(I, MVO_ROI, MVO_SEG, isLargeObject);

#if AUTO_CAR_DETECTION
	trackAutoCar(MVO_ROI, isAutoCar);
#endif

		LargeMVOTracking(I, MVO_SEG, MVO_ROI, isLargeObject, hardIdCode);
		//cout << "LargeMVOTracking done" << endl;
#if SEND_DATA
		prepareSendData();
#endif
		// Small MVOs refinement
		SmallROIRefine(isLargeObject, MVO_ROI, SmallObjectROI);



		// Annotation
#if DEBUG_FINAL
		Annotation(I, SmallObjectROI);
#endif
	
}

Void CarSnukt::MOVDetector(const Mat &I, vector<Mat> &MOV_ROI, vector<Mat> &MVO_SEG, vector<bool> &isLargeObject) {
#if DETECTOR_BG
	if (isBsAvai)
	{
		// Update the current BG
		if (!isBAvai) {
			B = Bs;
		}

		// Initialize the masks	and needed variables		
		vector<bool> isInROI;
		vector<vector<Point2i> > CriticPntsVec;

		// Background suppression then filter it through the ROI mask
#if PERS_VIEW
		B = TransBGM;
#endif
		//imshow("I", I);
		//imshow("B", B);
		//waitKey(0);
		BGSuppress(I, B, FG);

		// Ghost objects suppression
#if BGM_DYNAMIC
#if GHOST_REMOVE
		GhostDet(I, lastI, FG, MVOSH, MVOGSH);
#else
		MVOSH = FG;
		MVOGSH = FG - MVOSH;
#endif
#else
		MVOSH = FG;
		MVOGSH = FG - MVOSH;
#endif
		// Shadow detection
		ShadowDet(I, B, MVOSH, MVO_ROI, MVO_SEG);

		// Knowledge-based background model update (having low impacts to the system, optional)
#if BGM_KNOWLEDGE
		BGMKnowledgedBasedUpdate(I, Bs, isBAvai, B, MVOGSH);
#endif		

		// Detect the large MVOs (cars, trucs, etc.)
		LargeMVODetection(I, MVO_SEG, MVO_ROI, isLargeObject);

#if AUTO_CAR_DETECTION
		detectAutoCar(I, MVO_SEG, MVO_ROI, isLargeObject, isAutoCar);
#endif

#elif DETECTOR_YOLO
	MOV_ROI.clear();
	isLargeObject.clear();
	
	vector<bbox_t> yoloResutVec = pYolo->detect(I);
	cout << "yolo size: "<<yoloResutVec.size() << endl;
	for (int i = 0; i < yoloResutVec.size(); i++) {
		// (x1, y1)	.	. 
		//		.	.	.
		//		.	.	(x2, y2)
		int id, x1, x2, y1, y2;

		x1 = yoloResutVec[i].x;
		x2 = x1 + yoloResutVec[i].w;
		y1 = yoloResutVec[i].y;
		y2 = y1 + yoloResutVec[i].h;
		Mat objRect = (Mat_<int>(1, 4) << x1, x2, y1, y2);
		MOV_ROI.push_back(objRect);
		//cout << x1 << " " << x2 << " " << y1 << " " << y2<< endl;
		Mat objSeg= I(Range(y1, y2), Range(x1, x2));
		MVO_SEG.push_back(objSeg);

		id = yoloResutVec[i].obj_id;
		switch (id){
		case 0: //car
			isLargeObject.push_back(1);
			break;
		case 1: //pedestrian
			isLargeObject.push_back(0);
			break;
		default:
			break;
		}
	}

#endif
}

inline Void CarSnukt::Annotation(Mat &I, vector<Mat> &SmallObjectROI)
{
	// Important, only coppy
	Mat tmpI; I.copyTo(tmpI);
	Mat tmpBGMTrans; TransBGM.copyTo(tmpBGMTrans);

#if DEBUG_TARGET_LINE
#if CAR_0 == 1
	for (uint8_t j = R0_PATH_0; j <= R0_PATH_5; j++){
		Point2d tmp[4];
		tmp[0] = Point2d(mRouteMap.mEntrInfo[j].minX, mRouteMap.mEntrInfo[j].minY);
		tmp[1] = Point2d(mRouteMap.mEntrInfo[j].minX, mRouteMap.mEntrInfo[j].maxY);
		tmp[2] = Point2d(mRouteMap.mEntrInfo[j].maxX, mRouteMap.mEntrInfo[j].minY);
		tmp[3] = Point2d(mRouteMap.mEntrInfo[j].maxX, mRouteMap.mEntrInfo[j].maxY);
		circle(tmpI, tmp[0], 1, Scalar(0, 250, 250), 3, 4, 0);
		circle(tmpI, tmp[1], 1, Scalar(0, 250, 250), 3, 4, 0);
		circle(tmpI, tmp[2], 1, Scalar(0, 250, 250), 3, 4, 0);
		circle(tmpI, tmp[3], 1, Scalar(0, 250, 250), 3, 4, 0);
		line(tmpI, tmp[0], tmp[1], Scalar(0, 250, 250), 2, 4, 0);
		line(tmpI, tmp[0], tmp[2], Scalar(0, 250, 250), 2, 4, 0);
		line(tmpI, tmp[2], tmp[3], Scalar(0, 250, 250), 2, 4, 0);
		line(tmpI, tmp[1], tmp[3], Scalar(0, 250, 250), 2, 4, 0);

		if (mRouteMap.mTargetLine[j].expo.isUsed){
			for (unsigned i = 0; i < NUM_ROUTE_SAMPLE; i++){
				Point2d tmp = mRouteMap.mTargetLine[j].expo.Samples.at(i);
				circle(tmpI, tmp, 1, Scalar(0, 255, 0), 1, 4, 0);
			}
		}
		else if (mRouteMap.mTargetLine[j].line.isUsed){
			line(tmpI, mRouteMap.mTargetLine[j].line.A, mRouteMap.mTargetLine[j].line.B, Scalar(0, 255, 0), 2, 4, 0);
		}
	}
#endif
#endif
	// display tracking-related objects' information
	if (countNonZero(LiveObjList) > 0)
	{
		Mat NonZ;
		findNonZero(LiveObjList, NonZ);
		for (uint8_t i = 0; i < NonZ.total(); i++)
		{
			uint8_t ID = NonZ.at<Point2i>(i).x;

			// Draw the bounding boxes in the image plane
			Mat CurROI = TrackObj[ID].ROI;
			Rect rect(MAX(CurROI.at<int>(0) - 2, 0),
				MAX(CurROI.at<int>(2) - 2, 0),
				MIN(CurROI.at<int>(1) - CurROI.at<int>(0) + 4, I.cols - 1),
				MIN(CurROI.at<int>(3) - CurROI.at<int>(2) + 4, I.rows - 1)
				);
			rectangle(tmpI, rect, Scalar(0, 0, 255), 1);

			// Draw the object center in the image plane
			circle(tmpI, TrackObj[ID].CenterImgPlane, 3, Scalar(0, 0, 255), 2, 4, 0);

#if DETECT_DIRECTION
			// Draw the object head-tail in the image plane
			circle(tmpI, TrackObj[ID].Head, 3, Scalar(0, 255, 0), 2, 4, 0);
			circle(tmpI, TrackObj[ID].Tail, 3, Scalar(255, 0, 0), 2, 4, 0);
			line(tmpI, TrackObj[ID].Head, TrackObj[ID].Tail, Scalar(255, 255, 0), 2, 4, 0);
#endif

			// Draw the object IDs
			char str[200];
#if SEND_DATA
			sprintf(str, "%d", TrackObj[ID].dataCamToCar.id);
#else
			sprintf(str, "%d", TrackObj[ID].SoftID);
#endif
			putText(tmpI, str, Point2i(CurROI.at<int>(0), CurROI.at<int>(2)), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255, 255), 2);

#if TRANSFORM_CRITICAL_POINT			
			// Draw the critical points in the image plane
			vector<Point2i> tmpCriPnts = TrackObj[ID].CriticPntsVec;
			for (uint8_t j = 0; j < tmpCriPnts.size(); j++)
			{
				circle(tmpI, tmpCriPnts.at(j), 1, Scalar(0, 255, 255), 2, 4, 0);
			}

			// Draw the object direction in the transformed plane
			circle(tmpI, TrackObj[ID].Direction, 3, Scalar(0, 255, 255), 2, 4, 0);

			// Draw the object direction in the image plane
			line(tmpI, TrackObj[ID].CenterImgPlane, TrackObj[ID].Direction, Scalar(0, 255, 0), 2, 4, 0);
			Mat CurTransROI = TrackObj[ID].TransROI;
			Rect rectTrans(MAX(CurTransROI.at<int>(0) - 2, 0),
				MAX(CurTransROI.at<int>(2) - 2, 0),
				MIN(CurTransROI.at<int>(1) - CurTransROI.at<int>(0) + 4, tmpBGMTrans.cols - 1),
				MIN(CurTransROI.at<int>(3) - CurTransROI.at<int>(2) + 4, tmpBGMTrans.rows - 1)
				);
			rectangle(tmpBGMTrans, rectTrans, Scalar(0, 0, 255), 1);

			// Draw the object moving state
			if (TrackObj[ID].isMoving)
			{
				sprintf(str, "Move");
			}
			else
			{
				sprintf(str, "Stop");
			}
			putText(tmpI, str, Point2i(CurROI.at<int>(0), CurROI.at<int>(3)), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255, 255), 2);

			//line(tmpBGMTrans, TrackObj[ID].CenterTrans, TrackObj[ID].DirectionTrans, Scalar(255, 0, 255), 2, 4, 0);
			// transformed centroid	and direction					
			circle(tmpBGMTrans, TrackObj[ID].CenterTrans, 1, Scalar(0, 255, 255), 2, 4, 0);
#endif
		}
	}
#if DEBUG_AUTO_CAR_DETECTION
	if (countNonZero(LiveAutoCarList) > 0)
	{
		Mat NonZ;
		findNonZero(LiveAutoCarList, NonZ);
		for (uint8_t i = 0; i < NonZ.total(); i++)
		{
			uint8_t ID = NonZ.at<Point2i>(i).x;

			// Draw the bounding boxes in the image plane
			Mat CurROI = autoCar[ID].ROI;
			Rect rect(MAX(CurROI.at<int>(0) - 2, 0),
				MAX(CurROI.at<int>(2) - 2, 0),
				MIN(CurROI.at<int>(1) - CurROI.at<int>(0) + 4, I.cols - 1),
				MIN(CurROI.at<int>(3) - CurROI.at<int>(2) + 4, I.rows - 1)
			);
			rectangle(tmpI, rect, Scalar(255, 0, 0), 1);

			// Draw the object center in the image plane
			//circle(tmpI, TrackObj[ID].CenterImgPlane, 3, Scalar(0, 0, 255), 2, 4, 0);

#if DETECT_DIRECTION
			// Draw the object head-tail in the image plane
			circle(tmpI, autoCar[ID].Head, 3, Scalar(0, 255, 0), 2, 4, 0);
			circle(tmpI, autoCar[ID].Tail, 3, Scalar(255, 0, 0), 2, 4, 0);
			line(tmpI, autoCar[ID].Head, autoCar[ID].Tail, Scalar(255, 255, 0), 2, 4, 0);
#endif

			// Draw the object IDs
			char str[200];
#if SEND_DATA
			sprintf(str, "%d", autoCar[ID].dataCamToCar.id);
#else
			sprintf(str, "%d", autoCar[ID].SoftID);
#endif
			putText(tmpI, str, Point2i(CurROI.at<int>(0), CurROI.at<int>(2)), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 2);

#if TRANSFORM_CRITICAL_POINT			
			// Draw the critical points in the image plane
			vector<Point2i> tmpCriPnts = autoCar[ID].CriticPntsVec;
			for (uint8_t j = 0; j < tmpCriPnts.size(); j++)
			{
				circle(tmpI, tmpCriPnts.at(j), 1, Scalar(0, 255, 255), 2, 4, 0);
			}

			// Draw the object direction in the transformed plane
			circle(tmpI, autoCar[ID].Direction, 3, Scalar(0, 255, 255), 2, 4, 0);

			// Draw the object direction in the image plane
			line(tmpI, autoCar[ID].CenterImgPlane, autoCar[ID].Direction, Scalar(0, 255, 0), 2, 4, 0);
			Mat CurTransROI = autoCar[ID].TransROI;
			Rect rectTrans(MAX(CurTransROI.at<int>(0) - 2, 0),
				MAX(CurTransROI.at<int>(2) - 2, 0),
				MIN(CurTransROI.at<int>(1) - CurTransROI.at<int>(0) + 4, tmpBGMTrans.cols - 1),
				MIN(CurTransROI.at<int>(3) - CurTransROI.at<int>(2) + 4, tmpBGMTrans.rows - 1)
			);
			rectangle(tmpBGMTrans, rectTrans, Scalar(0, 0, 255), 1);

			// Draw the object moving state
			if (autoCar[ID].isMoving)
			{
				sprintf(str, "Move");
			}
			else
			{
				sprintf(str, "Stop");
			}
			putText(tmpI, str, Point2i(CurROI.at<int>(0), CurROI.at<int>(3)), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255, 255), 2);

			//line(tmpBGMTrans, TrackObj[ID].CenterTrans, TrackObj[ID].DirectionTrans, Scalar(255, 0, 255), 2, 4, 0);
			// transformed centroid	and direction					
			circle(tmpBGMTrans, autoCar[ID].CenterTrans, 1, Scalar(0, 255, 255), 2, 4, 0);
#endif
		}
	}
#endif

#if	EXCLUDE_SMALL_MVOS_IN_TRK
#else
	// display small objects' information in the 2D image
	for (size_t i = 0; i < SmallObjectROI.size(); i++)
	{
		Mat CurROI = SmallObjectROI.at(i);
		Rect rect(MAX(CurROI.at<int>(0) - 2, 0),
			MAX(CurROI.at<int>(2) - 2, 0),
			MIN(CurROI.at<int>(1) - CurROI.at<int>(0) + 4, tmpI.cols - 1),
			MIN(CurROI.at<int>(3) - CurROI.at<int>(2) + 4, tmpI.rows - 1)
			);
		rectangle(tmpI, rect, Scalar(0, 255, 0), 1);
	}
#endif

	// Draw the outer ROI
	line(tmpI, ROI_BL, ROI_BR, Scalar(255, 0, 255), 2, 4, 0);
	line(tmpI, ROI_BR, ROI_TR, Scalar(255, 0, 255), 2, 4, 0);
	line(tmpI, ROI_TR, ROI_TL, Scalar(255, 0, 255), 2, 4, 0);
	line(tmpI, ROI_TL, ROI_BL, Scalar(255, 0, 255), 2, 4, 0);

	// Draw the inner ROI
	line(tmpI, ROI_iBL, ROI_iBR, Scalar(255, 0, 0), 2, 4, 0);
	line(tmpI, ROI_iBR, ROI_iTR, Scalar(255, 0, 0), 2, 4, 0);
	line(tmpI, ROI_iTR, ROI_iTL, Scalar(255, 0, 0), 2, 4, 0);
	line(tmpI, ROI_iTL, ROI_iBL, Scalar(255, 0, 0), 2, 4, 0);

#if FULL_SCREEN	
	namedWindow("Annotation", CV_WINDOW_NORMAL);
	cvSetWindowProperty("Annotation", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
#endif
	imshow("Annotation", tmpI);

#if IS_USE_PER_TRANS
	imshow("Top-down perspective mapping", tmpBGMTrans);
#endif


#if DEBUG_VELOCITY
	Mat IforGps;
	I.copyTo(IforGps);

	if (countNonZero(LiveObjList) > 0)
	{
		Mat NonZ;
		findNonZero(LiveObjList, NonZ);
		for (uint8_t i = 0; i < NonZ.total(); i++)
		{
			uint8_t ID = NonZ.at<Point2i>(i).x;

			// Draw the object center in the image plane
			circle(tmpI, TrackObj[ID].CenterImgPlane, 3, Scalar(0, 0, 255), 2, 4, 0);
			string velocity = TrackObj[ID].dataCamToCar.vx + ", " + TrackObj[ID].dataCamToCar.vy;
			putText(IforGps, velocity, TrackObj[ID].CenterImgPlane, FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255, 255), 1);
			
		}
	}
	imshow("velocity", IforGps);

#endif
	cv::waitKey(1);
}

inline Void CarSnukt::prepareSendData() {
	int numOfObj = countNonZero(LiveObjList);
	uint64_t t, intervalT;
	time.getCurT(t);
	time.getTDiff(intervalT);
	Point2l curGps(0, 0);

	if (numOfObj > 0) {
		Mat NonZ;
		findNonZero(LiveObjList, NonZ);
		for (size_t i = 0; i < NonZ.total(); i++) {
			uint8_t ID = NonZ.at<Point2i>(i).x;
			camToCar* pCamToCar = & TrackObj[ID].dataCamToCar;
			//id
			pCamToCar->id = TrackObj[ID].vID;

			//timestamp
			pCamToCar->tStmp = t;

			//lat, lon
			int32_t pastLat, pastLon;
#if PIXEL2GPS_HOMOGRAPHY
			pixel2gps.getTargetGps64INT(TrackObj[ID].CenterImgPlane, curGps);
#elif PIXEL2GPS_TABLE
			//cout << TrackObj[ID].CenterImgPlane << endl;
			gpsT.getGps64INT(TrackObj[ID].CenterImgPlane, curGps);
#endif
			pastLat = pCamToCar->latitude;
			pastLon = pCamToCar->longitude;
			pCamToCar->latitude = curGps.x;
			pCamToCar->longitude = curGps.y;

			//vx, vy
			TrackObj[ID].gpsVel.getVelocity(intervalT, pCamToCar->latitude, pCamToCar->longitude, pCamToCar->vx, pCamToCar->vy);

		}
	}
#if AUTO_CAR_DETECTION
	int numOfAutoCar = countNonZero(LiveAutoCarList);
	if (numOfAutoCar > 0) {
		Mat NonZ;
		findNonZero(LiveAutoCarList, NonZ);
		for (size_t i = 0; i < NonZ.total(); i++) {
			uint8_t ID = NonZ.at<Point2i>(i).x;
			camToCar* pCamToCar = &autoCar[ID].dataCamToCar;
			//id
			pCamToCar->id = autoCar[ID].vID;

			//timestamp
			pCamToCar->tStmp = t;

			//lat, lon
			int32_t pastLat, pastLon;
			pixel2gps.getTargetGps64INT(autoCar[ID].CenterImgPlane, curGps);
			pastLat = pCamToCar->latitude;
			pastLon = pCamToCar->longitude;
			pCamToCar->latitude = curGps.x;
			pCamToCar->longitude = curGps.y;

			//vx, vy
			autoCar[ID].gpsVel.getVelocity(intervalT, pCamToCar->latitude, pCamToCar->longitude, pCamToCar->vx, pCamToCar->vy);

		}
	}
#endif
}

inline bool CarSnukt::isAutoCar(const Mat &img, const Mat& curSEG, const Mat &curROI, int &colCnt) {
	//1. crop roi
	Mat roiImg = img(Range(curROI.at<int>(2), curROI.at<int>(3) + 1),
		Range(curROI.at<int>(0), curROI.at<int>(1) + 1));
	//Mat roiImg = Mat::ones(curROI.at<int>(3) + 1- curROI.at<int>(2)+1, curROI.at<int>(1) + 1 - curROI.at<int>(0)+1, CV_8UC1);

#if DEBUG_AUTO_CAR_DETECTION
	//namedWindow("roiImg", WINDOW_NORMAL);
	//resizeWindow("roiImg", roiImg.cols*2, roiImg.rows * 2);
	//imshow("roiImg", roiImg);
	//waitKey();
	//destroyWindow("roiImg");

#endif

	//2. color detection
	Mat thrImg;

	colDet.getThrImg(roiImg, thrImg);
#if DEBUG_AUTO_CAR_DETECTION
	//namedWindow("thrImg", WINDOW_NORMAL);
	//resizeWindow("thrImg", thrImg.cols*1.5, thrImg.rows * 1.5);
	//imshow("thrImg", thrImg);
	//waitKey();
	//destroyWindow("thrImg");

	//namedWindow("curSEG", WINDOW_NORMAL);
	//resizeWindow("curSEG", curSEG.cols*1.5, curSEG.rows * 1.5);
	//imshow("curSEG", curSEG);
	//waitKey();
	//destroyWindow("curSEG");
#endif

	//3. find intersection of SEG & COLOR
	Mat interSec;
	//cv::bitwise_and(curSEG, thrImg, interSec);
#if DEBUG_AUTO_CAR_DETECTION
	//namedWindow("interSec", WINDOW_NORMAL);
	//resizeWindow("interSec", interSec.cols*1.5, interSec.rows * 1.5);
	//imshow("interSec", interSec);
	//waitKey();
	//destroyWindow("interSec");
#endif

	//4. count intersection pixel.
	//int interSecCnt = countNonZero(interSec);
	int interSecCnt = countNonZero(thrImg);
#if DEBUG_AUTO_CAR_DETECTION
	//cout << "interSecCnt : " << interSecCnt << endl;
#endif
	if (interSecCnt> NUM_COLOR_PIXEL_THR) {
		colCnt = interSecCnt;
		return true;
	}
	else {
		return false;
	}
}

Void CarSnukt::changeToPers(Mat &I) {
	warpPerspective(I, I, TransMat, Size(Trans_W, Trans_H));
	B = TransBGM;
	//imshow("B trnas", B);
	//imshow("I trnas", I);
	//waitKey();
}

Int CarSnukt::getDataToSend(vector<camToCar> & vecCamToCar) const
{
	vecCamToCar.clear();

	int numOfObj = countNonZero(LiveObjList);
	if (numOfObj > 0) {
		Mat NonZ;
		findNonZero(LiveObjList, NonZ);
		for (size_t i = 0; i < NonZ.total(); i++) {
			uint8_t ID = NonZ.at<Point2i>(i).x;
			vecCamToCar.push_back(TrackObj[ID].dataCamToCar);
		}
	}

	int numOfAutoCar = countNonZero(LiveAutoCarList);
	if (numOfAutoCar> 0) {
		Mat NonZ;
		findNonZero(LiveAutoCarList, NonZ);
		for (size_t i = 0; i < NonZ.total(); i++) {
			uint8_t ID = NonZ.at<Point2i>(i).x;
			vecCamToCar.push_back(autoCar[ID].dataCamToCar);
		}
	}

	return numOfObj+numOfAutoCar;
}

Void CarSnukt::updateT(const SYSTEMTIME &t) {
	time.updateT(t);
}

Void CarSnukt::setPixel2Gps(const Point2f(&pixel)[4], const Point2d(&gps)[4],
	const int latSameDigit, const int lonSameDigit,
	const int latWholeDigit, const int lonWholeDigit,
	const int latPrecision, const int lonPrecision)
{
	pixel2gps.setGps(pixel, gps, latSameDigit, lonSameDigit, latWholeDigit, lonWholeDigit, latPrecision, lonPrecision);
}

Void CarSnukt::setPixel2GpsTable(const char* gpsTName, const int col, const int row) {
	gpsT.setGpsTable(gpsTName, col, row);
}


Void CarSnukt::setGps2Pixel(const Point2d(&gps)[4], const Point2f(&pixel)[4],
	const int latSameDigit, const int lonSameDigit,
	const int latWholeDigit, const int lonWholeDigit,
	const int latPrecision, const int lonPrecision)
{
	gps2pixel.setGps2Pixel(gps, pixel, latSameDigit, lonSameDigit, latWholeDigit, lonWholeDigit, latPrecision, lonPrecision);
}

Void CarSnukt::getTargetPixel(Point2l& gps, Point2d& targetPixel) 
{
	gps2pixel.getTargetPixel(gps, targetPixel);
}

Void CarSnukt::detectAutoCar(const Mat &img, const vector<Mat> &mvoSeg, const vector<Mat> &mvoRoi, vector<bool> &isLargeObj, vector<bool> &isAutoCarVec) {
	bool isThereAutoCar = false; 
	int maxColCnt;
	int colCnt;
	int autoCarCandIdx = -1;

	//imshow("detectAutoCar Img", img);
	//waitKey();
	for (int i = 0; i < isLargeObj.size(); i++) {
		//1. check if it's large obejct and auto car.
		if (isLargeObj[i] && isAutoCar(img, mvoSeg[i], mvoRoi[i], colCnt))
		{
			//2. If there has been no Auto Car, or current MVO is more like autoCar,
			if ((!isThereAutoCar) || (isThereAutoCar && (colCnt > maxColCnt))) {
				//3. change auto Car info.
				maxColCnt = colCnt;
				autoCarCandIdx = i;
				isThereAutoCar = true;
			}
		}
	}

	//4. Erase isLagreObj info for MVO tracking. 
	if (isThereAutoCar) {
		isLargeObj[autoCarCandIdx] = false;
		//cout << "isLargeObj["<<autoCarCandIdx<<"] : "<<isLargeObj[autoCarCandIdx] << endl;
	}

	//5. update  isAutoCar vector
	for (int i = 0; i < isLargeObj.size(); i++) {
		if (i == autoCarCandIdx) {
			isAutoCarVec.push_back(true);
		}
		else {
			isAutoCarVec.push_back(false);
		}
	}

}

void CarSnukt::trackAutoCar(const vector<Mat> &MVO_ROI, vector<bool> &isAutoCar)
{
	// 1. if there are new auto car
	if (countNonZero(LiveAutoCarList) == 0)
	{
		for (size_t i = 0; i < MVO_ROI.size(); i++)
		{
			if (isAutoCar.at(i))
			{
				// 1) LiveAutoCarList update
				LiveAutoCarList.at<uint8_t>(0, NewAutoTrackObj) = 1;
				// 2) set soft id, update vid
				autoCar[NewAutoTrackObj].SoftID = NewAutoTrackObj;
				if (autoVID > AUTO_CAR_VIRTUAL_ID_END) {
					autoVID = AUTO_CAR_VIRTUAL_ID_START;
				}
				autoCar[NewAutoTrackObj].vID = autoVID++;
				
				//3) set ROI;
				Mat curROI = MVO_ROI.at(i);
				autoCar[NewAutoTrackObj].ROI = curROI;
				///update the centerimag from ROI
				Point2i CurPos = Point2i((curROI.at<int>(0) + curROI.at<int>(1)) >> 1,
					(curROI.at<int>(2) + curROI.at<int>(3)) >> 1);
				autoCar[NewAutoTrackObj].CenterImgPlane = CurPos;

				// 3) resetGps
				autoCar[NewAutoTrackObj].gpsVel.resetIsFirstMoment();
				// 4) newAutoTrackObj++ % LIVE_TRACK_OBJECT_SIZE
				NewAutoTrackObj++;
				NewAutoTrackObj = NewAutoTrackObj% LIVE_AUTO_CAR_SIZE;
			}
		}
	}else //2. track the current existing live objects		
	{
		bool isThereMatchAutoCar = false;
		for (size_t i = 0; i < MVO_ROI.size(); i++)
		{
			if (isAutoCar.at(i))
			{
				//if there is match car, update the info.
				Mat curROI = MVO_ROI.at(i);
				autoCar[NewAutoTrackObj].ROI = curROI;
				///update the centerimag from ROI
				Point2i CurPos = Point2i((curROI.at<int>(0) + curROI.at<int>(1)) >> 1,
					(curROI.at<int>(2) + curROI.at<int>(3)) >> 1);
				autoCar[NewAutoTrackObj].CenterImgPlane =  CurPos;
				isThereMatchAutoCar = true;
			}
		}

		if (!isThereMatchAutoCar) {// if there is no match car
			Mat NonZIdx;
			findNonZero(LiveAutoCarList, NonZIdx);
			for (int i = 0; i < NonZIdx.total(); i++) {
				int TrackObjIdx = NonZIdx.at<Point2i>(i).x;
				LiveAutoCarList.at<uint8_t>(0, TrackObjIdx) = 0;
			}

		}
	}

}
