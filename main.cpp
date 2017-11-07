// ITRCBS.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"

#include "main.h"
#include <time.h>

int main() {
	
	// assertions 
	assert((VIDEO + CAMERA + STATIC_IMAGE) == 1 && "Only one type of input should be enable at a time");
	//assert((ANN + RANTREE + SIZE_SALIENCY) == 1 && "Only one type of classification should be enable at a time");
	assert(BGM_STABLE_CNT > BGM_N);

#if VIDEO
	// open the video file
	VideoCapture cap(VIDEO_FILE);
	assert(cap.isOpened() && "Cannot open the video file");
	double count = cap.get(CV_CAP_PROP_FRAME_COUNT);
	assert(LAST_IMG_IDX <= count && "LAST_IMG_IDX is too high");
#elif CAMERA
	VideoCapture cap(CAM_ID); // open the default camera
	assert(cap.isOpened() && "Cannot open the camera");
#endif

#if VIDEO || CAMERA 
	uint32_t ImgIdx = 0;
	bool isStop = false;
#endif	 
	bool isFirstFrame = true;


// read the background image 
#if BGM_FIRST_BUILD
	Mat firstFrame;
#if STATIC_IMAGE
	char ImgName[100];
	sprintf(ImgName, "%s%0.4d%s", DATASET_DIR, ImgIdx, FILE_EXT);
	//cout << ImgName << endl;
	firstFrame = imread(ImgName);
#elif VIDEO||CAMERA
	cap >> firstFrame;
#endif
	assert(firstFrame.empty() != 1 && "Cannot first frame for bgl");
	myCarSnukt.LoadBG(firstFrame);
#else
	myCarSnukt.LoadBG();
#endif

#if	BGM_BUILD_WATITING_FRAME
	int numberOfSkip = 0;
#endif

#if STATIC_IMAGE
	for (uint32_t tmpImgIdx = FIRST_IMG_IDX; tmpImgIdx < LAST_IMG_IDX; tmpImgIdx = tmpImgIdx + 1)
#elif VIDEO || CAMERA
	// read images from a video file
	while (!isStop)
#endif
	{
#if DEBUG_RUNNING_TIME
		clock_t t_arr[100];
		int i = 0;
		t_arr[i++] = clock();
#endif
#if STATIC_IMAGE
		Mat tmpI = ReadImage(tmpImgIdx);
		ImgIdx = tmpImgIdx - FIRST_IMG_IDX;
#elif VIDEO || CAMERA
		// Read images from the video or a camera
		Mat tmpI = ReadImage(cap);
#endif
		//deliver time stamp to myCarSnukt
#if SEND_DATA
		myCarSnukt.updateT(now);
#endif
#if DEBUG_RUNNING_TIME
		t_arr[i++] = clock();
		cout << "ReadImage " << t_arr[i - 1] - t_arr[i - 2] << endl;
#endif
		// Initilization once
		if (isFirstFrame){
//			printf("Define the ROI and the image region for the perspective transformation.\n");
			myCarSnukt.Initialize(tmpI.rows, tmpI.cols);

#if STATIC_ROI
			myCarSnukt.FormROI(ROI_BL, ROI_BR, ROI_TR, ROI_TL);
#if IS_USE_PER_TRANS
			myCarSnukt.FormTransBGM(tmpI, Trans_BL, Trans_BR, Trans_TR, Trans_TL, Trans_W, Trans_H);
#endif
#if PIXEL2GPS
			Point2f pixel[4] = { pixel0, pixel1, pixel2, pixel3 };
			Point2d gps[4] = { mapDouble0, mapDouble1, mapDouble2, mapDouble3 };
			myCarSnukt.setPixel2Gps(pixel, gps, LAT_SAME_DIGIT, LON_SAME_DIGIT, LAT_WHOLE_DIGIT, LON_WHOLE_DIGIT, LAT_PRECISION, LON_PRECISION);
#if DEBUG_GPS
			myCarSnukt.setGps2Pixel(gps,pixel, LAT_SAME_DIGIT, LON_SAME_DIGIT, LAT_WHOLE_DIGIT, LON_WHOLE_DIGIT, LAT_PRECISION, LON_PRECISION);
#endif

#endif
#else
			myCarSnukt.InputROIandPersMap(tmpI);
#endif		
			isFirstFrame = false;
		}
#if PERS_VIEW
		myCarSnukt.changeToPers(tmpI);
#endif

		// Update history images
		if (ImgIdx <= 3){
			III = tmpI;	II = tmpI; I = tmpI;
		}
		else{
			III = II; II = I; I = tmpI;
		}

#if DEBUG_RUNNING_TIME
		t_arr[i++] = clock();
		cout << "Update history images " << t_arr[i - 1] - t_arr[i - 2] << endl;
#endif
#if DEBUG_IMG_IDX
		char str[200];
		sprintf(str, "idx = %d", ImgIdx);
		putText(I, str, Point2f(1, 10), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255, 255), 2);
#endif
		// CarSnukt algorithm 
#if BGM_DYNAMIC
#if WAIT_BGM_BUILD
		if (!isBgInitEnd) {
//			cout << "NOT initBgIsOver" << endl;
			if ((ImgIdx%INITAIL_BGM_DT) == 0) {
				myCarSnukt.UpdateBGM(I);
			}
		}
		else {
//			cout << "initBgIsOver" << endl;
			if ((ImgIdx%BGM_DT) == 0) {
					myCarSnukt.UpdateBGM(I);
				}
		}
#else
		if ((ImgIdx%BGM_DT) == 0) {
			myCarSnukt.UpdateBGM(I);
#if SAVE_NEW_BG
			myCarSnukt.StoreBG();
		}
#else
		}
#endif
#endif


#if DEBUG_RUNNING_TIME
		t_arr[i++] = clock();
		cout << "BGM update " << t_arr[i - 1] - t_arr[i - 2] << endl;
#endif

#if DEBUG_IMG_IDX
		//if (ImgIdx < TARGET_IMG_IDX) {
		//	ImgIdx++;
		//	continue;
		//}

#endif
		// CarSnukt Detector 
#if	WAIT_BGM_BUILD
		if (numberOfSkip < BGM_BUILD_WATITING_FRAME) {
			numberOfSkip++;
		}
		else {
			isBgInitEnd = true;
			myCarSnukt.CarSnuktDet(I, III);

		}

#else
		myCarSnukt.CarSnuktDet(I, III);
#endif
#if DEBUG_RUNNING_TIME
		t_arr[i++] = clock();
		cout << "CarSnukt Detector " << t_arr[i - 1] - t_arr[i - 2] << endl;
#endif
#else 

		// background update if applicable
		if ((ImgIdx%BGM_DT) == 0 && (BGM_update_cnt < BGM_STABLE_CNT) && !myCarSnukt.isBGLoaded())
		{
			myCarSnukt.UpdateBGM(I);
			BGM_update_cnt++;
		}
#if LOAD_STORE_BG
		if (myCarSnukt.GetBGMStatus())
			myCarSnukt.StoreBG();
#endif
		// CarSnukt Detector (main program)		 
		if (myCarSnukt.GetBGMStatus()) // background formation done
		{
			// the main algorithm
			myCarSnukt.CarSnuktDet(I, III);		
		}
#endif 

#if SEND_DATA
#if DEBUG_GPS
		Mat IforGPS = I;
		Point2f targetPixel;

#endif
#if	WAIT_BGM_BUILD
		if (numberOfSkip == 1) {
			cout << "Waiting for backgound image built...." << endl;
		}
		if (numberOfSkip < BGM_BUILD_WATITING_FRAME) {}
		else {
			vector<camToCar> dataToSend;
			size_t numOfObj = myCarSnukt.getDataToSend(dataToSend);
			for (size_t i = 0; i < numOfObj; i++) {
				cout << "ID : " << dataToSend[i].id
					<< "   TimeStamp : " << dataToSend[i].tStmp
					<< "   lat, lon: " << dataToSend[i].latitude << ", " << dataToSend[i].longitude
					<< "   Vx, Vy : " << dataToSend[i].vx << ", " << dataToSend[i].vy
					<< "   Heading, width, length  : " << dataToSend[i].heading
					<< ", " << dataToSend[i].width << ", " << dataToSend[i].length << endl;
#if DEBUG_GPS
				myCarSnukt.getTargetPixel(Point2l(dataToSend[i].latitude, dataToSend[i].longitude), targetPixel);
				circle(IforGPS, targetPixel, 1, Scalar(0, 255, 0), 3);
#endif
			}
			cout << "********************************************" << endl;
#if DEBUG_GPS
			imshow("gpsTest", IforGPS);
			waitKey(1);
#endif
		}
#if DEBUG_RUNNING_TIME
		t_arr[i++] = clock();
		cout << "Send data " << t_arr[i - 1] - t_arr[i - 2] << endl;
#endif
#else
		vector<camToCar> dataToSend;
		size_t numOfObj = myCarSnukt.getDataToSend(dataToSend);
		for (size_t i = 0; i < numOfObj; i++) {
			cout << "ID : " << dataToSend[i].id
				<< "   TimeStamp : " << dataToSend[i].tStmp
				<< "   long, lat : " << dataToSend[i].longitude << ", " << dataToSend[i].latitude
				<< "   Vx, Vy : " << dataToSend[i].vx << ", " << dataToSend[i].vy
				<< "   Heading, width, length  : " << dataToSend[i].heading
				<< ", " << dataToSend[i].width << ", " << dataToSend[i].length << endl;
		}
		cout << "********************************************" << endl;
#endif
#endif

		// Check the termination condition		
#if VIDEO
		ImgIdx++;
		if (ImgIdx > LAST_IMG_IDX)
		{
			isStop = true;
		}
#elif CAMERA
		ImgIdx++;
		ImgIdx %= 100000;
#endif

#if DEBUG_RUNNING_TIME
		t_arr[i++] = clock();
		cout << "Whole " << t_arr[i - 1] - t_arr[0] << endl;
		cout << "********************************************" << endl;
#endif

#if CAMERA
#if REOPEN_CAM_WHEN_TIME_OVER
		//waitKey();
		if (clock() - process_start > TIME_LIMIT) {
			isTimeOver = true;
		}

		if (isTimeOver) {
			cout << "Process exceed time limit. Reopen cam" << endl;
			cap.release();
			if (!cap.open(CAM_ID)) {
				std::cout << "Error opening video stream or file" << std::endl;
				return -1;
			}
		}
#endif
#endif
	}

	return(0);
}

