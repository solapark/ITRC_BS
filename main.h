#pragma once
#ifndef _MAIN_H_
#define _MAIN_H_

#include "CarSnukt.h"

// Variables, structures and classes
Mat				I, II, III;
uint32_t		FirstImgIdx = 0;
uint32_t		LastImgIdx = 0;
uint32_t		ImgIdx = FIRST_IMG_IDX;
Int				BGM_update_cnt = 0;
CarSnukt		myCarSnukt;
SYSTEMTIME		now;

// sending out variables
bool start = false;
double PredDisError = 0.0f;
double CurDisError = 0.0f;
Point2i CurPos(0, 0);
int key;
bool isBgInitEnd = false;
//to reopen cam when processing time > TIME_LIMIT
bool isTimeOver = false;
double process_start;


#if STATIC_IMAGE
inline Mat ReadImage(int ImgIdx)
#elif VIDEO || CAMERA
inline Mat ReadImage(VideoCapture &cap)
#endif
{
	int i = 0;

	Mat I;
#if STATIC_IMAGE
	char ImgName[100];
	sprintf(ImgName, FILE_FORMAT, DATASET_DIR, ImgIdx, FILE_EXT);
	//cout << ImgName << endl;
	I = imread(ImgName);

#elif VIDEO || CAMERA
	//clock_t timeStart = clock();
	cap >> I;	
//cout << "cap >> I = " << clock() - timeStart << endl;
#endif	
#if SEND_DATA
	GetLocalTime(&now);
#endif
	//assert(I.empty() != 1 && "Cannot read imagel");
#if CAMERA
	while (I.empty()  && "Cannot read imagel") {
		cap.release();
		if (!cap.open(CAM_ID)) {
			std::cout << "Error opening video stream or file" << std::endl;
			continue;
		}
		cap >> I;
	}
#endif

#if CAMERA
#if REOPEN_CAM_WHEN_TIME_OVER
	isTimeOver = false;
	process_start = clock();
#endif
#endif

#if DETECTOR_BG
	I.convertTo(I, CV_32FC3, 1 / 255.0);
	GaussianBlur(I, I, cv::Size(5, 5), 0.3);
#endif
	return I;
}

#endif // !_MAIN_H_