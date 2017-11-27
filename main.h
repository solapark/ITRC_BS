#pragma once
#ifndef _MAIN_H_
#define _MAIN_H_

#include "CarSnukt.h"

// Variables, structures and classes
Mat				I, II, III;
uint32_t		FirstImgIdx = 0;
uint32_t		LastImgIdx = 0;
uint32_t		ImgIdx = 0;
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


#if STATIC_IMAGEf
inline Mat  ReadImage(int ImgIdx);
#elif VIDEO || CAMERA
inline Mat ReadImage(VideoCapture &cap);
#endif 

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
	cap >> I;	
#endif	
#if SEND_DATA
	GetLocalTime(&now);
#endif
	assert(I.empty() != 1 && "Cannot read imagel");

#if CAMERA
#if REOPEN_CAM_WHEN_TIME_OVER
	isTimeOver = false;
	process_start = clock();
#endif
#endif

	I.convertTo(I, CV_32FC3, 1 / 255.0);
	GaussianBlur(I, I, cv::Size(5, 5), 0.3);
	return I;
}

#endif // !_MAIN_H_