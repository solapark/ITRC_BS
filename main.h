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

#if STATIC_IMAGE
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
//	clock_t t_arr[100];
	int i = 0;
	//t_arr[i++] = clock();

	Mat I;
#if STATIC_IMAGE
	char ImgName[100];
	sprintf(ImgName, "%s%0.4d%s", DATASET_DIR, ImgIdx, FILE_EXT);
	//cout << ImgName << endl;
	I = imread(ImgName);
#elif VIDEO || CAMERA
	cap >> I;	
#endif	
#if SEND_DATA
	GetLocalTime(&now);
#endif
	//t_arr[i++] = clock();
	//cout << "ReadImage() cap >> I " << t_arr[i - 1] - t_arr[i - 2] << endl;

	assert(I.empty() != 1 && "Cannot read imagel");
//	cout << "I.type() : " << I.type() << endl;
//	resize(I, I, Size(SIZE_HOR, SIZE_VER));
	//t_arr[i++] = clock();
	//cout << "ReadImage() resize " << t_arr[i - 1] - t_arr[i - 2] << endl;

	I.convertTo(I, CV_32FC3, 1 / 255.0);
	//t_arr[i++] = clock();
	//cout << "ReadImage() I.convertTo " << t_arr[i - 1] - t_arr[i - 2] << endl;

	GaussianBlur(I, I, cv::Size(5, 5), 0.3);
	//t_arr[i++] = clock();
	//cout << "ReadImage() GaussianBlur " << t_arr[i - 1] - t_arr[i - 2] << endl;
	return I;
}

#endif // !_MAIN_H_