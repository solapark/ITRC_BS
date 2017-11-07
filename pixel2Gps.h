#include "stdafx.h"
#pragma once
#ifndef _PIXEL2GPS_H
#define _PIXEL2GPS_H
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace std;
using namespace cv;

class pixel2Gps {
public:
	pixel2Gps();
	pixel2Gps(
		const Point2f(&pixel)[4], const Point2d(&gps)[4],
		const int latSameDigit, const int lonSameDigit,
		const int latWholeDigit, const int lonWholeDigit,
		const int latPrecision, const int lonPrecision);

	void setGps(
		const Point2f(&pixel)[4], const Point2d(&gps)[4],
		const int latSameDigit, const int lonSameDigit,
		const int latWholeDigit, const int lonWholeDigit,
		const int latPrecision, const int lonPrecision);

	void getTargetGps64INT(const Point2f targetPixel, Point2l &targetGPSInt);
	void getTargetGpsDouble(const Point2f targetPixel, Point2d &targetGPSDouble);

private:
	Mat A, B, transMat;
	uint64_t lonOffset, latOffset;
	int lonPrecision, latPrecision;
	void calcTargetGps(const Point2f targetPixel, Point2f &targetGPS);
};
#endif // !_PIXEL2GPS_H
