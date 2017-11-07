#include "stdafx.h"
#pragma once

#ifndef _GPS2PIXEL_H
#define _GPS2PIXEL_H
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace std;
using namespace cv;

class gps2Pixel{
public:
	gps2Pixel();
	gps2Pixel(
		const Point2d(&gps)[4], const Point2f(&pixel)[4],
		const int latSameDigit, const int lonSameDigit,
		const int latWholeDigit, const int lonWholeDigit,
		const int latPrecision, const int lonPrecision);
	void setGps2Pixel(
		const Point2d(&gps)[4], const Point2f(&pixel)[4],
		const int latSameDigit, const int lonSameDigit,
		const int latWholeDigit, const int lonWholeDigit,
		const int latPrecision, const int lonPrecision);
	void getTargetPixel(const Point2d &targetGpsDouble, Point2f& targetPixel);
	void getTargetPixel(const Point2l &targetGpsLong, Point2f& targetPixel);


private:
	Mat A, B, transMat;
	int lonWholeDigit, latWholeDigit;
	uint64_t lonOffset, latOffset;
	int lonPrecision, latPrecision;
	void calcTargetGps(const Point2f& targetGpS, Point2f &targetPixel);
};
#endif // !_PIXEL2GPS_H

