#include "stdafx.h"
#include "gps2Pixel.h"

gps2Pixel::gps2Pixel() {}
gps2Pixel::gps2Pixel(
	const Point2d(&gps)[4], const Point2f(&pixel)[4],
	const int latSameDigit, const int lonSameDigit,
	const int latWholeDigit, const int lonWholeDigit,
	const int latPrecision, const int lonPrecision)
	:latWholeDigit(latWholeDigit), lonWholeDigit(lonWholeDigit), latPrecision(latPrecision), lonPrecision(lonPrecision)
{
	A = Mat(1, 1, CV_32FC2);
	B = Mat(1, 1, CV_32FC2);
	assert(lonWholeDigit - lonSameDigit <= 6 && latWholeDigit - latSameDigit <= 6 && "MUST wholeDigit-sameDigit <= 6");

	Point2f gpsFloat[4];
	uint64_t latInt, lonInt;
	float latFloat, lonFloat;

	for (int i = 0; i < 4; i++) {
		latInt = gps[i].x * pow(10, latWholeDigit - 2);
		lonInt = gps[i].y * pow(10, lonWholeDigit - 3);
		cout << "latInt ,lonInt : " << lonInt << ", " << latInt << endl;
		lonOffset = lonInt / pow(10, lonWholeDigit - lonSameDigit);
		lonOffset *= pow(10, lonWholeDigit - lonSameDigit);
		latOffset = latInt / pow(10, latWholeDigit - latSameDigit);
		latOffset *= pow(10, latWholeDigit - latSameDigit);
		cout << "lonOffset ,latOffset: " << lonOffset << ", " << latOffset << endl;
		lonFloat = lonInt - lonOffset;
		latFloat = latInt - latOffset;
		cout << "lonFloat ,latFloat: " << lonFloat << ", " << latFloat << endl;
		gpsFloat[i] = Point2f(latFloat, lonFloat);
		cout << "gpsFloat[i] : " << gpsFloat[i] << endl;
	}
	transMat = getPerspectiveTransform(gpsFloat, pixel);
}

void gps2Pixel::setGps2Pixel(
	const Point2d(&gps)[4], const Point2f(&pixel)[4],
	const int latSameDigit, const int lonSameDigit,
	const int latWholeDigit, const int lonWholeDigit,
	const int latPrecision, const int lonPrecision) 
{
	this->latWholeDigit = latWholeDigit;
	this->lonWholeDigit = lonWholeDigit;
	this->latPrecision = latPrecision;
	this->lonPrecision = lonPrecision;

	A = Mat(1, 1, CV_32FC2);
	B = Mat(1, 1, CV_32FC2);
	assert(lonWholeDigit - lonSameDigit <= 6 && latWholeDigit - latSameDigit <= 6 && "MUST wholeDigit-sameDigit <= 6");

	Point2f gpsFloat[4];
	uint64_t latInt, lonInt;
	float latFloat, lonFloat;

	for (int i = 0; i < 4; i++) {
		latInt = gps[i].x * pow(10, latWholeDigit - 2);
		lonInt = gps[i].y * pow(10, lonWholeDigit - 3);
		cout << "latInt ,lonInt : " << lonInt << ", " << latInt << endl;
		lonOffset = lonInt / pow(10, lonWholeDigit - lonSameDigit);
		lonOffset *= pow(10, lonWholeDigit - lonSameDigit);
		latOffset = latInt / pow(10, latWholeDigit - latSameDigit);
		latOffset *= pow(10, latWholeDigit - latSameDigit);
		cout << "lonOffset ,latOffset: " << lonOffset << ", " << latOffset << endl;
		lonFloat = lonInt - lonOffset;
		latFloat = latInt - latOffset;
		cout << "lonFloat ,latFloat: " << lonFloat << ", " << latFloat << endl;
		gpsFloat[i] = Point2f(latFloat, lonFloat);
		cout << "gpsFloat[i] : " << gpsFloat[i] << endl;
	}
	transMat = getPerspectiveTransform(gpsFloat, pixel);
}

void gps2Pixel::getTargetPixel(const Point2d &targetGpsDouble, Point2f& targetPixel)
{
	Point2l targetGpsInt;
	Point2f targetGpsFloat;
	targetGpsInt.x = targetGpsDouble.x *pow(10, latWholeDigit - 2);
	targetGpsInt.y = targetGpsDouble.y *pow(10, lonWholeDigit - 3);
	cout << "targetGpsInt : " << targetGpsInt << endl;
	targetGpsFloat.x = targetGpsInt.x - latOffset;
	targetGpsFloat.y = targetGpsInt.y - lonOffset;
	cout << "targetGpsFloat : " << targetGpsFloat << endl;
	calcTargetGps(targetGpsFloat, targetPixel);
	targetPixel.x = (int)round(targetPixel.x);
	targetPixel.y = (int)round(targetPixel.y);
	cout << "targetPixel : " << targetPixel << endl;
}

void gps2Pixel::getTargetPixel(const Point2l &targetGpsLong, Point2f& targetPixel)
{
//	Point2l targetGpsInt;
	Point2f targetGpsFloat;
//	targetGpsInt.x = targetGpsLong.x *pow(10, latWholeDigit - 2);
//	targetGpsInt.y = targetGpsLong.y *pow(10, lonWholeDigit - 3);
//	cout << "targetGpsInt : " << targetGpsInt << endl;
	targetGpsFloat.x = targetGpsLong.x - latOffset;
	targetGpsFloat.y = targetGpsLong.y - lonOffset;
	cout << "targetGpsFloat : " << targetGpsFloat << endl;
	calcTargetGps(targetGpsFloat, targetPixel);
	targetPixel.x = (int)round(targetPixel.x);
	targetPixel.y = (int)round(targetPixel.y);
	cout << "targetPixel : " << targetPixel << endl;
}

void gps2Pixel::calcTargetGps(const Point2f& targetGpS, Point2f &targetPixel) {
	A.at<Point2f>(0) = targetGpS;
	perspectiveTransform(A, B, transMat);
	targetPixel = (Point2f)B.at<Point2f>(0);
}