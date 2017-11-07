#include "stdafx.h"
#include "pixel2Gps.h"

pixel2Gps::pixel2Gps() {}
pixel2Gps::pixel2Gps(
	const Point2f(&pixel)[4], const Point2d(&gps)[4],
	const int latSameDigit, const int lonSameDigit,
	const int latWholeDigit, const int lonWholeDigit,
	const int latPrecision, const int lonPrecision)
	:latPrecision(latPrecision), lonPrecision(lonPrecision)
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
		//			cout << "lonInt ,latInt : " << lonInt << ", " << latInt << endl;
		lonOffset = lonInt / pow(10, lonWholeDigit - lonSameDigit);
		lonOffset *= pow(10, lonWholeDigit - lonSameDigit);
		latOffset = latInt / pow(10, latWholeDigit - latSameDigit);
		latOffset *= pow(10, latWholeDigit - latSameDigit);
		//			cout << "lonOffset ,latOffset: " << lonOffset << ", " << latOffset << endl;
		lonFloat = lonInt - lonOffset;
		latFloat = latInt - latOffset;
		cout << "lonFloat ,latFloat: " << lonFloat << ", " << latFloat << endl;
		gpsFloat[i] = Point2f(lonFloat, latFloat);
		//			cout << "gpsFloat[i] : " << gpsFloat[i] << endl;
	}
	transMat = getPerspectiveTransform(pixel, gpsFloat);
}

void pixel2Gps::setGps(
	const Point2f(&pixel)[4], const Point2d(&gps)[4],
	const int latSameDigit, const int lonSameDigit,
	const int latWholeDigit, const int lonWholeDigit,
	const int latPrecision, const int lonPrecision)
{
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
		//			cout << "lonInt ,latInt : " << lonInt << ", " << latInt << endl;
		lonOffset = lonInt / pow(10, lonWholeDigit - lonSameDigit);
		lonOffset *= pow(10, lonWholeDigit - lonSameDigit);
		latOffset = latInt / pow(10, latWholeDigit - latSameDigit);
		latOffset *= pow(10, latWholeDigit - latSameDigit);
		//			cout << "lonOffset ,latOffset: " << lonOffset << ", " << latOffset << endl;
		lonFloat = lonInt - lonOffset;
		latFloat = latInt - latOffset;
		cout << "lonFloat ,latFloat: " << lonFloat << ", " << latFloat << endl;
		gpsFloat[i] = Point2f(lonFloat, latFloat);
		//			cout << "gpsFloat[i] : " << gpsFloat[i] << endl;
	}
	transMat = getPerspectiveTransform(pixel, gpsFloat);
}

void pixel2Gps::getTargetGps64INT(const Point2f targetPixel, Point2l &targetGPSInt)
{
	Point2f targetGpsFloat;
	calcTargetGps(targetPixel, targetGpsFloat);
	//		cout << "targetGpsFloat : " << targetGpsFloat << endl;
	targetGPSInt.x = targetGpsFloat.y + latOffset;
	targetGPSInt.y = targetGpsFloat.x + lonOffset;
	//cout << "targetGPSInt : " << targetGPSInt << endl;
}

void pixel2Gps::getTargetGpsDouble(const Point2f targetPixel, Point2d &targetGPSDouble)
{
	Point2l targetGpsInt;
	getTargetGps64INT(targetPixel, targetGpsInt);
	targetGPSDouble.x = targetGpsInt.x / (double)pow(10, latPrecision);
	targetGPSDouble.y = targetGpsInt.y / (double)pow(10, lonPrecision);
	cout.precision(10);
	cout << "targetGPSDouble : " << targetGPSDouble << endl;

}

void pixel2Gps::calcTargetGps(const Point2f targetPixel, Point2f &targetGPS) {
	A.at<Point2f>(0) = targetPixel;
	perspectiveTransform(A, B, transMat);
	targetGPS = (Point2f)B.at<Point2f>(0);
}