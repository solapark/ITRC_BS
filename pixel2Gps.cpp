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
	transMat = Mat(3, 3, CV_64F);

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
		//cout << "lonFloat ,latFloat: " << lonFloat << ", " << latFloat << endl;
		gpsFloat[i] = Point2f(latFloat, lonFloat);
		//			cout << "gpsFloat[i] : " << gpsFloat[i] << endl;
	}
	transMat = getPerspectiveTransform(pixel, gpsFloat);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			transArr[i][j] = transMat.at<double>(i, j);
			//			cout << transArr[i][j] << endl;
		}
	}

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
	transMat = Mat(3, 3, CV_64F);
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
		//cout << "lonFloat ,latFloat: " << lonFloat << ", " << latFloat << endl;
		gpsFloat[i] = Point2f(latFloat, lonFloat);
		//			cout << "gpsFloat[i] : " << gpsFloat[i] << endl;
	}
	transMat = getPerspectiveTransform(pixel, gpsFloat);
	//	cout << "transMat : " << transMat << endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			transArr[i][j] = transMat.at<double>(i, j);
			//			cout << transArr[i][j] << endl;
		}
	}
}


void pixel2Gps::getTargetGps64INT(const Point2f targetPixel, Point2l &targetGPSInt)
{
	Point2d targetGpsDouble;
	calcTargetGps(targetPixel, targetGpsDouble);
	//		cout << "targetGpsFloat : " << targetGpsFloat << endl;
	targetGPSInt.x = targetGpsDouble.x + latOffset;
	targetGPSInt.y = targetGpsDouble.y + lonOffset;
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

void pixel2Gps::calcTargetGps(const Point2f targetPixel, Point2d &targetGPS) {
	float xOrg = targetPixel.x;
	float yOrg = targetPixel.y;

	double w = transArr[2][0] * xOrg + transArr[2][1] * yOrg + transArr[2][2];

	double xp = round((transArr[0][0] * xOrg + transArr[0][1] * yOrg + transArr[0][2]) / w);
	double yp = round((transArr[1][0] * xOrg + transArr[1][1] * yOrg + transArr[1][2]) / w);
	//cout << "xp : " << xp << ", " << "yp : " << yp << endl;
	targetGPS.x = xp;
	targetGPS.y = yp;
}