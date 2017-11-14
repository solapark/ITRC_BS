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
	transMat = Mat(3, 3, CV_64F);

	assert(lonWholeDigit - lonSameDigit <= 6 && latWholeDigit - latSameDigit <= 6 && "MUST wholeDigit-sameDigit <= 6");

	Point2f gpsFloat[4];
	uint64_t latInt, lonInt;
	float latFloat, lonFloat;

	for (int i = 0; i < 4; i++) {
		latInt = gps[i].x * pow(10, latWholeDigit - 2);
		lonInt = gps[i].y * pow(10, lonWholeDigit - 3);
		//		cout << "latInt ,lonInt : " << lonInt << ", " << latInt << endl;
		lonOffset = lonInt / pow(10, lonWholeDigit - lonSameDigit);
		lonOffset *= pow(10, lonWholeDigit - lonSameDigit);
		latOffset = latInt / pow(10, latWholeDigit - latSameDigit);
		latOffset *= pow(10, latWholeDigit - latSameDigit);
		//		cout << "lonOffset ,latOffset: " << lonOffset << ", " << latOffset << endl;
		lonFloat = lonInt - lonOffset;
		latFloat = latInt - latOffset;
		//		cout << "lonFloat ,latFloat: " << lonFloat << ", " << latFloat << endl;
		gpsFloat[i] = Point2f(latFloat, lonFloat);
		//		cout << "gpsFloat[i] : " << gpsFloat[i] << endl;
	}
	transMat = getPerspectiveTransform(gpsFloat, pixel);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			transArr[i][j] = transMat.at<double>(i, j);
			//cout << transArr[i][j] << endl;
		}
	}
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
	transMat = Mat(3, 3, CV_64F);
	assert(lonWholeDigit - lonSameDigit <= 6 && latWholeDigit - latSameDigit <= 6 && "MUST wholeDigit-sameDigit <= 6");

	Point2f gpsFloat[4];
	uint64_t latInt, lonInt;
	float latFloat, lonFloat;

	for (int i = 0; i < 4; i++) {
		latInt = gps[i].x * pow(10, latWholeDigit - 2);
		lonInt = gps[i].y * pow(10, lonWholeDigit - 3);
		//		cout << "latInt ,lonInt : " << lonInt << ", " << latInt << endl;
		lonOffset = lonInt / pow(10, lonWholeDigit - lonSameDigit);
		lonOffset *= pow(10, lonWholeDigit - lonSameDigit);
		latOffset = latInt / pow(10, latWholeDigit - latSameDigit);
		latOffset *= pow(10, latWholeDigit - latSameDigit);
		//		cout << "lonOffset ,latOffset: " << lonOffset << ", " << latOffset << endl;
		lonFloat = lonInt - lonOffset;
		latFloat = latInt - latOffset;
		//		cout << "lonFloat ,latFloat: " << lonFloat << ", " << latFloat << endl;
		gpsFloat[i] = Point2f(latFloat, lonFloat);
		//		cout << "gpsFloat[i] : " << gpsFloat[i] << endl;
	}
	transMat = getPerspectiveTransform(gpsFloat, pixel);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			transArr[i][j] = transMat.at<double>(i, j);
			//cout << transArr[i][j] << endl;
		}
	}
}

void gps2Pixel::getTargetPixel(const Point2d &targetGpsDouble, Point2d& targetPixel)
{
	Point2l targetGpsInt;
	Point2d targetGpsDbl;
	targetGpsInt.x = targetGpsDouble.x *pow(10, latWholeDigit - 2);
	targetGpsInt.y = targetGpsDouble.y *pow(10, lonWholeDigit - 3);
	//	cout << "targetGpsInt : " << targetGpsInt << endl;
	targetGpsDbl.x = targetGpsInt.x - latOffset;
	targetGpsDbl.y = targetGpsInt.y - lonOffset;
	//	cout << "targetGpsFloat : " << targetGpsFloat << endl;
	calcTargetGps(targetGpsDbl, targetPixel);
	//	cout << "targetPixel : " << targetPixel << endl;
}

void gps2Pixel::getTargetPixel(const Point2l &targetGpsLong, Point2d& targetPixel)
{
	//	Point2l targetGpsInt;
	Point2d targetGpsDbl;
	//	targetGpsInt.x = targetGpsLong.x *pow(10, latWholeDigit - 2);
	//	targetGpsInt.y = targetGpsLong.y *pow(10, lonWholeDigit - 3);
	//	cout << "targetGpsInt : " << targetGpsInt << endl;
	targetGpsDbl.x = targetGpsLong.x - latOffset;
	targetGpsDbl.y = targetGpsLong.y - lonOffset;
	//	cout << "targetGpsFloat : " << targetGpsFloat << endl;
	calcTargetGps(targetGpsDbl, targetPixel);
	//	cout << "targetPixel : " << targetPixel << endl;
}

void gps2Pixel::calcTargetGps(const Point2d& targetGps, Point2d &targetPixel) {
	double xOrg = targetGps.x;
	double yOrg = targetGps.y;

	double w = transArr[2][0] * xOrg + transArr[2][1] * yOrg + transArr[2][2];

	double xp = round((transArr[0][0] * xOrg + transArr[0][1] * yOrg + transArr[0][2]) / w);
	double yp = round((transArr[1][0] * xOrg + transArr[1][1] * yOrg + transArr[1][2]) / w);
	//cout << "xp : " << xp << ", " << "yp : " << yp << endl;
	targetPixel.x = xp;
	targetPixel.y = yp;

}