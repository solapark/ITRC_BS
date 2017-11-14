#include "stdafx.h"
#include "gpsVelocity.h"
gpsVelocity::gpsVelocity(){}

gpsVelocity::gpsVelocity(const int latPrcsn, const int lonPrcsn, const int velPrcsn)
:isFirstMoment(true){
	latDivider = pow(10, latPrcsn);
	lonDivider = pow(10, lonPrcsn);
	velFactor = pow(10, velPrcsn);
	velFactorWs = velFactor / 1000;		//1000 = ms to s
	cout << "latDiver : " << latDivider << endl;
	cout << "lonDiver : " << lonDivider << endl;
	cout << "velFoctor: " << velFactor << endl;
	cout << "velFactorWs : " << velFactorWs << endl;

}

void gpsVelocity::setGpsVelocity(const int latPrcsn, const int lonPrcsn, const int velPrcsn, const int16_t velLimit) {
	latDivider = pow(10, latPrcsn);
	lonDivider = pow(10, lonPrcsn);
	velFactor = pow(10, velPrcsn);
	this->velLimit = velLimit;
	
	//cout << "latDiver : " << latDivider << endl;
	//cout << "lonDiver : " << lonDivider << endl;
	//cout << "velFoctor: " << velFactor << endl;
}

void gpsVelocity::resetIsFirstMoment()
{
	isFirstMoment = true;
	latWholeTime = 0;
	lonWholeTime = 0;
}

void gpsVelocity::getVelocity(const uint64_t timeMs, const int32_t curLatInt, const int32_t curLonInt, int16_t &latVelInt, int16_t &lonVelInt) {
	if (isFirstMoment) {
		latVelInt16 = latVelInt = 0;
		lonVelInt16 = lonVelInt = 0;
		pastLat = curLatInt / latDivider;
		pastLon = curLonInt / lonDivider;
		//latWholeTime += 30;
		//lonWholeTime += 30;
		//latWholeTime += timeMs;
		//lonWholeTime += timeMs;
		isFirstMoment = false;
	}
	else {
		curLat = curLatInt / latDivider;
		curLon = curLonInt / lonDivider;
		cout.precision(10);
		double latDist = gpsDist(pastLat, 0, curLat, 0);
		double lonDist = gpsDist(0, pastLon, 0, curLon);

		//latWholeTime += timeMs;
		//lonWholeTime += timeMs;
		latWholeTime = timeMs;
		lonWholeTime = timeMs;
		//latWholeTime = 30;
		//lonWholeTime = 30;
		latVelInt32 = latDist / ((double)latWholeTime / 1000)*velFactor;
		lonVelInt32 = lonDist / ((double)lonWholeTime / 1000)*velFactor;

		if (abs(latVelInt32) > velLimit || abs(lonVelInt32) > velLimit) {
			//	//cout << "error" << endl;
			latVelInt16 = latVelInt = 0;
			lonVelInt16 = lonVelInt = 0;
			isFirstMoment = true;
		}
		else {
			latVelInt16 = latVelInt = latVelInt32;
			lonVelInt16 = lonVelInt = lonVelInt32;
			pastLat = curLat;
			pastLon = curLon;
		}

		//if (abs(latVelInt32) > velLimit || abs(lonVelInt32) > velLimit ) {
		//	//cout << "error" << endl;
		//	latVelInt16 = latVelInt = 0;
		//	lonVelInt16 = lonVelInt = 0;
		//	latWholeTime = 0;
		//	lonWholeTime = 0;
		//}
		//else 
		//{
		//	if (latVelInt32 == 0 && lonVelInt32 == 0) {//use past velocity
		//		latVelInt = latVelInt16;
		//		lonVelInt = lonVelInt16;
		//	}
		//	else if (latVelInt32 == 0) {
		//		latVelInt = latVelInt16;
		//		lonVelInt16 = lonVelInt = lonVelInt32;
		//		lonWholeTime = 0;
		//	}
		//	else if (lonVelInt32 == 0) {
		//		lonVelInt = lonVelInt16;
		//		latVelInt16 = latVelInt = latVelInt32;
		//		latWholeTime = 0;
		//	}
		//	else
		//	{
		//		latVelInt16 = latVelInt = latVelInt32;
		//		lonVelInt16 = lonVelInt = lonVelInt32;
		//		latWholeTime = 0;
		//		lonWholeTime = 0;
		//	}
		//}

		//cout << "latVelInt, lonVelInt : " << latVelInt << " m/s, " << lonVelInt << " m/s" << endl;
	}

}

void gpsVelocity::getVelocity(int16_t &latVelInt, int16_t &lonVelInt) {
	latVelInt = latVelInt16;
	lonVelInt = lonVelInt16;
}


/**
* Returns the distance between two points on the Earth.
* Direct translation from http://en.wikipedia.org/wiki/Haversine_formula
* @param lat1d Latitude of the first point in degrees
* @param lon1d Longitude of the first point in degrees
* @param lat2d Latitude of the second point in degrees
* @param lon2d Longitude of the second point in degrees
* @return The distance between the two points in meters
*/
double gpsVelocity::gpsDist(const double lat1d, const double lon1d, double const lat2d, double const lon2d) {
	double lat1r, lon1r, lat2r, lon2r, u, v;
	lat1r = deg2rad(lat1d);
	lon1r = deg2rad(lon1d);
	lat2r = deg2rad(lat2d);
	lon2r = deg2rad(lon2d);
	u = sin((lat2r - lat1r) / 2);
	v = sin((lon2r - lon1r) / 2);
	if (lat1d > lat2d || lon1d > lon2d) {
		return -(2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v)) * 1000);
	}
	else {
		return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v)) * 1000;
	}
}

double gpsVelocity::deg2rad(const double deg) {

	return (deg * M_PI / 180);
}

//  This function converts radians to decimal degrees
double gpsVelocity::rad2deg(const double rad) {
	return (rad * 180 / M_PI);
}