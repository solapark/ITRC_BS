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
	wholeTime = 0;
}

void gpsVelocity::getVelocity(const uint64_t timeMs, const int32_t curLatInt, const int32_t curLonInt, int16_t &latVelInt, int16_t &lonVelInt) {
	if (isFirstMoment) {
//		cout << "isFirstMoment "<< endl;
		latVelInt16 = latVelInt = 0;
		lonVelInt16 = lonVelInt = 0;
		pastLat = curLatInt / latDivider;
		pastLon = curLonInt / lonDivider;
		//wholeTime += timeMs;
		wholeTime += 30;
		isFirstMoment = false;
	}
	else {
		curLat = curLatInt / latDivider;
		curLon = curLonInt / lonDivider;
		cout.precision(10);
		cout << "***************" << endl;
		//cout << "timeMs : "<< timeMs << endl;
		cout << "pastLat, pastLon : " << pastLat << ", " << pastLon << endl;
		cout << "curLat, curLon : " << curLat << ", " << curLon << endl;
		double latDist = gpsDist(pastLat, 0, curLat, 0);
		double lonDist = gpsDist(0, pastLon, 0, curLon);
		//cout << "timeMs : " << timeMs << endl;
		cout.precision(11);
		cout << "latDist , lonDist : " << latDist << " m, " << lonDist << " m"<< endl;
	
		//wholeTime += timeMs;
		wholeTime += 30;

		if (latDist == 0 && lonDist == 0) {
			latVelInt = latVelInt16;
			lonVelInt = lonVelInt16;
		}
		else {
			latVelInt32 = latDist / ((double)wholeTime / 1000)*velFactor;
			lonVelInt32 = lonDist / ((double)wholeTime / 1000)*velFactor;
			if (abs(latVelInt32) > velLimit || abs(lonVelInt32) > velLimit)
			{
				latVelInt16 = latVelInt = INT16_MAX;
				lonVelInt16 = lonVelInt = INT16_MAX;
			}
			else
			{
				latVelInt16 = latVelInt = latVelInt32;
				lonVelInt16 = lonVelInt = lonVelInt32;
			}
			wholeTime = 0;
		}
		cout << "latVelInt, lonVelInt : " << latVelInt << " m/s, " << lonVelInt << " m/s" << endl;
		pastLat = curLat;
		pastLon = curLon;
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