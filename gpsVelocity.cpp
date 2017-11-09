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

void gpsVelocity::setPrecision(const int latPrcsn, const int lonPrcsn, const int velPrcsn) {
	latDivider = pow(10, latPrcsn);
	lonDivider = pow(10, lonPrcsn);
	velFactor = pow(10, velPrcsn);
	//cout << "latDiver : " << latDivider << endl;
	//cout << "lonDiver : " << lonDivider << endl;
	//cout << "velFoctor: " << velFactor << endl;
}

void gpsVelocity::resetIsFirstMoment()
{
	isFirstMoment = true;
}

void gpsVelocity::getVelocity(const uint64_t timeMs, const int32_t curLatInt, const int32_t curLonInt, int16_t &latVelInt, int16_t &lonVelInt) {
	if (isFirstMoment) {
//		cout << "isFirstMoment "<< endl;
		latVelInt = 0;
		lonVelInt = 0;
		pastLat = curLatInt / latDivider;
		pastLon = curLonInt / lonDivider;
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
		//latVelInt = latDist / (timeMs / 1000) * velFactor;
		//lonVelInt = lonDist / (timeMs / 1000) * velFactor;
		cout << "latVelInt, latVelInt: " << latDist / abs(((double)30 / 1000) * velFactor) << " m/s, " << abs(((double)30 / 1000) * velFactor) << " m/s" << endl;
		//assert(abs(latDist / ((double)30 / 1000) * velFactor) < INT16_MAX && abs(lonDist / ((double)30 / 1000) * velFactor) < INT16_MAX && "velocity overflow");
		if (abs(latDist / ((double)30 / 1000) * velFactor) > INT16_MAX || abs(lonDist / ((double)30 / 1000) * velFactor) > INT16_MAX)
		{
			latVelInt = INT16_MAX;
			lonVelInt = INT16_MAX;
		}
		else
		{
			latVelInt = latDist / ((double)30 / 1000) * velFactor;
			lonVelInt = lonDist / ((double)30 / 1000) * velFactor;
		}
		//cout << "latVelInt, lonDist : " << latVelInt << " 10^-2 m/s, " << lonVelInt << " 10^-2 m/s" << endl;
		cout << "latVelInt, lonDist : " << latVelInt << " m/s, " << lonVelInt << " m/s" << endl;
		pastLat = curLat;
		pastLon = curLon;
	}

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