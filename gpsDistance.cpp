#include "stdafx.h"
#include "gpsDistance.h"
gpsDistance::gpsDistance(){}

gpsDistance::gpsDistance(const int latPrcsn, const int lonPrcsn, const int velPrcsn)
{
	latDiver = pow(10, latPrcsn);
	lonDiver = pow(10, lonPrcsn);
	velFactor = pow(10, velPrcsn);
	//velFactorWs = velFactor / 1000;		//1000 = ms to s
	//cout << "latDiver : " << latDiver << endl;
	//cout << "lonDiver : " << lonDiver << endl;
	//cout << "velFoctor: " << velFactor << endl;
	//cout << "velFactorWs : " << velFactorWs << endl;

}

void gpsDistance::getVelocity(const uint64_t timeMs, const int32_t lon1Int, const int32_t lat1Int, const int32_t lon2Int, const int32_t lat2Int, int16_t &latVelInt, int16_t &lonVelInt) {
	double latDist = gpsDist(lat1Int/ latDiver, 0, lat2Int/ latDiver, 0);
	double lonDist = gpsDist(0,lon1Int/ lonDiver, 0,lon2Int / lonDiver);
//	cout << "latDist , lonDist : " << latDist << "m, " << lonDist << "m"<< endl;
	latVelInt = latDist / (timeMs /1000) * velFactor;
	lonVelInt = lonDist / (timeMs / 1000) * velFactor;

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
double gpsDistance::gpsDist(const double lat1d, const double lon1d, double const lat2d, double const lon2d) {
	double lat1r, lon1r, lat2r, lon2r, u, v;
	lat1r = deg2rad(lat1d);
	lon1r = deg2rad(lon1d);
	lat2r = deg2rad(lat2d);
	lon2r = deg2rad(lon2d);
	u = sin((lat2r - lat1r) / 2);
	v = sin((lon2r - lon1r) / 2);
	return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v)) * 1000;
}

double gpsDistance::deg2rad(const double deg) {

	return (deg * M_PI / 180);
}

//  This function converts radians to decimal degrees
double gpsDistance::rad2deg(const double rad) {
	return (rad * 180 / M_PI);
}