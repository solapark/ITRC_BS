#ifndef _GPSVELOCITY_H
#define _GPSVELOCITY_H

#define _USE_MATH_DEFINES
#include <cmath> 
#include <cstdint>
#include <iostream>
#include <cassert>
#define earthRadiusKm 6371.0

using namespace std;

class gpsVelocity {
public:
	gpsVelocity();
	gpsVelocity(const int latPrcsn, const int lonPrcsn, const int velPrcsn);
	void setPrecision(const int latPrcsn, const int lonPrcsn, const int velPrcsn);
	void resetIsFirstMoment();
	void getVelocity(const uint64_t timeMs, const int32_t lat2Int, const int32_t lon2Int, int16_t &latVelInt, int16_t &lonVelInt);

private:
	bool isFirstMoment;
	double pastLat, pastLon, curLat, curLon;
	double latDivider, lonDivider;
	int velFactor;
	int velFactorWs;

	double gpsDist(const double lat1d, const double lon1d, double const lat2d, double const lon2d);
	double deg2rad(const double deg);
	double rad2deg(const double rad);
};

#endif // !_GPSVELOCITY_H

