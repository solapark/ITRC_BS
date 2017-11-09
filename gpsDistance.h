#ifndef _GPSDISTANCE_H
#define _GPSDISTANCE_H

#define _USE_MATH_DEFINES
#include <cmath> 
#include <cstdint>
#include <iostream>
#define earthRadiusKm 6371.0

using namespace std;

class gpsDistance {
public:
	gpsDistance();
	gpsDistance(const int latPrcsn, const int lonPrcsn, const int velPrcsn);
	void getVelocity(const uint64_t timeMs, const int32_t lon1Int, const int32_t lat1Int, const int32_t lon2Int, const int32_t lat2Int, int16_t &latVelInt, int16_t &lonVelInt);

private:
	double latDiver;
	double lonDiver;
	int velFactor;
	int velFactorWs;

	double gpsDist(const double lat1d, const double lon1d, double const lat2d, double const lon2d);
	double deg2rad(const double deg);
	double rad2deg(const double rad);
};

#endif // !_GPSDISTANCE_H

