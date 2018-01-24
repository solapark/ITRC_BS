#pragma once
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class gpsTable {
public:
	gpsTable() {}

	gpsTable(const char* gpsFile, const int tableRow, const int tableCol);

	void setGpsTable(const char* gpsFile, const int tableRow, const int tableCol);

	int getGps(const Point2i& pixel, Point2d &gps) const;
	int getGps64INT(const Point2i& pixel, Point2l &gps) const;

private:
	Point2d* table;
	int tableRow;
	int tableCol;
};