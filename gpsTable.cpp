#include "stdafx.h"
#include "gpsTable.h"

gpsTable::gpsTable(const char* gpsFile, const int tableRow, const int tableCol) : tableRow(tableRow), tableCol(tableCol)
{
	FILE* pFile = fopen(gpsFile, "r");
	if (pFile == NULL) {
		cout << "gpsFile can't be opened" << endl;
		return;
	}

	table = new Point2d[tableRow * tableCol];
	int x, y;
	Point2i pixel; //x = col, y = row
	Point2d gps; //x = lat, y = lon

	long count = 0;
	while (!feof(pFile))
	{
		//cout << ++count << endl;
		fscanf(pFile, "%d\t%d\t%lf\t%lf\n", &pixel.x, &pixel.y, &gps.x, &gps.y);
		//printf("공백으로 분리 : %d %d %.7f %.7f\n", pixel.x, pixel.y, gps.x, gps.y);
		//table[80*360+44] = gps;
		table[(pixel.y - 1)*tableCol + (pixel.x - 1)] = gps;
		//table[pixel.x*tableCol + pixel.y] = gps;
	}

	fclose(pFile);
}

void gpsTable::setGpsTable(const char* gpsFile, const int tableRow, const int tableCol) {
	this->tableRow = tableRow;
	this->tableCol = tableCol;

	FILE* pFile = fopen(gpsFile, "r");
	if (pFile == NULL) {
		cout << "gpsFile can't be opened" << endl;
		return;
	}

	table = new Point2d[tableRow * tableCol];
	int x, y;
	Point2i pixel; //x = col, y = row
	Point2d gps; //x = lat, y = lon

	long count = 0;
	while (!feof(pFile))
	{
		//cout << ++count << endl;
		fscanf(pFile, "%d\t%d\t%lf\t%lf\n", &pixel.x, &pixel.y, &gps.x, &gps.y);
//		cout.precision(10);
//		cout<<"공백으로 분리 : " << pixel.x << " " << pixel.y << " " << gps.x << " " << gps.y<<endl;
		table[(pixel.y - 1)*tableCol + (pixel.x - 1)] = gps;
		//table[pixel.x*tableCol + pixel.y] = gps;
	}

	fclose(pFile);
}

int gpsTable::getGps(const Point2i& pixel, Point2d &gps) const {
	Point2i targetPixel = pixel;
	gps = table[targetPixel.y * tableCol + targetPixel.x];
	//cout.precision(10);
	//cout << "pixel : " << pixel << " gps: " << gps << endl;
	if (gps.x == 0 && gps.y == 0) {
		// no data
		return -1;
	}
	return 1;
}

int gpsTable::getGps64INT(const Point2i& pixel, Point2l &gps) const {
	Point2d gpsD;
	getGps(pixel, gpsD);
	gps.x = gpsD.x * (double)10000000;
	gps.y = gpsD.y * (double)10000000;
	return 1;
}

