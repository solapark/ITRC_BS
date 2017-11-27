#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;

class colorDetector {
public:
	colorDetector();
	
	colorDetector(
		const int lowH, const int highH, 
		const int lowS, const int highS, 
		const int lowV, const int highV, 
		const bool isRefine);
	
	void getThrImg(const Mat &imgOrg, Mat & mgThr);
	void showResult();

private:
	int lowH, highH, lowS, highS, lowV, highV;
	bool isRefine;
	Mat imgOriginal, imgHSV, imgThresholded;
};