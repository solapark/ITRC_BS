#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;

class colorDetector {
public:
	colorDetector();
	
	colorDetector(
		const float lowH, const float highH,
		const float lowS, const float highS,
		const float lowV, const float highV,
		const bool isRefine);
	
	void getThrImg(const Mat &imgOrg, Mat & mgThr);
	void showResult();

private:
	float lowH, highH, lowS, highS, lowV, highV;
	bool isRefine;
	Mat imgOriginal, imgHSV, imgThresholded;
};