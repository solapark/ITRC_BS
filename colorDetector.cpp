#include "stdafx.h"
#include "colorDetector.h"

colorDetector::colorDetector() {}

colorDetector::colorDetector(const int lowH, const int highH, const int lowS, const int highS, const int lowV, const int highV, const bool isRefine)
	:lowH(lowH), highH(highH), lowS(lowS), highS(highS), lowV(lowV), highV(highV), isRefine(isRefine) {}

void colorDetector::getThrImg(const Mat &imgOrg, Mat & imgThr) {
	imgOrg.copyTo(imgOriginal);
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	inRange(imgHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), imgThr); //Threshold the image

	if (isRefine) {
		//morphological opening (remove small objects from the foreground)
		erode(imgThr, imgThr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThr, imgThr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThr, imgThr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThr, imgThr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	}

}

void colorDetector::showResult() {
	imshow("Original", imgOriginal); //show the original image
	imshow("Thresholded Image", imgThresholded); //show the thresholded image
	waitKey();
}
