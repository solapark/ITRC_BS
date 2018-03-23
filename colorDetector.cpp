#include "stdafx.h"
#include "colorDetector.h"

colorDetector::colorDetector() {}

colorDetector::colorDetector(const float lowH, const float highH,
	const float lowS, const float highS,
	const float lowV, const float highV,
	const bool isRefine, bool isImgNormed)
	:lowH(lowH), highH(highH), lowS(lowS), highS(highS), lowV(lowV), highV(highV), isRefine(isRefine), isImgNormed(isImgNormed){}

void colorDetector::getThrImg(const Mat &imgOrg, Mat & imgThr) {
	imgOrg.copyTo(imgOriginal);

	if (!isImgNormed) {
		imgOriginal.convertTo(imgOriginal, CV_32FC3, 1 / 255.0);
		GaussianBlur(imgOriginal, imgOriginal, cv::Size(5, 5), 0.3);
	}

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
	//std::cout << "lowH : " << lowH << " lowS : " << lowS << " lowV : " << lowV << " highH : " << highH << " highS : " << highS << " highV : " << highV << std::endl;
	//imshow("Original", imgOrg); //show the original image
	//imshow("Thresholded Image", imgThr); //show the thresholded image
	//waitKey();
}

void colorDetector::showResult() {
	imshow("Original", imgOriginal); //show the original image
	imshow("Thresholded Image", imgThresholded); //show the thresholded image
	waitKey();
}
