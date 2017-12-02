#include "stdafx.h"
#include "colorDetector.h"

colorDetector::colorDetector() {}

colorDetector::colorDetector(const float lowH, const float highH,
	const float lowS, const float highS,
	const float lowV, const float highV,
	const bool isRefine)
	:lowH(lowH), highH(highH), lowS(lowS), highS(highS), lowV(lowV), highV(highV), isRefine(isRefine) {}

void colorDetector::getThrImg(const Mat &imgOrg, Mat & imgThr) {
	imgOrg.copyTo(imgOriginal);
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	//std::vector<Mat> hsvChannels;
	//split(imgHSV, hsvChannels);
	//double minHue, maxHue, minSat, maxSat, minVal, maxVal;
	//cv::minMaxIdx(hsvChannels[0], &minHue, &maxHue, 0, 0);
	//cv::minMaxIdx(hsvChannels[1], &minSat, &maxSat, 0, 0);
	//cv::minMaxIdx(hsvChannels[2], &minVal, &maxVal, 0, 0);
	//std::cout <<"minHue : "<< minHue << " maxHue : "<<maxHue << " minSat : " << minSat << " maxSat : " << maxSat << " minVal : " << minVal << " maxVal : " << maxVal << std::endl;
	//std::cout<<imgHSV.type()<< std::endl;
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
