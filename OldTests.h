#ifndef _OLD_TESTS_
#define _OLD_TESTS_

#include <windows.h>
#include "Utils.h"
#include <iostream>
#include <fstream>
#include <vector>
#include "PixelBasedRegistration.h"
#include "FeatureBasedRegistration.h"
#include "PyramidBasedPixel.h"
#include "OpticalFlowAlignment.h"
using namespace cv;
using namespace std;

namespace oldtests
{
	//Simple struct to return from lsfiles
	struct List {
		vector<string> files;
		vector<string> folders;
	};

	void RemoveBlackBorder(Mat& alignedImg, const Mat img1);
	double GetPSNR(const Mat& I1, const Mat& I2);
	double GetPSNR(const Mat& I1, const Mat& I2, Rect roi);
	struct List lsfiles(string folder);
	int RunPSNRTest();
	int RunFeatureMatchScaleTest();
	int RunFeatureMatchTest();
	int RunPixelBasedTest();
	int RunCascadingPixelBasedTest();
	int RunPyramidPixelBasedTest();
	int RunOpticalFlowBasedAlignmentTest();
	void OverlayText(Mat& ip, string line1, string line2);
	int FinBigTest_AllAlgo();
	int MultiImage_FinBigTest_AllAlgo();
	int MultiImage_FinCorpus_NoPSNR();
	int MultiImage_FinCorpus_NoPSNR2();
	int MultiImage_FinCorpus_Scale();
}
#endif // !_OLD_TESTS_

