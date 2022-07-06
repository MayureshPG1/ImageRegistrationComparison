#ifndef _TESTS_
#define _TESTS_

#include <windows.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "FeatureBasedAlignment.h"
#include "PyramidBasedPixelAlignment.h"
#include "OpticalFlowAlignment.h"
using namespace cv;
using namespace std;

namespace TestRuns
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
	int MultiImage_FinCorpus_MultiBlend();
	int GetMaxImage(Mat& bufferImg, Mat ipImg);
	int MultiImage_FinCorpus_MultiMaxBlend();
}
#endif // !_OLD_TESTS_

