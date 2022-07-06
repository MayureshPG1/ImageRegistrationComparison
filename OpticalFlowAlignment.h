#ifndef _OPTICAL_FLOW_ALIGNMENT_
#define _OPTICAL_FLOW_ALIGNMENT_
#include <opencv2/opencv.hpp>
#include "iostream"
#include "CommonData.h"

using namespace std;
using namespace cv;
class OpticalFlowAlignment
{
public:
	OpticalFlowAlignment(Mat img1, Mat img2, ScalingFactor scalingFactor = ScalingFactor::FULL);
	~OpticalFlowAlignment();
	Mat& GetAlignedImage();
	int RunOpticalFlowAlignment();
	int RunOpticalFlowAlignment_OPT(bool useFWBW, bool useORB, bool useEigenTest);
	int RunOpticalFlowAlignment_DS();
	Rect GetMaxRoi();
private:
	Mat origImg1, origImg2, alignedImg;
	Mat im1Gray, im2Gray, scaleMat;
	Mat transformationMat;
	ScalingFactor scalingFactor;
};

#endif