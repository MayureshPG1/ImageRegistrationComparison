#ifndef _PYRAMID_BASED_PIXEL_
#define _PYRAMID_BASED_PIXEL_
#include <opencv2/opencv.hpp>
#include "iostream"
#include "CommonData.h"
using namespace std;
using namespace cv;

//opencv for below enum name is not defined, so it's same thing with name
enum MotionModel
{
	TRANSLATION = 0,
	EUCLIDEAN = 1,
	AFFINE = 2,
	HOMOGRAPHY = 3
};

class PyramidBasedPixelAlignment
{
private:
	vector<Mat> img1Pyr, img2Pyr;
	vector<TermCriteria> exitCriteria;
	int numLevels;
	Mat transformationMat;
	Mat scaleMat;
	Mat origImg1, origImg2, alignedImg;
	MotionModel tfType;
public:
	PyramidBasedPixelAlignment(Mat img1, Mat img2/*,int numLevels*/);
	~PyramidBasedPixelAlignment();
	int RunPyramidBasedPixelAlignment(MotionModel model);
	Mat& GetAlignedImage();
	Rect GetMaxRoi();
};
#endif // !_PYRAMID_BASED_PIXEL_

