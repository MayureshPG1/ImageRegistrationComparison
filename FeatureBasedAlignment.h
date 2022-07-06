#ifndef _FEATURE_BASED_REGISTRATION_
#define _FEATURE_BASED_REGISTRATION_
#include <opencv2/opencv.hpp>
#include "iostream"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include "CommonData.h"
using namespace std;
using namespace cv;

enum FeatureType { fORB, fKAZE, fAKAZE, fBRISK }; //fDAISY, fFREAKfBRIEF
enum MatchType { BASIC, RATIO_SYMMETRY, FILTER_SYMMETRY };

class FeatureBasedAlignment
{
private:
	//images to be registered
	Mat origImg1, origImg2, alignedImg;
	Mat im1Gray, im2Gray, scaleMat;
	Mat transformationMat;
	//string path;
	ScalingFactor scalingFactor;

	const int MAX_FEATURES = 500;
	const float GOOD_MATCH_PERCENT = 0.15f;
	const float RATIO_THRESHOLD = 0.8f;
	const int FILTER_THRESHOLD = 3;

	vector<DMatch> filter_distance(Mat descriptors, std::vector< DMatch > matches);
public:
	FeatureBasedAlignment(Mat& img1, Mat& img2, ScalingFactor scalingFactor = ScalingFactor::FULL);//bool saveOutput,
	~FeatureBasedAlignment();
	int RunFeatureBasedAlignment(FeatureType fType, MatchType mType);
	Mat& GetAlignedImage();
	Rect GetMaxRoi();
};

#endif
