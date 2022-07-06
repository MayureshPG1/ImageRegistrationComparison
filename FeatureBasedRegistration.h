#ifndef _FEATURE_BASED_REGISTRATION_
#define _FEATURE_BASED_REGISTRATION_
#include "Utils.h"
#include <opencv2/opencv.hpp>
#include "iostream"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
using namespace std;
using namespace cv;
//using namespace cv::xfeatures2d;

enum FeatureType{ fORB, fKAZE, fAKAZE, fBRISK}; //fDAISY, fFREAKfBRIEF
enum MatchType{BASIC, RATIO_SYMMETRY, FILTER_SYMMETRY };

class FeatureBasedRegistration
{
private:
	//images to be registered
	Mat origImg1, origImg2, alignedImg;
	Mat im1Gray, im2Gray, scaleMat;
	Mat transform;
	//bool saveOutput;
	string path;
	float scalingFactor;

	const int MAX_FEATURES = 500;
	const float GOOD_MATCH_PERCENT = 0.15f;
	const float RATIO_THRESHOLD = 0.8f;
	const int FILTER_THRESHOLD = 3;

	void CalcDifference(const Mat& image1, const Mat& image2, Mat& diffimg);
	vector<DMatch> filter_distance(Mat descriptors, std::vector< DMatch > matches);
public:
	FeatureBasedRegistration(Mat& img1, Mat& img2, float scalingFactor = 1.0);//bool saveOutput,
	~FeatureBasedRegistration();
	//void CalcFeatureBasedReg(Mat& homographyMat, FeatureType fType);
	int CalcFeatureBasedRegNewMatch(FeatureType fType, MatchType mType);
	Mat& GetAlignedImage();
	Rect GetMaxRoi();
	//void CalcORBFeatureBasedReg();
	//void CalcBRISKFeatureBasedReg();
	//void CalcKAZEFeatureBasedReg();
	//void CalcAKAZEFeatureBasedReg();
};

#endif
