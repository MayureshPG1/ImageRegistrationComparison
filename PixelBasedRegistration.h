#ifndef _PIXEL_BASED_REGISTRATION_
#define _PIXEL_BASED_REGISTRATION_

#include <opencv2/opencv.hpp>
#include "iostream"
#include "opencv2/reg/mapaffine.hpp"
#include "opencv2/reg/mapshift.hpp"
#include "opencv2/reg/mapprojec.hpp"
#include "opencv2/reg/mappergradshift.hpp"
#include "opencv2/reg/mappergradeuclid.hpp"
#include "opencv2/reg/mappergradsimilar.hpp"
#include "opencv2/reg/mappergradaffine.hpp"
#include "opencv2/reg/mappergradproj.hpp"
#include "opencv2/reg/mapperpyramid.hpp"
using namespace std;
using namespace cv;
using namespace cv::reg;

class PixelBasedRegistration
{
private:
	//images to be registered
	Mat img1, img2;
	bool usePyramid, saveOutput;
	string path;
	void CalcDifference(const Mat& image1, const Mat& image2, Mat& diffimg);
public:
	PixelBasedRegistration(Mat& img1, Mat& img2, bool usePyramid, bool saveOutput, string path);
	~PixelBasedRegistration();
	void ShiftRegistration();
	void AffineRegistration();
};

#endif // !_PIXEL_BASED_REGISTRATION_

