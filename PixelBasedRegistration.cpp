#include "PixelBasedRegistration.h"


void PixelBasedRegistration::CalcDifference(const Mat& image1, const Mat& image2, Mat& diffimg)
{
	Mat img1, img2;
	image1.convertTo(img1, CV_32FC3);
	image2.convertTo(img2, CV_32FC3);
	if (img1.channels() != 1)
		cvtColor(img1, img1, COLOR_BGR2GRAY);
	if (img2.channels() != 1)
		cvtColor(img2, img2, COLOR_BGR2GRAY);

	Mat imgDiff;
	img1.copyTo(imgDiff);
	imgDiff -= img2;
	imgDiff /= 2.f;
	imgDiff += 128.f;

	Mat imgSh;
	imgDiff.convertTo(imgSh, CV_8UC3);
}

PixelBasedRegistration::PixelBasedRegistration(Mat& img1, Mat& img2, bool usePyramid = true, bool saveOutput = true, string path = "")
{
	//constructor
	this->img1 = img1;
	this->img2 = img2;
	this->usePyramid = usePyramid;
	this->saveOutput = saveOutput;
	this->path = path;
}

PixelBasedRegistration::~PixelBasedRegistration()
{
	//destructor
}

void PixelBasedRegistration::ShiftRegistration()
{
	string imgName = "gradshift";
	// Register
	Ptr<MapperGradShift> mapper = makePtr<MapperGradShift>();
	Ptr<Map> mapPtr;

	if (this->usePyramid)
	{
		MapperPyramid mappPyr(mapper);
		mapPtr = mappPyr.calculate(img1, img2);
		imgName += "-pyramid";
	}
	else
	{
		mapPtr = mapper->calculate(img1, img2);
	}

	// Print result
	MapShift* mapShift = dynamic_cast<MapShift*>(mapPtr.get());
	cout << endl << "--- shift mapping ---" << endl;
	cout << Mat(mapShift->getShift()) << endl;

	// calc registration accuracy
	Mat destimg, diffimg;
	mapShift->inverseWarp(img2, destimg);
	CalcDifference(img1, destimg, diffimg);
	imwrite(path + imgName + "-reg.JPG", destimg);
	imwrite(path + imgName + "-diff.JPG", diffimg);
}

void PixelBasedRegistration::AffineRegistration()
{
	string imgName = "affine";
	// Register
	Ptr<MapperGradAffine> mapper = makePtr<MapperGradAffine>();
	Ptr<Map> mapPtr;

	if (this->usePyramid)
	{
		MapperPyramid mappPyr(mapper);
		mapPtr = mappPyr.calculate(img1, img2);
		imgName += "-pyramid";
	}
	else
	{
		mapPtr = mapper->calculate(img1, img2);
	}

	// Print result
	MapAffine* mapAff = dynamic_cast<MapAffine*>(mapPtr.get());
	cout << endl << "--- affine mapping ---" << endl;
	cout << Mat(mapAff->getLinTr()) << endl;
	cout << Mat(mapAff->getShift()) << endl;

	// calc registration accuracy
	Mat destimg, diffimg;
	mapAff->inverseWarp(img2, destimg);
	CalcDifference(img1, destimg, diffimg);
	imwrite(path + imgName + "-reg.JPG", destimg);
	imwrite(path + imgName + "-diff.JPG", diffimg);
}
