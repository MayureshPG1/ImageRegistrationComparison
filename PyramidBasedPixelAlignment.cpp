/* This class implements pixel based alignment using pyramid logic*/
#include "PyramidBasedPixelAlignment.h"

PyramidBasedPixelAlignment::PyramidBasedPixelAlignment(Mat img1, Mat img2 /*, int numLevels = 4*/)
{
	//if (numLevels > 4)
	//	numLevels = 4;//restruction due to maxiter number

	this->numLevels = 4;//hardcode for now for having better control over exit criteria
	this->origImg2 = img2;
	this->origImg1 = img1;

	//convert to grayscale
	Mat im1Gray, im2Gray;
	cvtColor(origImg1, im1Gray, CV_BGR2GRAY);
	cvtColor(origImg2, im2Gray, CV_BGR2GRAY);
	
	int maxIter = 8;
	float maxEPS = 0.001f;

	exitCriteria.push_back(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 2, 0.001));
	exitCriteria.push_back(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 4, 0.01));
	exitCriteria.push_back(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 6, 0.1));
	exitCriteria.push_back(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 8, 0.1));

	//build pyramid
	for (int i = 0; i < numLevels; i++)
	{
		Mat tmp1,tmp2;
		//TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, maxIter - 2 * i, maxEPS * pow(10,i));
		//exitCriteria.push_back(criteria);

		if (i == 0)
		{
			img1Pyr.push_back(im1Gray.clone());
			img2Pyr.push_back(im2Gray.clone());
			//pyrDown(im1Gray, tmp1);
			//pyrDown(im2Gray, tmp2);
			//img1Pyr.push_back(tmp1.clone());
			//img2Pyr.push_back(tmp2.clone());
		}
		else
		{
			pyrDown(img1Pyr[i - 1], tmp1);
			pyrDown(img2Pyr[i - 1], tmp2);
			img1Pyr.push_back(tmp1.clone());
			img2Pyr.push_back(tmp2.clone());
		}
	}

	//at higher resolutions do less iterations at eps accuracy
	//std::reverse(exitCriteria.begin(), exitCriteria.end());
}

PyramidBasedPixelAlignment::~PyramidBasedPixelAlignment()
{
	exitCriteria.clear();
	img1Pyr.clear();
	img2Pyr.clear();
}

Rect PyramidBasedPixelAlignment::GetMaxRoi()
{
	Mat frame = this->GetAlignedImage();
	Mat tf = this->transformationMat;
	const double width = static_cast<double>(frame.cols);
	const double height = static_cast<double>(frame.rows);
	const double aspect_ratio = width / height;

	//-- Get the corners from the image
	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0, 0);//top left 
	obj_corners[1] = cvPoint(frame.cols, 0);//top right
	obj_corners[2] = cvPoint(frame.cols, frame.rows);//bottom right 
	obj_corners[3] = cvPoint(0, frame.rows);//bottom left
	std::vector<Point2f> scene_corners(4);

	if(this->tfType == MotionModel::HOMOGRAPHY)
		perspectiveTransform(obj_corners, scene_corners, tf);
	else
		transform(obj_corners, scene_corners, tf);

	double top_content, bottom_content, right_content, left_content;

	top_content = max(scene_corners[0].y, scene_corners[1].y);
	bottom_content = min(scene_corners[2].y, scene_corners[3].y);
	left_content = max(scene_corners[0].x, scene_corners[3].x);
	right_content = min(scene_corners[1].x, scene_corners[2].x);

	// Respect aspect ratio.
	const double width_content = right_content - left_content;
	const double height_content = bottom_content - top_content;
	const double center_x = left_content + (width_content / 2.0);
	const double center_y = top_content + (height_content / 2.0);
	double width_cropped, height_cropped;
	if (height_content * aspect_ratio > width_content) {
		// Width is limiting.
		width_cropped = width_content;
		height_cropped = width_content / aspect_ratio;
	}
	else {
		// Height is limiting.
		height_cropped = height_content;
		width_cropped = height_content * aspect_ratio;
	}

	Rect roi(center_x - (width_cropped / 2.0), center_y - (height_cropped / 2.0), width_cropped, height_cropped);

	if (roi.x < 0)
		roi.x = 0;

	if ((roi.x + roi.width) >= frame.cols)
		roi.width = frame.cols - roi.x;

	if (roi.y < 0)
		roi.y = 0;

	if ((roi.y + roi.height) >= frame.rows)
		roi.height = frame.rows - roi.y;

	if ((roi.width < 0) || (roi.height < 0))
	{
		cout << "WARPING FAILED!!" << endl;
		roi.x = roi.y = 0;
		roi.width = frame.cols;
		roi.height = frame.rows;
	}

	return roi;
}

int PyramidBasedPixelAlignment::RunPyramidBasedPixelAlignment(MotionModel model = MotionModel::AFFINE)
{
	int retVal = 0;
	this->tfType = model;
	if (model == MotionModel::HOMOGRAPHY)
	{
		transformationMat = Mat::eye(3, 3, CV_32F);
		scaleMat = (Mat_<float>(3, 3) << 1.0, 1.0, 2.0, 1.0, 1.0, 2.0, 0.5, 0.5, 1.0);
	}
	else
	{
		transformationMat = Mat::eye(2, 3, CV_32F);
		scaleMat = (Mat_<float>(2, 3) << 1.0, 1.0, 2.0, 1.0, 1.0, 2.0);
	}

	for (int i = (numLevels - 1); i >= 0; i--)
	{
		try
		{
			findTransformECC(img1Pyr[i], img2Pyr[i], transformationMat, model, exitCriteria[i]);
		}
		catch (exception e)
		{
			cout << "FAIL-algorithm could not converge" << endl;
			retVal = -1;
			return retVal;
		}

		//cout << img1Pyr[i].size() << endl;
		//cout << transformationMat << endl;

		if (i != 0)
		{
			transformationMat = transformationMat.mul(scaleMat);
		}
	}
	
	if (model == MotionModel::HOMOGRAPHY)
	{
		warpPerspective(origImg2, this->alignedImg, transformationMat, origImg2.size(), INTER_LINEAR + WARP_INVERSE_MAP);
	}
	else
	{
		warpAffine(origImg2, this->alignedImg, transformationMat, origImg2.size(), INTER_LINEAR + WARP_INVERSE_MAP);
	}
	return retVal;
}

Mat& PyramidBasedPixelAlignment::GetAlignedImage()
{
	return this->alignedImg;
}
