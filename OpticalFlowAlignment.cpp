/* This class implements optical flow based alignment methods */
#include "OpticalFlowAlignment.h"

OpticalFlowAlignment::OpticalFlowAlignment(Mat img1, Mat img2, ScalingFactor scalingFactor)
{
	this->origImg1 = img1;
	this->origImg2 = img2;

	this->scalingFactor = scalingFactor;
	// Convert images to grayscale
	cvtColor(origImg1, im1Gray, CV_BGR2GRAY);
	cvtColor(origImg2, im2Gray, CV_BGR2GRAY);

	//use pyramid scaling
	Mat tmp1;
	if (scalingFactor == ScalingFactor::HALF)
	{
		pyrDown(im1Gray, tmp1);
		im1Gray = tmp1.clone();

		pyrDown(im2Gray, tmp1);
		im2Gray = tmp1.clone();
		scaleMat = (Mat_<double>(3, 3) << 1.0, 1.0, 2.0, 1.0, 1.0, 2.0, 0.5, 0.5, 1.0);
	}
	else if (scalingFactor == ScalingFactor::QUARTER)
	{
		pyrDown(im1Gray, tmp1);
		pyrDown(tmp1, im1Gray);

		pyrDown(im2Gray, tmp1);
		pyrDown(tmp1, im2Gray);
		scaleMat = (Mat_<double>(3, 3) << 1.0, 1.0, 4.0, 1.0, 1.0, 4.0, 0.25, 0.25, 1.0);
	}
}

OpticalFlowAlignment::~OpticalFlowAlignment()
{
}

Rect OpticalFlowAlignment::GetMaxRoi()
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

	perspectiveTransform(obj_corners, scene_corners, tf);
	//perspectiveTransform(obj_corners, scene_corners, tf);
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

Mat& OpticalFlowAlignment::GetAlignedImage()
{
	return this->alignedImg;
}

int OpticalFlowAlignment::RunOpticalFlowAlignment_OPT(bool useFWBW, bool useORB, bool useEigenTest)
{
	int retVal = 0;

	vector <Point2f> prevFeatures, nextFeatures, prevBackFeatures;
	vector <Point2f> goodFeatures1, goodFeatures2;
	vector <uchar> status1,status2;
	vector <float> err1,err2;

	Ptr<Feature2D> featDescriptor;
	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;
	vector<int>& keypointIndexes = vector<int>();

	if (!useORB)
	{
		//Estimating the features in frame1
		goodFeaturesToTrack(im1Gray, prevFeatures, 200, 0.01, 30);
	}
	else
	{
		//use ORB instead	
		featDescriptor = ORB::create(200);
		featDescriptor->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
		KeyPoint::convert(keypoints1, prevFeatures, keypointIndexes);
	}

	//forward calc
	if (!useEigenTest)
	{
		calcOpticalFlowPyrLK(im1Gray, im2Gray, prevFeatures, nextFeatures, status1, err1);
	}
	else
	{
		calcOpticalFlowPyrLK(im1Gray, im2Gray, prevFeatures, nextFeatures, status1, err1, Size(21, 21), 3,
			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01),
			OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4); /*1e-1*///make aggresive criteria to check an impact
	}

	if (!useFWBW)
	{
		for (size_t i = 0; i < status1.size(); i++)
		{
			if (status1[i])
			{
				goodFeatures1.push_back(prevFeatures[i]);
				goodFeatures2.push_back(nextFeatures[i]);
			}
		}
	}
	else
	{
		//backward calc
		calcOpticalFlowPyrLK(im2Gray, im1Gray, nextFeatures, prevBackFeatures, status2, err2);

		for (int i = 0; i < prevFeatures.size(); i++)
		{
			if (status1[i] && status2[i])
			{
				goodFeatures1.push_back(prevFeatures[i]);
				goodFeatures2.push_back(nextFeatures[i]);
			}
		}
	}

	if ((goodFeatures1.size() <= 4) || (goodFeatures2.size() <= 4))
	{
		cout << "FAIL--OpticalFlow too few points to find transform" << endl;
		retVal = -1;
		return retVal;
	}
	/***homography alignment**/
	transformationMat = findHomography(goodFeatures2, goodFeatures1, RANSAC);

	if (transformationMat.empty())
	{
		cout << "FAIL--OpticalFlow could not find transform" << endl;
		retVal = -1;
		return retVal;
	}

	// if scaling is applied rescale homography mat
	if ((scalingFactor == ScalingFactor::HALF) || (scalingFactor == ScalingFactor::QUARTER))
	{
		transformationMat = transformationMat.mul(scaleMat);
	}

	Mat img2Reg;
	// Use homography to warp image
	// apply calculated matrix on original image not on the scaled image
	warpPerspective(origImg2, img2Reg, transformationMat, origImg2.size());

	this->alignedImg = img2Reg.clone();

	return retVal;
}

//double symmetry algorithm
//the idea for double symmetry is this
//1.match orb features in both images independently
//apply ratio symmetry filtering on them to get best ORB features
//this will fetch best ORB without any spatial constraint
//2. now get this best ORB and apply FWBW filtering with optical flow on them
// this will apply a hard spatial constraints on features
// idea is that with this double filtering only the best of best features will remain and
// this will result in homography not being skewed in failure cases
/******DOES NOT WORK AT ALL********/
int OpticalFlowAlignment::RunOpticalFlowAlignment_DS()
{
	int retVal = 0;

	vector <Point2f> nextFeatures, prevBackFeatures;
	vector <Point2f> goodFeatures1, goodFeatures2;
	vector <uchar> status1, status2;
	vector <float> err1, err2;

	Ptr<Feature2D> featDescriptor;
	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;

	/**1. ORB symmetry**/
	//use ORB
	featDescriptor = ORB::create(200);
	
	//compute orb features in image 1 and 2
	featDescriptor->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
	featDescriptor->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);

	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	vector< vector<DMatch> > matches12, matches21;
	matcher->knnMatch(descriptors1, descriptors2, matches12, 2);
	matcher->knnMatch(descriptors2, descriptors1, matches21, 2);

	std::vector<DMatch> good_matches1, good_matches2;
	std::vector<DMatch> better_matches;
	const float RATIO_THRESHOLD = 0.8f;
	// ratio test proposed by David Lowe paper
	for (int i = 0; i < matches12.size(); i++) {
		if (matches12[i][0].distance < RATIO_THRESHOLD * matches12[i][1].distance)
			good_matches1.push_back(matches12[i][0]);
	}

	for (int i = 0; i < matches21.size(); i++) {
		if (matches21[i][0].distance < RATIO_THRESHOLD * matches21[i][1].distance)
			good_matches2.push_back(matches21[i][0]);
	}

	// Symmetric Test
	for (int i = 0; i<good_matches1.size(); i++) {
		for (int j = 0; j<good_matches2.size(); j++) {
			if (good_matches1[i].queryIdx == good_matches2[j].trainIdx && good_matches2[j].queryIdx == good_matches1[i].trainIdx) {
				better_matches.push_back(DMatch(good_matches1[i].queryIdx, good_matches1[i].trainIdx, good_matches1[i].distance));
				break;
			}
		}
	}


	if (better_matches.size() <= 4)
	{
		retVal = -1;
		cout << "TOO Less points at ORB ratio symmetry !!!FAIL!!!" << endl;
		return retVal;
	}

	// Extract location of good matches
	std::vector<Point2f> points1, points2;

	for (size_t i = 0; i < better_matches.size(); i++)
	{
		points1.push_back(keypoints1[better_matches[i].queryIdx].pt);
		points2.push_back(keypoints2[better_matches[i].trainIdx].pt);
	}

	/**2. optical flow symmetry**/
	calcOpticalFlowPyrLK(im1Gray, im2Gray, points1, nextFeatures, status1, err1);

	//backward calc
	//do not use nextFeatures as an inital guess calc this optical flow independently
	//there is a correspondence btw points1 and points2
	calcOpticalFlowPyrLK(im2Gray, im1Gray, points2, prevBackFeatures, status2, err2);

	for (int i = 0; i < nextFeatures.size(); i++)
	{
		if (status1[i] && status2[i])
		{
			goodFeatures1.push_back(nextFeatures[i]);
			goodFeatures2.push_back(prevBackFeatures[i]);
		}
	}

	if ((goodFeatures1.size() <= 4) || (goodFeatures2.size() <= 4))
	{
		cout << "FAIL--OpticalFlow too few points to find transform" << endl;
		retVal = -1;
		return retVal;
	}
	/***homography alignment**/
	transformationMat = findHomography(goodFeatures2, goodFeatures1, RANSAC);

	if (transformationMat.empty())
	{
		cout << "FAIL--OpticalFlow could not find transform" << endl;
		retVal = -1;
		return retVal;
	}

	// if scaling is applied rescale homography mat
	if ((scalingFactor == ScalingFactor::HALF) || (scalingFactor == ScalingFactor::QUARTER))
	{
		transformationMat = transformationMat.mul(scaleMat);
	}

	Mat img2Reg;
	// Use homography to warp image
	// apply calculated matrix on original image not on the scaled image
	warpPerspective(origImg2, img2Reg, transformationMat, origImg2.size());

	this->alignedImg = img2Reg.clone();

	return retVal;
}

int OpticalFlowAlignment::RunOpticalFlowAlignment()
{
	int retVal = 0;

	vector <Point2f> features1, features2;
	vector <Point2f> goodFeatures1, goodFeatures2;
	vector <uchar> status;
	vector <float> err;

	//Estimating the features in frame1 and frame2
	goodFeaturesToTrack(im1Gray, features1, 200, 0.01, 30);
	calcOpticalFlowPyrLK(im1Gray, im2Gray, features1, features2, status, err);

	for (size_t i = 0; i < status.size(); i++)
	{
		if (status[i])
		{
			goodFeatures1.push_back(features1[i]);
			goodFeatures2.push_back(features2[i]);
		}
	}

	if ((goodFeatures1.size() <= 4) || (goodFeatures2.size() <= 4))
	{
		cout << "FAIL--OpticalFlow too few points to find transform" << endl;
		retVal = -1;
		return retVal;
	}
	/***homography alignment**/
	transformationMat = findHomography(goodFeatures2, goodFeatures1, RANSAC);

	// if scaling is applied rescale homography mat
	if ((scalingFactor == 0.5) || (scalingFactor == 0.25))
	{
		transformationMat = transformationMat.mul(scaleMat);
	}
	if (transformationMat.empty())
	{
		cout << "FAIL--OpticalFlow could not find transform" << endl;
		retVal = -1;
		return retVal;
	}
	//this->transformationMat = transformationMat;
	Mat img2Reg;

	// Use homography to warp image
	// apply calculated matrix on original image not on the scaled image
	warpPerspective(origImg2, img2Reg, transformationMat, origImg2.size());

	this->alignedImg = img2Reg.clone();

	/**affine alignment**/
	////All the parameters scale, angle, and translation are stored in affine
	//transformationMat = estimateRigidTransform(goodFeatures1, goodFeatures2, true);

	//if (transformationMat.empty())
	//{
	//	cout << "FAIL--OpticalFlow could not find transform" << endl;
	//	retVal = -1;
	//	return retVal;
	//}
	////Warp the new frame using the smoothed parameters
	//warpAffine(origImg2, this->alignedImg, transformationMat, origImg2.size(), INTER_LINEAR + WARP_INVERSE_MAP);

	return retVal;
}