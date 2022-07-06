#include "FeatureBasedRegistration.h"

using namespace std;
using namespace cv;
void FeatureBasedRegistration::CalcDifference(const Mat& image1, const Mat& image2, Mat& diffimg)
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

	diffimg = imgSh.clone();
}

Rect FeatureBasedRegistration::GetMaxRoi()
{
	Mat frame = this->GetAlignedImage();
	Mat tf = this->transform;
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

FeatureBasedRegistration::FeatureBasedRegistration(Mat& img1, Mat& img2, float scalingFactor)// bool saveOutput = false, 
{
	//constructor
	this->origImg1 = img1;
	this->origImg2 = img2;
	//this->saveOutput = saveOutput;
	//this->path = path;
	this->scalingFactor = scalingFactor;
	// Convert images to grayscale
	cvtColor(origImg1, im1Gray, CV_BGR2GRAY);
	cvtColor(origImg2, im2Gray, CV_BGR2GRAY);

	// apply scaling
	//resize(im1Gray, im1Gray, Size(im1Gray.cols*this->scalingFactor, im1Gray.rows*this->scalingFactor));
	//resize(im2Gray, im2Gray, Size(im2Gray.cols*this->scalingFactor, im2Gray.rows*this->scalingFactor));

	//use pyramid scaling
	Mat tmp1;
	if (scalingFactor == 0.5)
	{
		pyrDown(im1Gray, tmp1);
		im1Gray = tmp1.clone();

		pyrDown(im2Gray, tmp1);
		im2Gray = tmp1.clone();
		scaleMat = (Mat_<double>(3, 3) << 1.0, 1.0, 2.0, 1.0, 1.0, 2.0, 0.5, 0.5, 1.0);
	}
	else if (scalingFactor == 0.25)
	{
		pyrDown(im1Gray, tmp1);
		pyrDown(tmp1, im1Gray);

		pyrDown(im2Gray, tmp1);
		pyrDown(tmp1, im2Gray);
		scaleMat = (Mat_<double>(3, 3) << 1.0, 1.0, 4.0, 1.0, 1.0, 4.0, 0.25, 0.25, 1.0);
	}

}

FeatureBasedRegistration::~FeatureBasedRegistration()
{

}

vector<DMatch> FeatureBasedRegistration::filter_distance(Mat descriptors, vector< DMatch > matches)
{
	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors.rows; i++) {
		double dist = matches[i].distance;
		if (dist < min_dist)
			min_dist = dist;
		if (dist > max_dist)
			max_dist = dist;
	}

	//printf("-- Max dist : %f \n", max_dist);
	//printf("-- Min dist : %f \n", min_dist);

	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.
	std::vector< DMatch > good_matches;

	for (int i = 0; i < descriptors.rows; i++)
	{
		if (matches[i].distance <= max(FILTER_THRESHOLD * min_dist, 0.02))
		{
			good_matches.push_back(matches[i]);
		}
	}
	return good_matches;
}

int FeatureBasedRegistration::CalcFeatureBasedRegNewMatch(FeatureType fType = FeatureType::fORB, MatchType mType = MatchType::BASIC)
{
	//string imgName;
	Mat homographyMat;
	int retVal = 0;

	// Variables to store keypoints and descriptors
	std::vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;

	// Detect features and compute descriptors.
	Ptr<Feature2D> featDescriptor;
	switch (fType)
	{
	case FeatureType::fORB:
		//imgName = "ORBFeat";
		featDescriptor = ORB::create(MAX_FEATURES);
		break;
	case FeatureType::fKAZE:
		//imgName = "KAZEFeat";
		featDescriptor = KAZE::create();
		break;
	case FeatureType::fBRISK:
		//imgName = "BRISKFeat";
		featDescriptor = BRISK::create();
		break;
	case FeatureType::fAKAZE:
		//imgName = "AKAZEFeat";
		featDescriptor = AKAZE::create();
		break;
	default:
		break;
	}

	//ostringstream ss;
	//ss << this->scalingFactor;
	//imgName += "-Scale" + ss.str();

	//cout << imgName << endl;
	//cout << "computing keypoints..." << endl;
	featDescriptor->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
	featDescriptor->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);

	if ((descriptors1.empty()) || (descriptors2.empty()))
	{
		cout << "FAIL-Not enough descriptors are found" << endl;
		retVal = -2;
		return retVal;//-2 special case non recoverable unlike -1 error
	}
	// Match features.
	//cout << "matching keypoints..." << endl;

	std::vector<DMatch> matches, matches2;
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	vector< vector<DMatch> > matches12, matches21;
	std::vector<DMatch> good_matches1, good_matches2;
	std::vector<DMatch> better_matches;
	int numGoodMatches;

	switch (mType)
	{
	default:
	case MatchType::BASIC:
		//imgName += "-mBasic";
		//cout << "basic matching" << endl;
		matcher->match(descriptors1, descriptors2, matches, Mat());

		// Sort matches by score
		std::sort(matches.begin(), matches.end());
		//cout << "Matches:" << matches.size() << endl;
		// Remove not so good matches
		numGoodMatches = (int)(matches.size() * GOOD_MATCH_PERCENT);//TODO:reject more matches we hardly need 4 points
		matches.erase(matches.begin() + numGoodMatches, matches.end());
		//cout << "Basic Matches:" << matches.size() << endl;
		break;
	case MatchType::RATIO_SYMMETRY:
		//imgName += "-mRatioSymm";
		//cout << "ratio symmetry matching" << endl;
		matcher->knnMatch(descriptors1, descriptors2, matches12, 2);
		matcher->knnMatch(descriptors2, descriptors1, matches21, 2);

		//cout << "Matches1-2:" << matches12.size() << endl;
		//cout << "Matches2-1:" << matches21.size() << endl;

		// ratio test proposed by David Lowe paper
		for (int i = 0; i < matches12.size(); i++) {
			if (matches12[i][0].distance < RATIO_THRESHOLD * matches12[i][1].distance)
				good_matches1.push_back(matches12[i][0]);
		}

		for (int i = 0; i < matches21.size(); i++) {
			if (matches21[i][0].distance < RATIO_THRESHOLD * matches21[i][1].distance)
				good_matches2.push_back(matches21[i][0]);
		}

		//cout << "Good matches1:" << good_matches1.size() << endl;
		//cout << "Good matches2:" << good_matches2.size() << endl;

		// Symmetric Test
		for (int i = 0; i<good_matches1.size(); i++) {
			for (int j = 0; j<good_matches2.size(); j++) {
				if (good_matches1[i].queryIdx == good_matches2[j].trainIdx && good_matches2[j].queryIdx == good_matches1[i].trainIdx) {
					better_matches.push_back(DMatch(good_matches1[i].queryIdx, good_matches1[i].trainIdx, good_matches1[i].distance));
					break;
				}
			}
		}

		//cout << "Better matches:" << better_matches.size() << endl;

		if (better_matches.size() > 4)
			matches = better_matches;
		else
		{
			retVal = -1;
			cout << "TOO Less points !!!FAIL!!!" << endl;
			return retVal;
			////imgName.insert(0, "FAIL");
			//cout << "TOO Less points !!!FAIL!!! Applying basic match" << endl;
			//matcher->match(descriptors1, descriptors2, matches, Mat());

			//// Sort matches by score
			//std::sort(matches.begin(), matches.end());
			////cout << "Matches:" << matches.size() << endl;
			//// Remove not so good matches
			//numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;//TODO:reject more matches we hardly need 4 points
			//matches.erase(matches.begin() + numGoodMatches, matches.end());
			////cout << "Basic Matches:" << matches.size() << endl;
		}

		break;
	case MatchType::FILTER_SYMMETRY:
		//imgName += "-mFilterSymm";
		//cout << "filter symmetry matching" << endl;
		matcher->match(descriptors1, descriptors2, matches);
		matcher->match(descriptors2, descriptors1, matches2);

		std::vector< DMatch > good_matches1, good_matches2, better_matches;
		good_matches1 = filter_distance(descriptors1, matches);
		good_matches2 = filter_distance(descriptors2, matches2);

		//cout << "Good matches1:" << good_matches1.size() << endl;
		//cout << "Good matches2:" << good_matches2.size() << endl;

		for (int i = 0; i<good_matches1.size(); i++)
		{
			DMatch temp1 = good_matches1[i];

			for (int j = 0; j<good_matches2.size(); j++)
			{
				DMatch temp2 = good_matches2[j];
				if (temp1.queryIdx == temp2.trainIdx && temp2.queryIdx == temp1.trainIdx)
				{
					better_matches.push_back(temp1);
					break;
				}
			}
		}

		//cout << "Better matches:" << better_matches.size() << endl;

		if (better_matches.size() > 4)
			matches = better_matches;
		else
		{
			retVal = -1;
			cout << "TOO Less points !!!FAIL!!!" << endl;
			return retVal;
			////imgName.insert(0, "FAIL");
			//cout << "TOO Less points !!!FAIL!!! Applying basic match" << endl;
			//matcher->match(descriptors1, descriptors2, matches, Mat());

			//// Sort matches by score
			//std::sort(matches.begin(), matches.end());
			////cout << "Matches:" << matches.size() << endl;
			//// Remove not so good matches
			//numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;//TODO:reject more matches we hardly need 4 points
			//matches.erase(matches.begin() + numGoodMatches, matches.end());
			////cout << "Basic Matches:" << matches.size() << endl;
		}

		break;
	}


	// Draw top matches
	//if (Utils::DUMPOUTPUTS)
	//{
	//	Mat imMatches;
	//	drawMatches(im1Gray, keypoints1, im2Gray, keypoints2, matches, imMatches);
	//	imwrite(path + imgName + "-matches.jpg", imMatches);
	//}


	// Extract location of good matches
	std::vector<Point2f> points1, points2;

	for (size_t i = 0; i < matches.size(); i++)
	{
		points1.push_back(keypoints1[matches[i].queryIdx].pt);
		points2.push_back(keypoints2[matches[i].trainIdx].pt);
	}


	// align img2 with img1
	// Find homography
	homographyMat = findHomography(points2, points1, RANSAC);

	// if scaling is applied rescale homography mat
	if ((scalingFactor == 0.5) || (scalingFactor == 0.25))
	{
		homographyMat = homographyMat.mul(scaleMat);
	}
	this->transform = homographyMat;
	Mat img2Reg, diffimg;
	// Use homography to warp image
	// apply calculated matrix on original image not on the scaled image
	warpPerspective(origImg2, img2Reg, homographyMat, origImg2.size());

	this->alignedImg = img2Reg.clone();

	//Rect r = GetMaxRoi();

	//cout << "done!" << endl;
	return retVal;
}

//void FeatureBasedRegistration::CalcFeatureBasedReg(Mat& homographyMat, FeatureType fType = FeatureType::fORB)
//{
//	string imgName;
//	// Convert images to grayscale
//	Mat im1Gray, im2Gray;
//	cvtColor(origImg1, im1Gray, CV_BGR2GRAY);
//	cvtColor(origImg2, im2Gray, CV_BGR2GRAY);
//
//	// apply scaling
//	resize(im1Gray, im1Gray, Size(im1Gray.cols*this->scalingFactor, im1Gray.rows*this->scalingFactor));
//	resize(im2Gray, im2Gray, Size(im2Gray.cols*this->scalingFactor, im2Gray.rows*this->scalingFactor));
//
//	// Variables to store keypoints and descriptors
//	std::vector<KeyPoint> keypoints1, keypoints2;
//	Mat descriptors1, descriptors2;
//
//	// Detect features and compute descriptors.
//	Ptr<Feature2D> featDescriptor;
//	switch (fType)
//	{
//	case FeatureType::fORB:
//		imgName = "ORBFeat";
//		featDescriptor = ORB::create(MAX_FEATURES);
//		break;
//	case FeatureType::fKAZE:
//		imgName = "KAZEFeat";
//		featDescriptor = KAZE::create();
//		break;
//	//case FeatureType::fFREAK:
//	//	imgName = "FREAKFeat";
//	//	featDescriptor = FREAK::create();
//	//	break;
//	//case FeatureType::fDAISY:
//	//	imgName = "DAISYFeat";
//	//	featDescriptor = DAISY::create();
//	//	break;
//	case FeatureType::fBRISK:
//		imgName = "BRISKFeat";
//		featDescriptor = BRISK::create();
//		break;
//	//case FeatureType::fBRIEF:
//	//	imgName = "BRIEFFeat";
//	//	featDescriptor = BriefDescriptorExtractor::create();
//	//	break;
//	case FeatureType::fAKAZE:
//		imgName = "AKAZEFeat";
//		featDescriptor = AKAZE::create();
//		break;
//	default:
//		break;
//	}
//
//	ostringstream ss;
//	ss << this->scalingFactor;
//	imgName += "-Scale" + ss.str();
//
//	cout << imgName << endl;
//	cout << "computing keypoints..." << endl;
//	featDescriptor->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
//	featDescriptor->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);
//
//	// Match features.
//	cout << "matching keypoints..." << endl;
//	std::vector<DMatch> matches;
//	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
//	matcher->match(descriptors1, descriptors2, matches, Mat());
//
//	// Sort matches by score
//	std::sort(matches.begin(), matches.end());
//
//	// Remove not so good matches
//	const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;//TODO:reject more matches we hardly need 4 points
//	matches.erase(matches.begin() + numGoodMatches, matches.end());
//
//
//	// Draw top matches
//	if (Utils::DUMPOUTPUTS)
//	{
//		Mat imMatches;
//		drawMatches(im1Gray, keypoints1, im2Gray, keypoints2, matches, imMatches);
//		imwrite(path + imgName + "-matches.jpg", imMatches);
//	}
//
//
//	// Extract location of good matches
//	std::vector<Point2f> points1, points2;
//
//	for (size_t i = 0; i < matches.size(); i++)
//	{
//		points1.push_back(keypoints1[matches[i].queryIdx].pt);
//		points2.push_back(keypoints2[matches[i].trainIdx].pt);
//	}
//
//	// align img1 with img2
//	// Find homography
//	//Mat h = findHomography(points1, points2, RANSAC);
//
//	//Mat img1Reg, diffimg;
//	//// Use homography to warp image
//	//warpPerspective(img1, img1Reg, h, img2.size());
//	//CalcDifference(img2, img1Reg, diffimg);
//
//	// align img2 with img1
//	// Find homography
//	homographyMat = findHomography(points2, points1, RANSAC);
//
//	Mat img2Reg, diffimg;
//	// Use homography to warp image
//	// apply calculated matrix on original image not on the scaled image
//	warpPerspective(origImg2, img2Reg, homographyMat, origImg2.size());
//	this->regOutputImg = img2Reg.clone();
//
//	if (Utils::DUMPOUTPUTS)
//	{
//		//Mat diffimg;
//		//CalcDifference(origImg1, img2Reg, diffimg);
//		//imwrite(path + imgName + "-reg.jpg", img2Reg);
//		//imwrite(path + imgName + "-diff.jpg", diffimg);
//	}
//
//	if (!Utils::LEANEXECUTION)
//	{
//		//write avg image as output
//		Mat avgImg;
//		cv::addWeighted(origImg1, 0.5, img2Reg, 0.5, 0.0, avgImg);
//		imwrite(path + imgName + "-blend.jpg", avgImg);
//
//		cout << "done!" << endl;
//	}
//}

Mat& FeatureBasedRegistration::GetAlignedImage()
{
	return this->alignedImg;
}

//void FeatureBasedRegistration::CalcBRISKFeatureBasedReg()
//{
//	string imgName = "BRISKFeat";
//	// Convert images to grayscale
//	Mat im1Gray, im2Gray;
//	cvtColor(img1, im1Gray, CV_BGR2GRAY);
//	cvtColor(img2, im2Gray, CV_BGR2GRAY);
//
//	// Variables to store keypoints and descriptors
//	std::vector<KeyPoint> keypoints1, keypoints2;
//	Mat descriptors1, descriptors2;
//
//	// Detect SIFT features and compute descriptors.
//	cout << "computing keypoints..." << endl;
//	Ptr<BRISK> brisk = BRISK::create();
//	brisk->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
//	brisk->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);
//
//	// Match features.
//	cout << "matching keypoints..." << endl;
//	std::vector<DMatch> matches;
//	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
//	matcher->match(descriptors1, descriptors2, matches, Mat());
//
//	// Sort matches by score
//	std::sort(matches.begin(), matches.end());
//
//	// Remove not so good matches
//	const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
//	matches.erase(matches.begin() + numGoodMatches, matches.end());
//
//
//	// Draw top matches
//	Mat imMatches;
//	drawMatches(img1, keypoints1, img2, keypoints2, matches, imMatches);
//	imwrite(path + "matches.jpg", imMatches);
//
//
//	// Extract location of good matches
//	std::vector<Point2f> points1, points2;
//
//	for (size_t i = 0; i < matches.size(); i++)
//	{
//		points1.push_back(keypoints1[matches[i].queryIdx].pt);
//		points2.push_back(keypoints2[matches[i].trainIdx].pt);
//	}
//
//	// Find homography
//	Mat h = findHomography(points1, points2, RANSAC);
//
//	Mat img1Reg, diffimg;
//	// Use homography to warp image
//	warpPerspective(img1, img1Reg, h, img2.size());
//	CalcDifference(img1, img1Reg, diffimg);
//
//	imwrite(path + imgName + "-reg.jpg", img1Reg);
//	imwrite(path + imgName + "-diff.jpg", img1Reg);
//}
//
//void FeatureBasedRegistration::CalcKAZEFeatureBasedReg()
//{
//	string imgName = "KAZEFeat";
//	// Convert images to grayscale
//	Mat im1Gray, im2Gray;
//	cvtColor(img1, im1Gray, CV_BGR2GRAY);
//	cvtColor(img2, im2Gray, CV_BGR2GRAY);
//
//	// Variables to store keypoints and descriptors
//	std::vector<KeyPoint> keypoints1, keypoints2;
//	Mat descriptors1, descriptors2;
//
//	// Detect SURF features and compute descriptors.
//	cout << "computing keypoints..." << endl;
//	Ptr<KAZE> kaze = KAZE::create();
//	kaze->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
//	kaze->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);
//
//	// Match features.
//	cout << "matching keypoints..." << endl;
//	std::vector<DMatch> matches;
//	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
//	matcher->match(descriptors1, descriptors2, matches, Mat());
//
//	// Sort matches by score
//	std::sort(matches.begin(), matches.end());
//
//	// Remove not so good matches
//	const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
//	matches.erase(matches.begin() + numGoodMatches, matches.end());
//
//
//	// Draw top matches
//	Mat imMatches;
//	drawMatches(img1, keypoints1, img2, keypoints2, matches, imMatches);
//	imwrite(path + "matches.jpg", imMatches);
//
//
//	// Extract location of good matches
//	std::vector<Point2f> points1, points2;
//
//	for (size_t i = 0; i < matches.size(); i++)
//	{
//		points1.push_back(keypoints1[matches[i].queryIdx].pt);
//		points2.push_back(keypoints2[matches[i].trainIdx].pt);
//	}
//
//	// Find homography
//	Mat h = findHomography(points1, points2, RANSAC);
//
//	Mat img1Reg, diffimg;
//	// Use homography to warp image
//	warpPerspective(img1, img1Reg, h, img2.size());
//	CalcDifference(img1, img1Reg, diffimg);
//
//	imwrite(path + imgName + "-reg.jpg", img1Reg);
//	imwrite(path + imgName + "-diff.jpg", img1Reg);
//}
//
//void FeatureBasedRegistration::CalcAKAZEFeatureBasedReg()
//{
//	string imgName = "AKAZEFeat";
//	// Convert images to grayscale
//	Mat im1Gray, im2Gray;
//	cvtColor(img1, im1Gray, CV_BGR2GRAY);
//	cvtColor(img2, im2Gray, CV_BGR2GRAY);
//
//	// Variables to store keypoints and descriptors
//	std::vector<KeyPoint> keypoints1, keypoints2;
//	Mat descriptors1, descriptors2;
//
//	// Detect SURF features and compute descriptors.
//	cout << "computing keypoints..." << endl;
//	Ptr<AKAZE> akaze = AKAZE::create();
//	akaze->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
//	akaze->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);
//
//	// Match features.
//	cout << "matching keypoints..." << endl;
//	std::vector<DMatch> matches;
//	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
//	matcher->match(descriptors1, descriptors2, matches, Mat());
//
//	// Sort matches by score
//	std::sort(matches.begin(), matches.end());
//
//	// Remove not so good matches
//	const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
//	matches.erase(matches.begin() + numGoodMatches, matches.end());
//
//
//	// Draw top matches
//	Mat imMatches;
//	drawMatches(img1, keypoints1, img2, keypoints2, matches, imMatches);
//	imwrite(path + "matches.jpg", imMatches);
//
//
//	// Extract location of good matches
//	std::vector<Point2f> points1, points2;
//
//	for (size_t i = 0; i < matches.size(); i++)
//	{
//		points1.push_back(keypoints1[matches[i].queryIdx].pt);
//		points2.push_back(keypoints2[matches[i].trainIdx].pt);
//	}
//
//	// Find homography
//	Mat h = findHomography(points1, points2, RANSAC);
//
//	Mat img1Reg, diffimg;
//	// Use homography to warp image
//	warpPerspective(img1, img1Reg, h, img2.size());
//	CalcDifference(img1, img1Reg, diffimg);
//
//	imwrite(path + imgName + "-reg.jpg", img1Reg);
//	imwrite(path + imgName + "-diff.jpg", img1Reg);
//}
//
//void FeatureBasedRegistration::CalcORBFeatureBasedReg()
//{
//	string imgName = "ORBFeat";
//	// Convert images to grayscale
//	Mat im1Gray, im2Gray;
//	cvtColor(img1, im1Gray, CV_BGR2GRAY);
//	cvtColor(img2, im2Gray, CV_BGR2GRAY);
//
//	// Variables to store keypoints and descriptors
//	std::vector<KeyPoint> keypoints1, keypoints2;
//	Mat descriptors1, descriptors2;
//
//	// Detect ORB features and compute descriptors.
//	cout << "computing keypoints..." << endl;
//	Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
//	orb->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
//	orb->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);
//
//	// Match features.
//	cout << "computing keypoints..." << endl;
//	std::vector<DMatch> matches;
//	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
//	matcher->match(descriptors1, descriptors2, matches, Mat());
//
//	// Sort matches by score
//	std::sort(matches.begin(), matches.end());
//
//	// Remove not so good matches
//	const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
//	matches.erase(matches.begin() + numGoodMatches, matches.end());
//
//
//	// Draw top matches
//	Mat imMatches;
//	drawMatches(img1, keypoints1, img2, keypoints2, matches, imMatches);
//	imwrite(path + "matches.jpg", imMatches);
//
//
//	// Extract location of good matches
//	std::vector<Point2f> points1, points2;
//
//	for (size_t i = 0; i < matches.size(); i++)
//	{
//		points1.push_back(keypoints1[matches[i].queryIdx].pt);
//		points2.push_back(keypoints2[matches[i].trainIdx].pt);
//	}
//
//	// align img1 with img2
//	// Find homography
//	//Mat h = findHomography(points1, points2, RANSAC);
//
//	//Mat img1Reg, diffimg;
//	//// Use homography to warp image
//	//warpPerspective(img1, img1Reg, h, img2.size());
//	//CalcDifference(img2, img1Reg, diffimg);
//
//	// align img2 with img1
//	// Find homography
//	Mat h = findHomography(points2, points1, RANSAC);
//
//	Mat img2Reg, diffimg;
//	// Use homography to warp image
//	warpPerspective(img2, img2Reg, h, img2.size());
//	CalcDifference(img1, img2Reg, diffimg);
//
//	imwrite(path + imgName + "-reg.jpg", img2Reg);
//	imwrite(path + imgName + "-diff.jpg", diffimg);
//}