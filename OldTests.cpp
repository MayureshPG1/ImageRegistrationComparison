#include "OldTests.h"
namespace oldtests
{
	void RemoveBlackBorder(Mat& alignedImg, const Mat img1)
	{
		//remove black border after transformation by adding original image values
		//This is done to improve the psnr scores
		//TODO: instead can crop a min bounding rectangle too https://stackoverflow.com/questions/34978705/get-coordinates-of-white-pixels-opencv
		for (int i = 0; i < alignedImg.size().height; ++i)
		{
			for (int j = 0; j < alignedImg.size().width; ++j)
			{
				if ((alignedImg.at<Vec3b>(i, j)[0] <= 15) && (alignedImg.at<Vec3b>(i, j)[1] <= 15) && (alignedImg.at<Vec3b>(i, j)[2] <= 15))
				{
					alignedImg.at<Vec3b>(i, j) = img1.at<Vec3b>(i, j);
				}
			}
		}
	}

	double GetPSNR(const Mat& I1, const Mat& I2)
	{
		Mat s1;
		absdiff(I1, I2, s1);       // |I1 - I2|
		s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
		s1 = s1.mul(s1);           // |I1 - I2|^2

		Scalar s = sum(s1);         // sum elements per channel

		double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels

		if (sse <= 1e-10) // for small values return zero
			return 0;
		else
		{
			double  mse = sse / (double)(I1.channels() * I1.total());
			double psnr = 10.0*log10((255 * 255) / mse);
			return psnr;
		}
	}

	double GetPSNR(const Mat& I1, const Mat& I2, Rect roi)
	{
		Mat I1Roi = I1(roi);
		Mat I2Roi = I2(roi);
		Mat s1;
		absdiff(I1Roi, I2Roi, s1);       // |I1 - I2|
		s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
		s1 = s1.mul(s1);           // |I1 - I2|^2

		Scalar s = sum(s1);         // sum elements per channel

		double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels

		if (sse <= 1e-10) // for small values return zero
			return 0;
		else
		{
			double  mse = sse / (double)(I1Roi.channels() * I1Roi.total());
			double psnr = 10.0*log10((255 * 255) / mse);
			return psnr;
		}
	}


	//All of the hard work
	struct List lsfiles(string folder)
	{
		vector<string> files; //Will be added to List
		vector<string> folders; //Will be added to List
		char search_path[200];
		sprintf_s(search_path, "%s*.*", folder.c_str());
		WIN32_FIND_DATA fd;
		HANDLE hFind = ::FindFirstFile(search_path, &fd);
		if (hFind != INVALID_HANDLE_VALUE)
		{
			do
			{
				// read all (real) files in current folder, delete '!' read other 2 default folder . and ..
				if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
				{

					files.push_back(fd.cFileName);
				}
				else //Put folders into vector
				{
					folders.push_back(fd.cFileName);
				}
			} while (::FindNextFile(hFind, &fd));
			::FindClose(hFind);
		}
		List me;
		me.files = files;
		me.folders = folders;

		return me;
	}

	int RunPSNRTest()
	{
		Mat img1, img2;

		//get all the folders
		string folderpath = "C:\\Projects\\Noctacam\\Registration\\data\\";
		List listDir = lsfiles(folderpath);

		//remove . and ..
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));

		//write csv output file
		ofstream opFile;
		ostringstream ss;
		int64 e1, e2;
		double t;
		opFile.open(folderpath + "PSNRComparison.csv");
		opFile << " ,PSNR,PSNR-ROI,PSNR-0.5,PSNR-0.25,PSNR-ROI-0.25\n";

		//vector<double> scales{ 1.0 };// { 0.25, 0.5, 1.0 };

		// Variables for subpixel alignment
		Mat warpMat = Mat::eye(3, 3, CV_32F);

		// Specify the number of iterations.
		int number_of_iterations = 50;// 1000;

		double termination_eps = 0.1;// 1e-2;

		TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, number_of_iterations, termination_eps);

		Mat img1Gray, img2Gray, img2RegGray, avgImg;

		for (int i = 0; i < listDir.folders.size(); i++)
		{
			string newFolderPath = folderpath + listDir.folders[i] + "\\";
			cout << "******************************************" << endl;
			cout << "for image " + listDir.folders[i] << endl;

			string imgName = listDir.folders[i];
			opFile << imgName + ",";
			ss.str("");

			img1 = imread(newFolderPath + "input 1.JPG", IMREAD_UNCHANGED);
			if (!img1.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}


			img2 = imread(newFolderPath + "input 2.JPG", IMREAD_UNCHANGED);
			if (!img2.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}

			FeatureBasedRegistration fxReg(img1, img2, 0.5);
			double torbratio, psnr, psnr_roi, psnr_0_5, psnr_0_25, psnr_roi_0_25;
			Mat img2Aligned;

			cout << "Applying orb ratio match" << endl;
			e1 = cv::getTickCount();
			// pass returned warpmat to next iteration if required
			int retVal = fxReg.CalcFeatureBasedRegNewMatch(FeatureType::fORB, MatchType::RATIO_SYMMETRY);

			e2 = cv::getTickCount();
			img2Aligned = fxReg.GetAlignedImage();
			RemoveBlackBorder(img2Aligned, img1);
			if (retVal == -1)
			{
				imgName.insert(0, "FAIL");
				torbratio = psnr = -1;
			}
			else
			{
				torbratio = (e2 - e1) / cv::getTickFrequency();
				psnr = GetPSNR(img1, img2Aligned);

				//roi
				Rect roi = fxReg.GetMaxRoi();
				psnr_roi = GetPSNR(img1, img2Aligned, roi);

				//scale 0.5
				resize(img1, img1, Size(img1.cols*0.5, img1.rows*0.5));
				resize(img2Aligned, img2Aligned, Size(img2Aligned.cols*0.5, img2Aligned.rows*0.5));
				psnr_0_5 = GetPSNR(img1, img2Aligned);

				//scale 0.25
				resize(img1, img1, Size(img1.cols*0.5, img1.rows*0.5)); //do half of previous so now it's 0.25
				resize(img2Aligned, img2Aligned, Size(img2Aligned.cols*0.5, img2Aligned.rows*0.5));
				psnr_0_25 = GetPSNR(img1, img2Aligned);

				//roi + scale 0.25
				Rect newroi(roi.x / 4, roi.y / 4, roi.width / 4, roi.height / 4);
				psnr_roi_0_25 = GetPSNR(img1, img2Aligned, newroi);
			}

			ss << psnr;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnr_roi;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnr_0_5;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnr_0_25;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnr_roi_0_25;
			opFile << ss.str() + "\n";
			ss.str("");
			cout << "done!" << endl;
		}

		opFile.close();
	}

	int RunFeatureMatchScaleTest()
	{
		Mat img1, img2;

		//get all the folders
		string folderpath = "C:\\Projects\\Noctacam\\Registration\\data\\";
		List listDir = lsfiles(folderpath);

		//remove . and ..
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));

		//write csv output file
		ofstream opFile;
		ostringstream ss;
		int64 e1, e2;

		opFile.open(folderpath + "FeatureMatchScaleComparison.csv");
		//opFile << " ,imgPSNR,ORB-BASIC PSNR, ORB_RATIO PSNR, ORB_FILTER PSNR, PIXEL_PSNR, ORB_BASIC_TIME, ORB_RATIO_TIME,ORB_FILTER_TIME,PIXEL_TIME\n";

		//vector<double> scales{ 1.0 };// { 0.25, 0.5, 1.0 };

		// Variables for subpixel alignment
		Mat warpMat = Mat::eye(3, 3, CV_32F);

		// Specify the number of iterations.
		int number_of_iterations = 50;// 1000;

		double termination_eps = 0.1;// 1e-2;

		TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, number_of_iterations, termination_eps);

		Mat img1Gray, img2Gray, img2RegGray, avgImg;

		for (int i = 0; i < listDir.folders.size(); i++)
		{
			string newFolderPath = folderpath + listDir.folders[i] + "\\";
			cout << "******************************************" << endl;
			cout << "for image " + listDir.folders[i] << endl;

			string imgName = listDir.folders[i];
			opFile << imgName + ",";
			ss.str("");

			img1 = imread(newFolderPath + "input 1.JPG", IMREAD_UNCHANGED);
			if (!img1.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}


			img2 = imread(newFolderPath + "input 2.JPG", IMREAD_UNCHANGED);
			if (!img2.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}

			//calculate PSNR btw input images
			double imgPsnr = GetPSNR(img1, img2);

			ss << imgPsnr;
			opFile << ss.str() + ",";
			ss.str("");

			int retVal;
			vector<double> scales{ 1.0,0.5,0.25 };

			for (int i = 0; i < scales.size(); i++)
			{
				ss << scales[i];
				string strScale = ss.str();
				ss.clear();

				FeatureBasedRegistration fxReg(img1, img2, scales[i]);
				double torbbasic, psnrorbbasic, torbratio, psnrorbratio;
				Mat img2Aligned;

				//imgName = imgName + strScale + "-ORB";
				cout << "Applying orb basic match" << endl;
				e1 = cv::getTickCount();
				// pass returned warpmat to next iteration if required
				retVal = fxReg.CalcFeatureBasedRegNewMatch(FeatureType::fORB, MatchType::BASIC);
				e2 = cv::getTickCount();
				img2Aligned = fxReg.GetAlignedImage();
				RemoveBlackBorder(img2Aligned, img1);
				if (retVal == -1)
				{
					imgName.insert(0, "FAIL");
					torbbasic = psnrorbbasic = -1;
				}
				else
				{
					torbbasic = (e2 - e1) / cv::getTickFrequency();
					psnrorbbasic = GetPSNR(img1, img2Aligned);
				}

				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					Mat avgImg;
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					imwrite(newFolderPath + imgName + strScale + "-ORB" + "-mBasic" + "-blend.jpg", avgImg);
				}

				cout << "Applying orb ratio match" << endl;
				e1 = cv::getTickCount();
				// pass returned warpmat to next iteration if required
				retVal = fxReg.CalcFeatureBasedRegNewMatch(FeatureType::fORB, MatchType::RATIO_SYMMETRY);

				e2 = cv::getTickCount();
				img2Aligned = fxReg.GetAlignedImage();
				RemoveBlackBorder(img2Aligned, img1);
				if (retVal == -1)
				{
					imgName.insert(0, "FAIL");
					torbratio = psnrorbratio = -1;
				}
				else
				{
					torbratio = (e2 - e1) / cv::getTickFrequency();
					psnrorbratio = GetPSNR(img1, img2Aligned);
				}

				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					Mat avgImg;
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					imwrite(newFolderPath + imgName + strScale + "-ORB" + "-mRatioSymmetry" + "-blend.jpg", avgImg);
				}

				ss << psnrorbbasic;
				opFile << ss.str() + ",";
				ss.str("");

				ss << psnrorbratio;
				opFile << ss.str() + ",";
				ss.str("");

				ss << torbbasic;
				opFile << ss.str() + ",";
				ss.str("");

				ss << torbratio;
				opFile << ss.str() + ",";
				ss.str("");
			}
			opFile << "\n";
			cout << "done!" << endl;
		}

		opFile.close();
	}

	int RunFeatureMatchTest()
	{
		Mat img1, img2;

		//get all the folders
		string folderpath = "C:\\Projects\\Noctacam\\Registration\\data\\";
		List listDir = lsfiles(folderpath);

		//remove . and ..
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));

		//write csv output file
		ofstream opFile;
		ostringstream ss;
		int64 e1, e2;

		opFile.open(folderpath + "FeatureBasedComparison.csv");
		opFile << " ,imgPSNR,ORB-BASIC PSNR, ORB_RATIO PSNR, ORB_FILTER PSNR, PIXEL_PSNR, ORB_BASIC_TIME, ORB_RATIO_TIME,ORB_FILTER_TIME,PIXEL_TIME\n";

		//vector<double> scales{ 1.0 };// { 0.25, 0.5, 1.0 };

		// Variables for subpixel alignment
		Mat warpMat = Mat::eye(3, 3, CV_32F);

		// Specify the number of iterations.
		int number_of_iterations = 50;// 1000;

		double termination_eps = 0.1;// 1e-2;

		TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, number_of_iterations, termination_eps);

		Mat img1Gray, img2Gray, img2RegGray, avgImg;

		for (int i = 0; i < listDir.folders.size(); i++)
		{
			string newFolderPath = folderpath + listDir.folders[i] + "\\";
			cout << "******************************************" << endl;
			cout << "for image " + listDir.folders[i] << endl;

			string imgName = listDir.folders[i];
			opFile << imgName + ",";
			ss.str("");

			img1 = imread(newFolderPath + "input 1.JPG", IMREAD_UNCHANGED);
			if (!img1.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}


			img2 = imread(newFolderPath + "input 2.JPG", IMREAD_UNCHANGED);
			if (!img2.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}

			//calculate PSNR btw input images
			double imgPsnr = GetPSNR(img1, img2);
			int retVal;
			FeatureBasedRegistration fxReg(img1, img2, 1.0);
			double torbbasic, psnrorbbasic, torbratio, psnrorbratio, torbfilter, psnrorbfilter;
			Mat img2Aligned;

			imgName = imgName + "-ORB";
			cout << "Applying orb basic match" << endl;
			e1 = cv::getTickCount();
			// pass returned warpmat to next iteration if required
			retVal = fxReg.CalcFeatureBasedRegNewMatch(FeatureType::fORB, MatchType::BASIC);
			e2 = cv::getTickCount();
			img2Aligned = fxReg.GetAlignedImage();
			RemoveBlackBorder(img2Aligned, img1);
			if (retVal == -1)
			{
				imgName.insert(0, "FAIL");
				torbbasic = psnrorbbasic = -1;
			}
			else
			{
				torbbasic = (e2 - e1) / cv::getTickFrequency();
				psnrorbbasic = GetPSNR(img1, img2Aligned);
			}

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				Mat avgImg;
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-mBasic" + "-blend.jpg", avgImg);
			}

			cout << "Applying orb ratio match" << endl;
			e1 = cv::getTickCount();
			// pass returned warpmat to next iteration if required
			retVal = fxReg.CalcFeatureBasedRegNewMatch(FeatureType::fORB, MatchType::RATIO_SYMMETRY);

			e2 = cv::getTickCount();
			img2Aligned = fxReg.GetAlignedImage();
			RemoveBlackBorder(img2Aligned, img1);
			if (retVal == -1)
			{
				imgName.insert(0, "FAIL");
				torbratio = psnrorbratio = -1;
			}
			else
			{
				torbratio = (e2 - e1) / cv::getTickFrequency();
				psnrorbratio = GetPSNR(img1, img2Aligned);
			}

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				Mat avgImg;
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-mRatioSymmetry" + "-blend.jpg", avgImg);
			}

			cout << "Applying orb filter match" << endl;
			e1 = cv::getTickCount();
			// pass returned warpmat to next iteration if required
			retVal = fxReg.CalcFeatureBasedRegNewMatch(FeatureType::fORB, MatchType::FILTER_SYMMETRY);

			e2 = cv::getTickCount();
			img2Aligned = fxReg.GetAlignedImage();
			RemoveBlackBorder(img2Aligned, img1);
			if (retVal == -1)
			{
				torbfilter = psnrorbfilter = -1;
			}
			else
			{
				torbfilter = (e2 - e1) / cv::getTickFrequency();
				psnrorbfilter = GetPSNR(img1, img2Aligned);
			}

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				Mat avgImg;
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-mFilterSymmetry" + "-blend.jpg", avgImg);
			}

			cout << "Applying subpixel algorithm on standalone" << endl;

			//initialize 
			e1 = cv::getTickCount();
			//start with identity matrix to check standalone performance
			warpMat = Mat::eye(3, 3, CV_32F);
			img2Gray = 0;
			avgImg = 0;

			cvtColor(img1, img1Gray, CV_BGR2GRAY); //original input image
			cvtColor(img2, img2Gray, CV_BGR2GRAY);
			// Run the ECC algorithm. The results are stored in warp_matrix.
			findTransformECC(img1Gray, img2Gray, warpMat, MOTION_HOMOGRAPHY, criteria);

			// Use warpPerspective for Homography
			warpPerspective(img2, img2Aligned, warpMat, img1.size(), INTER_LINEAR + WARP_INVERSE_MAP);

			e2 = cv::getTickCount();
			double tpixel = (e2 - e1) / cv::getTickFrequency();
			double psnrpixel = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + "subpixel" + "-blend.jpg", avgImg);
			}

			//write all output data
			ss << imgPsnr;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrorbbasic;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrorbratio;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrorbfilter;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrpixel;
			opFile << ss.str() + ",";
			ss.str("");

			ss << torbbasic;
			opFile << ss.str() + ",";
			ss.str("");

			ss << torbratio;
			opFile << ss.str() + ",";
			ss.str("");

			ss << torbfilter;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tpixel;
			opFile << ss.str() + "\n";
			ss.str("");

			cout << "done!" << endl;
		}

		opFile.close();
	}

	//int RunFeatureBasedTest()
	//{
	//	Mat img1, img2;
	//
	//	//get all the folders
	//	string folderpath = "C:\\Projects\\Noctacam\\Registration\\data\\";
	//	List listDir = lsfiles(folderpath);
	//
	//	//remove . and ..
	//	listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
	//	listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));
	//
	//	//write csv output file
	//	ofstream opFile;
	//	ostringstream ss;
	//	int64 e1, e2;
	//	double t;
	//	opFile.open(folderpath + "FeatureBasedComparison.csv");
	//	opFile << "Data for execution times of feature based registration algorithms\n";
	//	//opFile << " ,ORB,AKAZE,BRISK\n";
	//	opFile << " ,ORB-time,ORB-PSNR,ORB+subPIX-time,ORB+subPIX-PSNR\n";
	//
	//	vector<double> scales{ 1.0 };// { 0.25, 0.5, 1.0 };
	//
	//	// Variables for subpixel alignment
	//	Mat warpMat = Mat::eye(3, 3, CV_32F);
	//
	//	// Specify the number of iterations.
	//	int number_of_iterations = 50;// 1000;
	//
	//// Specify the threshold of the increment
	//								  // in the correlation coefficient between two iterations
	//	double termination_eps = 0.1;// 1e-2;
	//
	//								   // Define termination criteria
	//	TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, number_of_iterations, termination_eps);
	//
	//	Mat img1Gray, img2Gray, img2RegGray, avgImg;
	//
	//	for (int i = 0; i < listDir.folders.size(); i++)
	//	{
	//		string newFolderPath = folderpath + listDir.folders[i] + "\\";
	//		cout << "******************************************" << endl;
	//		cout << "for image " + listDir.folders[i] << endl;
	//
	//		img1 = imread(newFolderPath + "input 1.JPG", IMREAD_UNCHANGED);
	//		if (!img1.data)
	//		{
	//			cout << "Could not open or find file" << endl;
	//			return -1;
	//		}
	//
	//
	//		img2 = imread(newFolderPath + "input 2.JPG", IMREAD_UNCHANGED);
	//		if (!img2.data)
	//		{
	//			cout << "Could not open or find file" << endl;
	//			return -1;
	//		}
	//
	//		for (int j = 0; j < scales.size(); j++)
	//		{
	//			ss << scales[j];
	//			string scaleName = ss.str();
	//			string imgName = listDir.folders[i] + "-" + ss.str();
	//			opFile << imgName + ",";
	//			ss.str("");
	//
	//			FeatureBasedRegistration fxReg(img1, img2, scales[j], newFolderPath);
	//
	//			e1 = cv::getTickCount();
	//			//orb
	//			double psnr, ssim;
	//			// pass returned warpmat to next iteration if required
	//			fxReg.CalcFeatureBasedReg(warpMat, FeatureType::fORB);
	//
	//			e2 = cv::getTickCount();
	//			t = (e2 - e1) / cv::getTickFrequency();
	//			double orbExeTime = t;
	//			ss << t;
	//			opFile << ss.str() + ",";
	//			ss.str("");
	//
	//			psnr = GetPSNR(img1, fxReg.GetRegisteredImage());
	//			//write psnr and ssim
	//			ss << psnr;
	//			opFile << ss.str() + ",";
	//			ss.str("");
	//			cout << warpMat << endl;
	//			cout << "type:" << warpMat.type() << endl;
	//			warpMat.convertTo(warpMat, CV_32F);
	//			/*************************************************************/
	//			//Run subpixel algorithm on output of feature based alignment to check if results improve
	//			imgName += "+subPixel";
	//
	//			cout << "Applying subpixel registration on feature alignment output" << endl;
	//
	//			e1 = cv::getTickCount();
	//			cvtColor(img1, img1Gray, CV_BGR2GRAY); //original input image
	//			cvtColor(fxReg.GetRegisteredImage(), img2RegGray, CV_BGR2GRAY);//get registered image
	//
	//			// Run the ECC algorithm. The results are stored in warp_matrix.
	//			//warpmat is initial guess taken from feature based algorithm
	//			findTransformECC(img1Gray, img2RegGray, warpMat, MOTION_HOMOGRAPHY, criteria);
	//			cout << warpMat << endl;
	//			Mat img2Aligned;
	//			// Use warpPerspective for Homography
	//			warpPerspective(img2, img2Aligned, warpMat, img1.size(), INTER_LINEAR + WARP_INVERSE_MAP);
	//
	//			e2 = cv::getTickCount();
	//			t = (e2 - e1) / cv::getTickFrequency();
	//
	//			ss << (t + orbExeTime);
	//			opFile << ss.str() + ",";
	//			ss.str("");
	//
	//			//check image similarity
	//			cout << "Checking image similarity" << endl;
	//			psnr = GetPSNR(img1, img2Aligned);
	//			//write psnr
	//			ss << psnr;
	//			opFile << ss.str() + "\n";
	//			ss.str("");
	//
	//			if (!Utils::LEANEXECUTION)
	//			{
	//				//write avg image as output
	//				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
	//				imwrite(newFolderPath + "ORBFeat+subPixel-Scale" + scaleName + "-blend.jpg", avgImg);
	//			}
	//			cout << "done!" << endl;
	//			///*************************************************************/
	//			////ss << ssim.val[1];
	//			////opFile << ss.str() + ",";
	//			////ss.str("");
	//
	//			////ss << ssim.val[2];
	//			////opFile << ss.str() + "\n";
	//			////ss.str("");
	//			////e1 = cv::getTickCount();
	//			//////akaze
	//			////fxReg.CalcFeatureBasedReg(FeatureType::fAKAZE);
	//
	//			////e2 = cv::getTickCount();
	//			////t = (e2 - e1) / cv::getTickFrequency();
	//			////ss << t;
	//			////opFile << ss.str() + ",";
	//			////ss.str("");
	//
	//			////e1 = cv::getTickCount();
	//			//////fxReg.CalcFeatureBasedReg(FeatureType::fKAZE);
	//			//////brisk
	//			////fxReg.CalcFeatureBasedReg(FeatureType::fBRISK);
	//
	//			////e2 = cv::getTickCount();
	//			////t = (e2 - e1) / cv::getTickFrequency();
	//			////ss << t;
	//			////opFile << ss.str() + "\n";
	//			////ss.str("");
	//		}
	//	}
	//
	//	opFile << " Img1-2Psnr,subPIX-time,subPIX-PSNR\n";
	//	for (int i = 0; i < listDir.folders.size(); i++)
	//	{
	//		string newFolderPath = folderpath + listDir.folders[i] + "\\";
	//		cout << "******************************************" << endl;
	//		cout << "Applying subpixel algorithm on standalone" << endl;
	//		cout << "for image " + listDir.folders[i] << endl;
	//		/*************************************************************/
	//		//Run only subpixel algorithm directly on images
	//		string imgName = listDir.folders[i];
	//		opFile << imgName + ",";
	//		ss.str("");
	//
	//		img1 = imread(newFolderPath + "input 1.JPG", IMREAD_UNCHANGED);
	//		if (!img1.data)
	//		{
	//			cout << "Could not open or find file" << endl;
	//			return -1;
	//		}
	//
	//
	//		img2 = imread(newFolderPath + "input 2.JPG", IMREAD_UNCHANGED);
	//		if (!img2.data)
	//		{
	//			cout << "Could not open or find file" << endl;
	//			return -1;
	//		}
	//
	//		//calculate PSNR btw input images
	//		double imgPsnr = GetPSNR(img1, img2);
	//		//write image psnr
	//		ss << imgPsnr;
	//		opFile << ss.str() + ",";
	//		ss.str("");
	//
	//		//initialize 
	//		e1 = cv::getTickCount();
	//		//start with identity matrix to check standalone performance
	//		warpMat = Mat::eye(3, 3, CV_32F);
	//		img2Gray = 0;
	//		avgImg = 0;
	//
	//		cvtColor(img1, img1Gray, CV_BGR2GRAY); //original input image
	//		cvtColor(img2, img2Gray, CV_BGR2GRAY);
	//		// Run the ECC algorithm. The results are stored in warp_matrix.
	//		findTransformECC(img1Gray, img2Gray, warpMat, MOTION_HOMOGRAPHY, criteria);
	//
	//		Mat img2Aligned;
	//		// Use warpPerspective for Homography
	//		warpPerspective(img2, img2Aligned, warpMat, img1.size(), INTER_LINEAR + WARP_INVERSE_MAP);
	//
	//		e2 = cv::getTickCount();
	//		t = (e2 - e1) / cv::getTickFrequency();
	//
	//		//write execution time
	//		ss << t;
	//		opFile << ss.str() + ",";
	//		ss.str("");
	//
	//		cout << "Checking image similarity" << endl;
	//		double psnr = GetPSNR(img1, img2Aligned);
	//		//write psnr
	//		ss << psnr;
	//		opFile << ss.str() + "\n";
	//		ss.str("");
	//
	//		if (!Utils::LEANEXECUTION)
	//		{
	//			//write avg image as output
	//			cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
	//			imwrite(newFolderPath + "subpixel" + "-blend.jpg", avgImg);
	//
	//			//check image similarity
	//			//ss << ssim;
	//			//opFile << ss.str() + "\n";
	//			//ss.str("");
	//		}
	//		cout << "done!" << endl;
	//	}
	//
	//	opFile.close();
	//}

	int RunPixelBasedTest()
	{
		Mat img1, img2;

		//get all the folders
		string folderpath = "C:\\Projects\\Noctacam\\Registration\\data\\";
		List listDir = lsfiles(folderpath);

		//remove . and ..
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));

		//write csv output file
		ofstream opFile;
		ostringstream ss;
		int64 e1, e2;

		opFile.open(folderpath + "FeatureBasedComparison.csv");
		//opFile << " ,ORB,AKAZE,BRISK\n";

		// Variables for subpixel alignment
		Mat warpMat;
		// Specify the number of iterations.
		int number_of_iterations = 7;// 15;// 1000;

									 // Specify the threshold of the increment
									 // in the correlation coefficient between two iterations
		double termination_eps = 0.001;// -1.0;// 0.001;// 1e-2;

									   // Define termination criteria
		TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, number_of_iterations, termination_eps);

		Mat img1Gray, img2Gray, avgImg;

		opFile << " ,Img1-2Psnr,homog-subPIX-psnr,affine-subPIX-psnr,euclid-subPIX-psnr,trans-subPIX-psnr,homog-subPIX-time,affine-subPIX-time,euclid-subPIX-time,trans-subPIX-time,\n";
		for (int i = 0; i < listDir.folders.size(); i++)
		{
			string newFolderPath = folderpath + listDir.folders[i] + "\\";
			cout << "******************************************" << endl;
			cout << "Applying subpixel algorithm on standalone" << endl;
			cout << "for image " + listDir.folders[i] << endl;
			/*************************************************************/
			//Run only subpixel algorithm directly on images
			string imgName = listDir.folders[i];
			opFile << imgName + ",";
			ss.str("");

			img1 = imread(newFolderPath + "input 1.JPG", IMREAD_UNCHANGED);
			if (!img1.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}


			img2 = imread(newFolderPath + "input 2.JPG", IMREAD_UNCHANGED);
			if (!img2.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}

			//calculate PSNR btw input images
			double imgPsnr = GetPSNR(img1, img2);

			//initialize 

			img2Gray = 0;
			avgImg = 0;

			cvtColor(img1, img1Gray, CV_BGR2GRAY); //original input image
			cvtColor(img2, img2Gray, CV_BGR2GRAY);

			/********homography*******/
			cout << "Running Homography" << endl;
			warpMat = Mat::eye(3, 3, CV_32F);
			Mat img2Aligned = Mat::zeros(img2.size(), img2.type());
			e1 = cv::getTickCount();
			// Run the ECC algorithm. The results are stored in warp_matrix.
			findTransformECC(img1Gray, img2Gray, warpMat, MOTION_HOMOGRAPHY, criteria);

			// Use warpPerspective for Homography
			warpPerspective(img2, img2Aligned, warpMat, img1.size(), INTER_LINEAR + WARP_INVERSE_MAP);

			e2 = cv::getTickCount();
			double tHomography = (e2 - e1) / cv::getTickFrequency();

			cout << "Checking image similarity" << endl;
			double psnrHomography = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-homog" + "-subpixel" + "-blend.jpg", avgImg);
			}

			/********MOTION_AFFINE*******/
			cout << "Running affine" << endl;
			warpMat = Mat::eye(2, 3, CV_32F);
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());
			e1 = cv::getTickCount();
			// Run the ECC algorithm. The results are stored in warp_matrix.
			findTransformECC(img1Gray, img2Gray, warpMat, MOTION_AFFINE, criteria);

			// Use warpPerspective for affine
			warpAffine(img2, img2Aligned, warpMat, img1.size(), INTER_LINEAR + WARP_INVERSE_MAP);

			e2 = cv::getTickCount();
			double tAffine = (e2 - e1) / cv::getTickFrequency();

			cout << "Checking image similarity" << endl;
			double psnrAffine = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-affine" + "-subpixel" + "-blend.jpg", avgImg);
			}

			/********MOTION_EUCLIDEAN*******/
			cout << "Running euclid" << endl;
			warpMat = Mat::eye(2, 3, CV_32F);
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());
			e1 = cv::getTickCount();
			// Run the ECC algorithm. The results are stored in warp_matrix.
			findTransformECC(img1Gray, img2Gray, warpMat, MOTION_EUCLIDEAN, criteria);

			// Use warpPerspective for euclid
			warpAffine(img2, img2Aligned, warpMat, img1.size(), INTER_LINEAR + WARP_INVERSE_MAP);

			e2 = cv::getTickCount();
			double tEuclid = (e2 - e1) / cv::getTickFrequency();

			cout << "Checking image similarity" << endl;
			double psnrEuclid = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-euclid" + "-subpixel" + "-blend.jpg", avgImg);
			}

			/********MOTION_TRANSLATION*******/
			cout << "Running trans" << endl;
			warpMat = Mat::eye(2, 3, CV_32F);
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());
			e1 = cv::getTickCount();
			// Run the ECC algorithm. The results are stored in warp_matrix.
			findTransformECC(img1Gray, img2Gray, warpMat, MOTION_TRANSLATION, criteria);

			// Use warpPerspective for trans
			warpAffine(img2, img2Aligned, warpMat, img1.size(), INTER_LINEAR + WARP_INVERSE_MAP);

			e2 = cv::getTickCount();
			double tTrans = (e2 - e1) / cv::getTickFrequency();

			cout << "Checking image similarity" << endl;
			double psnrTrans = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-trans" + "-subpixel" + "-blend.jpg", avgImg);
			}

			ss << imgPsnr;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrHomography;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrAffine;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrEuclid;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrTrans;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tHomography;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tAffine;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tEuclid;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tTrans;
			opFile << ss.str() + "\n";
			ss.str("");

			cout << "done!" << endl;
		}

		opFile.close();

	}

	int RunCascadingPixelBasedTest()
	{
		Mat img1, img2;

		//get all the folders
		string folderpath = "C:\\Projects\\Noctacam\\Registration\\data\\";
		List listDir = lsfiles(folderpath);

		//remove . and ..
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));

		//write csv output file
		ofstream opFile;
		ostringstream ss;
		int64 e1, e2;

		opFile.open(folderpath + "FeatureBasedComparison.csv");
		//opFile << " ,ORB,AKAZE,BRISK\n";

		// Variables for subpixel alignment
		Mat warpMat;
		// Specify the number of iterations.
		int number_of_iterations = 7;// 15;// 1000;

									 // Specify the threshold of the increment
									 // in the correlation coefficient between two iterations
		double termination_eps = 0.001;// -1.0;// 0.001;// 1e-2;

									   // Define termination criteria
		TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, number_of_iterations, termination_eps);

		Mat img1Gray, img2Gray, avgImg, img2Aligned;

		opFile << " ,Img1-2Psnr,trans-subPIX-psnr,euclid-subPIX-psnr,affine-subPIX-psnr,trans-subPIX-time,euclid-subPIX-time,affine-subPIX-time,\n";
		for (int i = 0; i < listDir.folders.size(); i++)
		{
			string newFolderPath = folderpath + listDir.folders[i] + "\\";
			cout << "******************************************" << endl;
			cout << "Applying subpixel algorithm on standalone" << endl;
			cout << "for image " + listDir.folders[i] << endl;
			/*************************************************************/
			//Run only subpixel algorithm directly on images
			string imgName = listDir.folders[i];
			opFile << imgName + ",";
			ss.str("");

			img1 = imread(newFolderPath + "input 1.JPG", IMREAD_UNCHANGED);
			if (!img1.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}


			img2 = imread(newFolderPath + "input 2.JPG", IMREAD_UNCHANGED);
			if (!img2.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}

			//calculate PSNR btw input images
			double imgPsnr = GetPSNR(img1, img2);

			//initialize 

			img2Gray = 0;
			avgImg = 0;

			cvtColor(img1, img1Gray, CV_BGR2GRAY); //original input image
			cvtColor(img2, img2Gray, CV_BGR2GRAY);

			/********MOTION_TRANSLATION*******/
			cout << "Running trans" << endl;
			warpMat = Mat::eye(2, 3, CV_32F);
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());
			e1 = cv::getTickCount();
			// Run the ECC algorithm. The results are stored in warp_matrix.
			findTransformECC(img1Gray, img2Gray, warpMat, MOTION_TRANSLATION, criteria);

			/**********/
			//keep calculated translation matrix
			Mat transMat = warpMat.clone();
			/**********/
			// Use warpPerspective for trans
			warpAffine(img2, img2Aligned, warpMat, img1.size(), INTER_LINEAR + WARP_INVERSE_MAP);

			e2 = cv::getTickCount();
			double tTrans = (e2 - e1) / cv::getTickFrequency();

			cout << "Checking image similarity" << endl;
			double psnrTrans = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-trans" + "-subpixel" + "-blend.jpg", avgImg);
			}

			/********MOTION_EUCLIDEAN*******/
			cout << "Running euclid" << endl;
			//warpMat = Mat::eye(2, 3, CV_32F);
			//pass previously calculated translation matrix as input
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());
			e1 = cv::getTickCount();
			// Run the ECC algorithm. The results are stored in warp_matrix.
			findTransformECC(img1Gray, img2Gray, warpMat, MOTION_EUCLIDEAN, criteria);

			// Use warpPerspective for euclid
			warpAffine(img2, img2Aligned, warpMat, img1.size(), INTER_LINEAR + WARP_INVERSE_MAP);

			e2 = cv::getTickCount();
			double tEuclid = (e2 - e1) / cv::getTickFrequency();

			cout << "Checking image similarity" << endl;
			double psnrEuclid = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-euclid" + "-subpixel" + "-blend.jpg", avgImg);
			}

			/********MOTION_AFFINE*******/
			cout << "Running affine" << endl;
			//warpMat = Mat::eye(2, 3, CV_32F);
			//pass previously calculated translation matrix as input
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());
			e1 = cv::getTickCount();
			// Run the ECC algorithm. The results are stored in warp_matrix.
			//pass previously calculated translation matrix as input
			findTransformECC(img1Gray, img2Gray, transMat, MOTION_AFFINE, criteria);

			// Use warpPerspective for affine
			warpAffine(img2, img2Aligned, transMat, img1.size(), INTER_LINEAR + WARP_INVERSE_MAP);

			e2 = cv::getTickCount();
			double tAffine = (e2 - e1) / cv::getTickFrequency();

			cout << "Checking image similarity" << endl;
			double psnrAffine = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-affine" + "-subpixel" + "-blend.jpg", avgImg);
			}

			ss << imgPsnr;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrTrans;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrEuclid;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrAffine;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tTrans;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tEuclid;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tAffine;
			opFile << ss.str() + "\n";
			ss.str("");

			cout << "done!" << endl;
		}

		opFile.close();

	}


	int RunPyramidPixelBasedTest()
	{
		Mat img1, img2;

		//get all the folders
		string folderpath = "C:\\Projects\\Noctacam\\Registration\\data\\";
		List listDir = lsfiles(folderpath);

		//remove . and ..
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));

		//write csv output file
		ofstream opFile;
		ostringstream ss;
		int64 e1, e2;

		opFile.open(folderpath + "PyramidBasedPixelComparison.csv");
		//opFile << " ,ORB,AKAZE,BRISK\n";

		Mat avgImg;

		opFile << " ,Img1-2Psnr,homog-subPIX-psnr,affine-subPIX-psnr,euclid-subPIX-psnr,trans-subPIX-psnr,homog-subPIX-time,affine-subPIX-time,euclid-subPIX-time,trans-subPIX-time,\n";
		for (int i = 0; i < listDir.folders.size(); i++)
		{
			string newFolderPath = folderpath + listDir.folders[i] + "\\";
			cout << "******************************************" << endl;
			cout << "Applying PYRAMID subpixel algorithm" << endl;
			cout << "for image " + listDir.folders[i] << endl;
			/*************************************************************/
			//Run only subpixel algorithm directly on images
			string imgName = listDir.folders[i] + "-Pyr";
			opFile << imgName + ",";
			ss.str("");

			img1 = imread(newFolderPath + "input 1.JPG", IMREAD_UNCHANGED);
			if (!img1.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}


			img2 = imread(newFolderPath + "input 2.JPG", IMREAD_UNCHANGED);
			if (!img2.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}

			//calculate PSNR btw input images
			double imgPsnr = GetPSNR(img1, img2);

			//initialize 
			//calculate pyramid only once
			PyramidBasedPixelAlignment pbpAlignment(img1, img2);//,4

																/********homography*******/
			cout << "Running Homography" << endl;
			Mat img2Aligned = Mat::zeros(img2.size(), img2.type());
			e1 = cv::getTickCount();
			// Run the pyramid algo for homography
			pbpAlignment.RunPyramidBasedPixelAlignment(MotionModel::HOMOGRAPHY);
			img2Aligned = pbpAlignment.GetAlignedImage();
			e2 = cv::getTickCount();
			RemoveBlackBorder(img2Aligned, img1);
			double tHomography = (e2 - e1) / cv::getTickFrequency();

			cout << "Checking image similarity" << endl;
			double psnrHomography = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-homog" + "-subpixel" + "-blend.jpg", avgImg);
			}

			/********MOTION_AFFINE*******/
			cout << "Running affine" << endl;
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());

			e1 = cv::getTickCount();
			// Run the pyramid algo for affine
			pbpAlignment.RunPyramidBasedPixelAlignment(MotionModel::AFFINE);
			img2Aligned = pbpAlignment.GetAlignedImage();
			e2 = cv::getTickCount();
			RemoveBlackBorder(img2Aligned, img1);

			double tAffine = (e2 - e1) / cv::getTickFrequency();

			cout << "Checking image similarity" << endl;
			double psnrAffine = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-affine" + "-subpixel" + "-blend.jpg", avgImg);
			}

			/********MOTION_EUCLIDEAN*******/
			cout << "Running euclid" << endl;
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());
			e1 = cv::getTickCount();
			// Run the pyramid algo for euclid
			pbpAlignment.RunPyramidBasedPixelAlignment(MotionModel::EUCLIDEAN);
			img2Aligned = pbpAlignment.GetAlignedImage();
			e2 = cv::getTickCount();
			RemoveBlackBorder(img2Aligned, img1);
			double tEuclid = (e2 - e1) / cv::getTickFrequency();

			cout << "Checking image similarity" << endl;
			double psnrEuclid = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-euclid" + "-subpixel" + "-blend.jpg", avgImg);
			}

			/********MOTION_TRANSLATION*******/
			cout << "Running trans" << endl;
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());
			e1 = cv::getTickCount();
			// Run the pyramid algo for trans
			pbpAlignment.RunPyramidBasedPixelAlignment(MotionModel::TRANSLATION);
			img2Aligned = pbpAlignment.GetAlignedImage();
			e2 = cv::getTickCount();
			RemoveBlackBorder(img2Aligned, img1);
			double tTrans = (e2 - e1) / cv::getTickFrequency();

			cout << "Checking image similarity" << endl;
			double psnrTrans = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-trans" + "-subpixel" + "-blend.jpg", avgImg);
			}

			ss << imgPsnr;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrHomography;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrAffine;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrEuclid;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrTrans;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tHomography;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tAffine;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tEuclid;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tTrans;
			opFile << ss.str() + "\n";
			ss.str("");

			cout << "done!" << endl;
		}

		opFile.close();
	}

	int RunOpticalFlowBasedAlignmentTest()
	{
		Mat img1, img2;

		//get all the folders
		string folderpath = "C:\\Projects\\Noctacam\\Registration\\data\\";
		List listDir = lsfiles(folderpath);

		//remove . and ..
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));

		//write csv output file
		ofstream opFile;
		ostringstream ss;
		int64 e1, e2;

		opFile.open(folderpath + "OpticalFlowBasedPixelComparison.csv");
		//opFile << " ,ORB,AKAZE,BRISK\n";

		Mat avgImg;

		opFile << " ,Img1-2Psnr,optflow-psnr,optflow-time,\n";
		for (int i = 0; i < listDir.folders.size(); i++)
		{
			string newFolderPath = folderpath + listDir.folders[i] + "\\";
			cout << "******************************************" << endl;
			cout << "Applying optical flow algorithm" << endl;
			cout << "for image " + listDir.folders[i] << endl;
			/*************************************************************/
			//Run only subpixel algorithm directly on images
			string imgName = listDir.folders[i] + "-optflow";
			opFile << imgName + ",";
			ss.str("");

			img1 = imread(newFolderPath + "input 1.JPG", IMREAD_UNCHANGED);
			if (!img1.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}


			img2 = imread(newFolderPath + "input 2.JPG", IMREAD_UNCHANGED);
			if (!img2.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}

			//calculate PSNR btw input images
			double imgPsnr = GetPSNR(img1, img2);

			//initialize 
			//calculate optical flow
			OpticalFlowAlignment optFlowAlignment(img1, img2);

			/********optical flow*******/
			cout << "Running optical flow" << endl;
			Mat img2Aligned = Mat::zeros(img2.size(), img2.type());
			e1 = cv::getTickCount();
			// Run the pyramid algo for homography
			optFlowAlignment.RunOpticalFlowAlignment();
			img2Aligned = optFlowAlignment.GetAlignedImage();
			e2 = cv::getTickCount();
			RemoveBlackBorder(img2Aligned, img1);
			double toptflow = (e2 - e1) / cv::getTickFrequency();

			cout << "Checking image similarity" << endl;
			double psnrOptFlow = GetPSNR(img1, img2Aligned);

			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				imwrite(newFolderPath + imgName + "-blend.jpg", avgImg);
			}


			ss << imgPsnr;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrOptFlow;
			opFile << ss.str() + ",";
			ss.str("");

			ss << toptflow;
			opFile << ss.str() + "\n";
			ss.str("");

			cout << "done!" << endl;
		}

		opFile.close();
	}

	void OverlayText(Mat& ip, string line1, string line2)
	{
		cv::putText(ip,
			line1,
			cv::Point(25, 65), // Coordinates
			cv::FONT_HERSHEY_COMPLEX, // Font
			2.0, // Scale. 2.0 = 2x bigger
			cv::Scalar(0, 0, 255), // BGR Color
			2 // Line Thickness (Optional)
		); // Anti-alias (Optional)

		cv::putText(ip,
			line2,
			cv::Point(25, 135), // Coordinates
			cv::FONT_HERSHEY_COMPLEX, // Font
			2.0, // Scale. 2.0 = 2x bigger
			cv::Scalar(0, 0, 255), // BGR Color
			2 // Line Thickness (Optional)
		); // Anti-alias (Optional)
	}
	
	int FinBigTest_AllAlgo()
	{
		Mat img1, img2;

		//get all the folders
		string folderpath = "C:\\Projects\\Noctacam\\Registration\\data\\";
		List listDir = lsfiles(folderpath);

		//remove . and ..
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));

		//write csv output file
		ofstream opFile;
		ostringstream ss;
		int64 e1, e2;

		opFile.open(folderpath + "FinBigTestAll.csv");

		Mat avgImg;

		opFile << " ,Img1-2Psnr,optflow-psnr,orbratio-psnr,affine-psnr,euclid-psnr,trans-psnr,optflow-time,orbratio-time,affine-time,euclid-time,trans-time\n";
		for (int i = 0; i < listDir.folders.size(); i++)
		{
			string newFolderPath = folderpath + listDir.folders[i] + "\\";
			cout << "******************************************" << endl;
			cout << "for image " + listDir.folders[i] << endl;


			string imgName = listDir.folders[i];
			opFile << imgName + ",";
			ss.str("");

			img1 = imread(newFolderPath + "input 1.JPG", IMREAD_UNCHANGED);
			if (!img1.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}


			img2 = imread(newFolderPath + "input 2.JPG", IMREAD_UNCHANGED);
			if (!img2.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}

			//calculate PSNR btw input images
			double imgPsnr = GetPSNR(img1, img2);

			Mat img2Aligned = Mat::zeros(img2.size(), img2.type());
			Rect roi;
			string line1, line2;
			/********optical flow*******/
			cout << "Applying optical flow algorithm" << endl;
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());

			e1 = cv::getTickCount();
			OpticalFlowAlignment optFlowAlignment(img1, img2);
			int retVal = optFlowAlignment.RunOpticalFlowAlignment();

			string tmp = imgName;
			double psnrOptFlow, toptflow;
			if (retVal == -1)
			{
				img2Aligned = img2;
				psnrOptFlow = -1;
				tmp.insert(0, "FAIL");
			}
			else
			{

				img2Aligned = optFlowAlignment.GetAlignedImage();
				roi = optFlowAlignment.GetMaxRoi();

				psnrOptFlow = GetPSNR(img1, img2Aligned, roi);
			}
		
			e2 = cv::getTickCount();
			toptflow = (e2 - e1) / cv::getTickFrequency();

			line1 = tmp + "-OptFlow" + "-blend.jpg";
			ss << "PSNR:" << psnrOptFlow << " Exe TIme:" << toptflow;
			line2 = ss.str();
			ss.str("");
			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				OverlayText(avgImg, line1, line2);
				imwrite(newFolderPath + line1, avgImg);
			}

			/********ORB Ratio Scale*******/
			cout << "Applying orb ratio scale" << endl;
			ss.str("");
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());

			e1 = cv::getTickCount();
			FeatureBasedRegistration fxReg(img1, img2, 0.5);
			retVal = fxReg.CalcFeatureBasedRegNewMatch(FeatureType::fORB, MatchType::RATIO_SYMMETRY);

			tmp = imgName;
			double psnrorbratio, torbratio;
			if (retVal != 0)
			{
				img2Aligned = img2;

				if(retVal == -1)
					tmp.insert(0, "FAIL");
				else if(retVal == -2)
					tmp.insert(0, "FAIL-NoDESC");

				psnrorbratio = -1;
			}
			else
			{
				img2Aligned = fxReg.GetAlignedImage();
				roi = fxReg.GetMaxRoi();
				psnrorbratio = GetPSNR(img1, img2Aligned, roi);
			}
			//if (retVal == -2)
			//{
			//	img2Aligned = img2;
			//	tmp.insert(0, "FAIL-NoDESC");
			//	psnrorbratio = -1;
			//}
			//else
			//{
			//	img2Aligned = fxReg.GetAlignedImage();
			//	roi = fxReg.GetMaxRoi();
			//}

			//if (retVal == -1)
			//{
			//	tmp.insert(0, "FAIL");
			//	psnrorbratio = -1;
			//}
			//else
			//{
			//	psnrorbratio = GetPSNR(img1, img2Aligned, roi);
			//}
			e2 = cv::getTickCount();

			torbratio = (e2 - e1) / cv::getTickFrequency();

			line1 = tmp + "-0.5" + "-ORB" + "-mRatioSymmetry" + "-blend.jpg";
			ss << "PSNR:" << psnrorbratio << " Exe TIme:" << torbratio;
			line2 = ss.str();
			ss.str("");
			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				OverlayText(avgImg, line1, line2);
				imwrite(newFolderPath + line1, avgImg);
			}

			/********MOTION_AFFINE*******/
			cout << "Running affine" << endl;
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());

			e1 = cv::getTickCount();
			PyramidBasedPixelAlignment pbpAlignment(img1, img2);
			// Run the pyramid algo for affine
			retVal = pbpAlignment.RunPyramidBasedPixelAlignment(MotionModel::AFFINE);

			tmp = imgName;
			double psnrAffine, tAffine;
			if (retVal == -1)
			{
				img2Aligned = img2;
				psnrAffine = -1;
				tmp.insert(0, "FAIL");
			}
			else
			{
				img2Aligned = pbpAlignment.GetAlignedImage();
				roi = pbpAlignment.GetMaxRoi();
				psnrAffine = GetPSNR(img1, img2Aligned, roi);
			}

			e2 = cv::getTickCount();

			tAffine = (e2 - e1) / cv::getTickFrequency();

			line1 = tmp + "-PyramidPixel" + "-Affine" + "-blend.jpg";
			ss << "PSNR:" << psnrAffine << " Exe TIme:" << tAffine;
			line2 = ss.str();
			ss.str("");
			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				OverlayText(avgImg, line1, line2);
				imwrite(newFolderPath + line1, avgImg);
			}

			/********MOTION_EUCLIDEAN*******/
			cout << "Running euclid" << endl;
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());

			e1 = cv::getTickCount();
			PyramidBasedPixelAlignment pbpAlignment1(img1, img2);
			// Run the pyramid algo for euclid
			retVal = pbpAlignment1.RunPyramidBasedPixelAlignment(MotionModel::EUCLIDEAN);
			tmp = imgName;
			double psnrEuclid, tEuclid;
			if (retVal == -1)
			{
				img2Aligned = img2;
				psnrEuclid = -1;
				tmp.insert(0, "FAIL");
			}
			else
			{
				img2Aligned = pbpAlignment1.GetAlignedImage();
				roi = pbpAlignment1.GetMaxRoi();
				psnrEuclid = GetPSNR(img1, img2Aligned, roi);
			}

			e2 = cv::getTickCount();

			tEuclid = (e2 - e1) / cv::getTickFrequency();
			line1 = tmp + "-PyramidPixel" + "-Euclid" + "-blend.jpg";
			ss << "PSNR:" << psnrEuclid << " Exe TIme:" << tEuclid;
			line2 = ss.str();
			ss.str("");
			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				OverlayText(avgImg, line1, line2);
				imwrite(newFolderPath + line1, avgImg);
			}

			/********MOTION_TRANSLATION*******/
			cout << "Running trans" << endl;
			img2Aligned = Mat::zeros(img2.size(), img2.type());
			avgImg = Mat::zeros(img2.size(), img2.type());

			e1 = cv::getTickCount();
			PyramidBasedPixelAlignment pbpAlignment2(img1, img2);
			retVal = pbpAlignment2.RunPyramidBasedPixelAlignment(MotionModel::TRANSLATION);

			tmp = imgName;
			double psnrTrans, tTrans;
			if (retVal == -1)
			{
				img2Aligned = img2;
				psnrTrans = -1;
				tmp.insert(0, "FAIL");
			}
			else
			{
				img2Aligned = pbpAlignment2.GetAlignedImage();
				roi = pbpAlignment2.GetMaxRoi();
				psnrTrans = GetPSNR(img1, img2Aligned, roi);
			}

			e2 = cv::getTickCount();
			tTrans = (e2 - e1) / cv::getTickFrequency();

			line1 = tmp + "-PyramidPixel" + "-Trans" + "-blend.jpg";
			ss << "PSNR:" << psnrTrans << " Exe TIme:" << tTrans;
			line2 = ss.str();
			ss.str("");
			if (!Utils::LEANEXECUTION)
			{
				//write avg image as output
				cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
				OverlayText(avgImg, line1, line2);
				imwrite(newFolderPath + line1, avgImg);
			}


			ss << imgPsnr;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrOptFlow;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrorbratio;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrAffine;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrEuclid;
			opFile << ss.str() + ",";
			ss.str("");

			ss << psnrTrans;
			opFile << ss.str() + ",";
			ss.str("");

			ss << toptflow;
			opFile << ss.str() + ",";
			ss.str("");

			ss << torbratio;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tAffine;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tEuclid;
			opFile << ss.str() + ",";
			ss.str("");

			ss << tTrans;
			opFile << ss.str() + "\n";
			ss.str("");

			cout << "done!" << endl;
		}

		opFile.close();
	}

	int MultiImage_FinBigTest_AllAlgo()
	{
		Mat img1, img2;

		//get all the folders
		string folderpath = "C:\\Projects\\Noctacam\\Registration\\bigdata\\";
		List listDir = lsfiles(folderpath);

		//remove . and ..
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));

		//write csv output file
		ofstream opFile;
		ostringstream ss;
		int64 e1, e2;

		opFile.open(folderpath + "MultiImage_FinBigTestAll.csv");

		Mat avgImg;
		opFile << " ,Img1-2Psnr,optflow-psnr,orbratio-psnr,affine-psnr,optflow-time,orbratio-time,affine-time,\n";
		for (int i = 0; i < listDir.folders.size(); i++)
		{
			string newFolderPath = folderpath + listDir.folders[i] + "\\";

			//get only *.jpg
			vector<string> inputImgNames;
			List listDir1 = lsfiles(newFolderPath);

			for (int l = 0; l < listDir1.files.size(); l++)
			{
				if (listDir1.files[l].find("jpg") != string::npos)
				{
					inputImgNames.push_back(listDir1.files[l]);
				}
			}

			std::sort(inputImgNames.begin(), inputImgNames.end());

			img1 = imread(newFolderPath + inputImgNames[0], IMREAD_UNCHANGED);
			if (!img1.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}

			for (int j = 1; j < inputImgNames.size(); j++)
			{
				string imgName = listDir.folders[i] + "-" + inputImgNames[0] + "-" + inputImgNames[j];
				opFile << imgName + ",";
				ss.str("");

				cout << "******************************************" << endl;
				cout << "for image:" + imgName << endl;

				img2 = imread(newFolderPath + inputImgNames[j], IMREAD_UNCHANGED);
				if (!img2.data)
				{
					cout << "Could not open or find file" << endl;
					return -1;
				}

				//calculate PSNR btw input images
				double imgPsnr = GetPSNR(img1, img2);

				Mat img2Aligned = Mat::zeros(img2.size(), img2.type());
				Rect roi;
				string line1, line2;
				/********optical flow*******/
				cout << "Applying optical flow algorithm" << endl;
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				e1 = cv::getTickCount();
				OpticalFlowAlignment optFlowAlignment(img1, img2);
				int retVal = optFlowAlignment.RunOpticalFlowAlignment();

				string tmp = imgName;
				double psnrOptFlow, toptflow;
				if (retVal == -1)
				{
					img2Aligned = img2;
					psnrOptFlow = -1;
					tmp.insert(0, "FAIL");
				}
				else
				{

					img2Aligned = optFlowAlignment.GetAlignedImage();
					roi = optFlowAlignment.GetMaxRoi();

					psnrOptFlow = GetPSNR(img1, img2Aligned, roi);
				}

				e2 = cv::getTickCount();
				toptflow = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-OptFlow" + "-blend.jpg";
				ss << "PSNR:" << psnrOptFlow << " Exe TIme:" << toptflow;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				/********ORB Ratio Scale*******/
				cout << "Applying orb ratio scale" << endl;
				ss.str("");
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				e1 = cv::getTickCount();
				FeatureBasedRegistration fxReg(img1, img2, 0.5);
				retVal = fxReg.CalcFeatureBasedRegNewMatch(FeatureType::fORB, MatchType::RATIO_SYMMETRY);

				tmp = imgName;
				double psnrorbratio, torbratio;
				if (retVal != 0)
				{
					img2Aligned = img2;

					if (retVal == -1)
						tmp.insert(0, "FAIL");
					else if (retVal == -2)
						tmp.insert(0, "FAIL-NoDESC");

					psnrorbratio = -1;
				}
				else
				{
					img2Aligned = fxReg.GetAlignedImage();
					roi = fxReg.GetMaxRoi();
					psnrorbratio = GetPSNR(img1, img2Aligned, roi);
				}

				e2 = cv::getTickCount();

				torbratio = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-0.5" + "-ORB" + "-mRatioSymmetry" + "-blend.jpg";
				ss << "PSNR:" << psnrorbratio << " Exe TIme:" << torbratio;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				/********MOTION_AFFINE*******/
				cout << "Running affine" << endl;
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				e1 = cv::getTickCount();
				PyramidBasedPixelAlignment pbpAlignment(img1, img2);
				// Run the pyramid algo for affine
				retVal = pbpAlignment.RunPyramidBasedPixelAlignment(MotionModel::AFFINE);

				tmp = imgName;
				double psnrAffine, tAffine;
				if (retVal == -1)
				{
					img2Aligned = img2;
					psnrAffine = -1;
					tmp.insert(0, "FAIL");
				}
				else
				{
					img2Aligned = pbpAlignment.GetAlignedImage();
					roi = pbpAlignment.GetMaxRoi();
					psnrAffine = GetPSNR(img1, img2Aligned, roi);
				}

				e2 = cv::getTickCount();

				tAffine = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-PyramidPixel" + "-Affine" + "-blend.jpg";
				ss << "PSNR:" << psnrAffine << " Exe TIme:" << tAffine;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				ss << imgPsnr;
				opFile << ss.str() + ",";
				ss.str("");

				ss << psnrOptFlow;
				opFile << ss.str() + ",";
				ss.str("");

				ss << psnrorbratio;
				opFile << ss.str() + ",";
				ss.str("");

				ss << psnrAffine;
				opFile << ss.str() + ",";
				ss.str("");

				ss << toptflow;
				opFile << ss.str() + ",";
				ss.str("");

				ss << torbratio;
				opFile << ss.str() + ",";
				ss.str("");

				ss << tAffine;
				opFile << ss.str() + ",";
				ss.str("");

				opFile << "\n";
			}
			cout << "done!" << endl;
		}

		opFile.close();
	}

	int MultiImage_FinCorpus_NoPSNR()
	{
		Mat img1, img2;

		//get all the folders
		string folderpath = "C:\\Projects\\Noctacam\\Registration\\FinCorpus\\";
		List listDir = lsfiles(folderpath);

		//remove . and ..
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));

		//write csv output file
		ofstream opFile;
		ostringstream ss;
		int64 e1, e2;

		opFile.open(folderpath + "MultiImage_FinCorpus.csv");

		Mat avgImg;
		opFile << " ,optflow-psnr,optflow-EG-psnr,optflow-FWBW-psnr,optflow-ORB-psnr,orbratio-psnr,optflow-time,optflow-EG-time,optflow-FWBW-time,optflow-ORB-time,orbratio-time,\n";
		for (int i = 0; i < listDir.folders.size(); i++)
		{
			string newFolderPath = folderpath + listDir.folders[i] + "\\";

			//get only *.jpg
			vector<string> inputImgNames;
			List listDir1 = lsfiles(newFolderPath);

			for (int l = 0; l < listDir1.files.size(); l++)
			{
				//if (listDir1.files[l].find("jpg") != string::npos)
				if(listDir1.files[l].length() == 11)//use only 'input *.jpg' not the saved output files
				{
					inputImgNames.push_back(listDir1.files[l]);
				}
			}

			std::sort(inputImgNames.begin(), inputImgNames.end());

			img1 = imread(newFolderPath + inputImgNames[0], IMREAD_UNCHANGED);
			if (!img1.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}

			for (int j = 1; j < inputImgNames.size(); j++)
			{
				string imgName = listDir.folders[i] + "-" + inputImgNames[0] + "-" + inputImgNames[j];
				opFile << imgName + ",";
				ss.str("");

				cout << "******************************************" << endl;
				cout << "for image:" + imgName << endl;

				img2 = imread(newFolderPath + inputImgNames[j], IMREAD_UNCHANGED);
				if (!img2.data)
				{
					cout << "Could not open or find file" << endl;
					return -1;
				}

				//calculate PSNR btw input images
				//double imgPsnr = GetPSNR(img1, img2);

				Mat img2Aligned = Mat::zeros(img2.size(), img2.type());
				Rect roi;
				string line1, line2;

				/********optical flow*******/
				cout << "Applying optical flow algorithm" << endl;
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				e1 = cv::getTickCount();
				OpticalFlowAlignment optFlowAlignment(img1, img2);
				//int retVal = optFlowAlignment.RunOpticalFlowAlignment();
				int retVal = optFlowAlignment.RunOpticalFlowAlignment();

				string tmp = imgName;
				double psnrOptFlow, toptflow;
				if (retVal == -1)
				{
					img2Aligned = img2;
					psnrOptFlow = -1;
					tmp.insert(0, "FAIL");
				}
				else
				{
					img2Aligned = optFlowAlignment.GetAlignedImage();
					//roi = optFlowAlignment.GetMaxRoi();
					//psnrOptFlow = GetPSNR(img1, img2Aligned, roi);
					psnrOptFlow = 0;
				}

				e2 = cv::getTickCount();
				toptflow = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-OptFlow" + "-blend.jpg";
				ss << "PSNR:" << psnrOptFlow << " Exe TIme:" << toptflow;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				/********optical flow eigen*******/
				cout << "Applying optical flow with Eigen algorithm" << endl;
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				e1 = cv::getTickCount();
				//OpticalFlowAlignment optFlowAlignment(img1, img2);
				retVal = optFlowAlignment.RunOpticalFlowAlignment_OPT(false, false, true);

				tmp = imgName;
				double psnrOptFlowEG, toptflowEG;
				if (retVal == -1)
				{
					img2Aligned = img2;
					psnrOptFlowEG = -1;
					tmp.insert(0, "FAIL");
				}
				else
				{
					img2Aligned = optFlowAlignment.GetAlignedImage();
					//roi = optFlowAlignment.GetMaxRoi();
					//psnrOptFlow = GetPSNR(img1, img2Aligned, roi);
					psnrOptFlowEG = 0;
				}

				e2 = cv::getTickCount();
				toptflowEG = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-OptFlow-Eigen" + "-blend.jpg";
				ss << "PSNR:" << psnrOptFlowEG << " Exe TIme:" << toptflowEG;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				/********optical flow fwbw*******/
				cout << "Applying optical flow with fwbw algorithm" << endl;
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				e1 = cv::getTickCount();
				//OpticalFlowAlignment optFlowAlignment(img1, img2);
				retVal = optFlowAlignment.RunOpticalFlowAlignment_OPT(true, false, false);

				tmp = imgName;
				double psnrOptFlowFWBW, toptflowFWBW;
				if (retVal == -1)
				{
					img2Aligned = img2;
					psnrOptFlowFWBW = -1;
					tmp.insert(0, "FAIL");
				}
				else
				{
					img2Aligned = optFlowAlignment.GetAlignedImage();
					//roi = optFlowAlignment.GetMaxRoi();
					//psnrOptFlow = GetPSNR(img1, img2Aligned, roi);
					psnrOptFlowFWBW = 0;
				}

				e2 = cv::getTickCount();
				toptflowFWBW = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-OptFlow-FWBW" + "-blend.jpg";
				ss << "PSNR:" << psnrOptFlowFWBW << " Exe TIme:" << toptflowFWBW;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				/********optical flow ORB*******/
				cout << "Applying optical flow with ORB algorithm" << endl;
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				e1 = cv::getTickCount();
				//OpticalFlowAlignment optFlowAlignment(img1, img2);
				retVal = optFlowAlignment.RunOpticalFlowAlignment_OPT(false, true, false);

				tmp = imgName;
				double psnrOptFlowORB, toptflowORB;
				if (retVal == -1)
				{
					img2Aligned = img2;
					psnrOptFlowORB = -1;
					tmp.insert(0, "FAIL");
				}
				else
				{
					img2Aligned = optFlowAlignment.GetAlignedImage();
					//roi = optFlowAlignment.GetMaxRoi();
					//psnrOptFlow = GetPSNR(img1, img2Aligned, roi);
					psnrOptFlowORB = 0;
				}

				e2 = cv::getTickCount();
				toptflowORB = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-OptFlow-ORB" + "-blend.jpg";
				ss << "PSNR:" << psnrOptFlowORB << " Exe TIme:" << toptflowORB;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				/********ORB Ratio Scale*******/
				cout << "Applying orb ratio scale" << endl;
				ss.str("");
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				e1 = cv::getTickCount();
				FeatureBasedRegistration fxReg(img1, img2, 0.5);
				retVal = fxReg.CalcFeatureBasedRegNewMatch(FeatureType::fORB, MatchType::RATIO_SYMMETRY);

				tmp = imgName;
				double psnrorbratio, torbratio;
				if (retVal != 0)
				{
					img2Aligned = img2;

					if (retVal == -1)
						tmp.insert(0, "FAIL");
					else if (retVal == -2)
						tmp.insert(0, "FAIL-NoDESC");

					psnrorbratio = -1;
				}
				else
				{
					img2Aligned = fxReg.GetAlignedImage();
					//roi = fxReg.GetMaxRoi();
					//psnrorbratio = GetPSNR(img1, img2Aligned, roi);
					psnrorbratio = 0;
				}

				e2 = cv::getTickCount();

				torbratio = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-0.5" + "-ORB" + "-mRatioSymmetry" + "-blend.jpg";
				ss << "PSNR:" << psnrorbratio << " Exe TIme:" << torbratio;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				//ss << imgPsnr;
				//opFile << ss.str() + ",";
				//ss.str("");

				ss << psnrOptFlow;
				opFile << ss.str() + ",";
				ss.str("");

				ss << psnrOptFlowEG;
				opFile << ss.str() + ",";
				ss.str("");

				ss << psnrOptFlowFWBW;
				opFile << ss.str() + ",";
				ss.str("");

				ss << psnrOptFlowORB;
				opFile << ss.str() + ",";
				ss.str("");

				ss << psnrorbratio;
				opFile << ss.str() + ",";
				ss.str("");

				ss << toptflow;
				opFile << ss.str() + ",";
				ss.str("");

				ss << toptflowEG;
				opFile << ss.str() + ",";
				ss.str("");

				ss << toptflowFWBW;
				opFile << ss.str() + ",";
				ss.str("");

				ss << toptflowORB;
				opFile << ss.str() + ",";
				ss.str("");

				ss << torbratio;
				opFile << ss.str() + ",";
				ss.str("");

				opFile << "\n";
			}
			cout << "done!" << endl;
		}

		opFile.close();
	}

	int MultiImage_FinCorpus_NoPSNR2()
	{
		Mat img1, img2;

		//get all the folders
		string folderpath = "C:\\Projects\\Noctacam\\Registration\\FinCorpus\\";
		List listDir = lsfiles(folderpath);

		//remove . and ..
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));

		//write csv output file
		ofstream opFile;
		ostringstream ss;
		int64 e1, e2;

		opFile.open(folderpath + "MultiImage_FinCorpus2.csv");

		Mat avgImg;
		opFile << " ,optflow-psnr,optflow-EG-psnr,optflow-FWBW-psnr,optflow-ORB-psnr,orbratio-psnr,optflow-time,optflow-EG-time,optflow-FWBW-time,optflow-ORB-time,orbratio-time,\n";
		for (int i = 0; i < listDir.folders.size(); i++)
		{
			string newFolderPath = folderpath + listDir.folders[i] + "\\";

			//get only *.jpg
			vector<string> inputImgNames;
			List listDir1 = lsfiles(newFolderPath);

			for (int l = 0; l < listDir1.files.size(); l++)
			{
				//if (listDir1.files[l].find("jpg") != string::npos)
				if (listDir1.files[l].length() == 11)//use only 'input *.jpg' not the saved output files
				{
					inputImgNames.push_back(listDir1.files[l]);
				}
			}

			std::sort(inputImgNames.begin(), inputImgNames.end());

			img1 = imread(newFolderPath + inputImgNames[0], IMREAD_UNCHANGED);
			if (!img1.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}

			for (int j = 1; j < inputImgNames.size(); j++)
			{
				string imgName = listDir.folders[i] + "-" + inputImgNames[0] + "-" + inputImgNames[j];
				opFile << imgName + ",";
				ss.str("");

				cout << "******************************************" << endl;
				cout << "for image:" + imgName << endl;

				img2 = imread(newFolderPath + inputImgNames[j], IMREAD_UNCHANGED);
				if (!img2.data)
				{
					cout << "Could not open or find file" << endl;
					return -1;
				}

				Mat img2Aligned = Mat::zeros(img2.size(), img2.type());
				Rect roi;
				string line1, line2;

				/********optical flow orb+eigen0.1+fwbw*******/
				cout << "Applying optical flow orb+eigen0.1+fwbw algorithm" << endl;
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				e1 = cv::getTickCount();
				OpticalFlowAlignment optFlowAlignment(img1, img2);
				int retVal = optFlowAlignment.RunOpticalFlowAlignment_OPT(true, true, true);

				string tmp = imgName;
				double psnrOptFlow_orb_EG01_fwbw, toptflow_orb_EG01_fwbw;
				if (retVal == -1)
				{
					img2Aligned = img2;
					psnrOptFlow_orb_EG01_fwbw = -1;
					tmp.insert(0, "FAIL");
				}
				else
				{
					img2Aligned = optFlowAlignment.GetAlignedImage();
					psnrOptFlow_orb_EG01_fwbw = 0;
				}

				e2 = cv::getTickCount();
				toptflow_orb_EG01_fwbw = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-OptFlow_orb+eigen0_1+fwbw" + "-blend.jpg";
				ss << "PSNR:" << psnrOptFlow_orb_EG01_fwbw << " Exe TIme:" << toptflow_orb_EG01_fwbw;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				/********optical flow eigen 0.1 thresh*******/
				cout << "Applying optical flow with Eigen 0.1 algorithm" << endl;
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				e1 = cv::getTickCount();
				retVal = optFlowAlignment.RunOpticalFlowAlignment_OPT(false, false, true);

				tmp = imgName;
				double psnrOptFlowEG0_1, toptflowEG0_1;
				if (retVal == -1)
				{
					img2Aligned = img2;
					psnrOptFlowEG0_1 = -1;
					tmp.insert(0, "FAIL");
				}
				else
				{
					img2Aligned = optFlowAlignment.GetAlignedImage();
					psnrOptFlowEG0_1 = 0;
				}

				e2 = cv::getTickCount();
				toptflowEG0_1 = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-OptFlow-Eigen_0_1" + "-blend.jpg";
				ss << "PSNR:" << psnrOptFlowEG0_1 << " Exe TIme:" << toptflowEG0_1;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				/********optical flow double symmetry*******/
				cout << "Applying optical flow with double symmetry algorithm" << endl;
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				e1 = cv::getTickCount();
				retVal = optFlowAlignment.RunOpticalFlowAlignment_DS();

				tmp = imgName;
				double psnrOptFlow_DS, toptflow_DS;
				if (retVal == -1)
				{
					img2Aligned = img2;
					psnrOptFlow_DS = -1;
					tmp.insert(0, "FAIL");
				}
				else
				{
					img2Aligned = optFlowAlignment.GetAlignedImage();
					psnrOptFlow_DS = 0;
				}

				e2 = cv::getTickCount();
				toptflow_DS = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-OptFlow-DS" + "-blend.jpg";
				ss << "PSNR:" << psnrOptFlow_DS << " Exe TIme:" << toptflow_DS;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				ss << psnrOptFlow_orb_EG01_fwbw;
				opFile << ss.str() + ",";
				ss.str("");

				ss << psnrOptFlowEG0_1;
				opFile << ss.str() + ",";
				ss.str("");

				ss << psnrOptFlow_DS;
				opFile << ss.str() + ",";
				ss.str("");

				ss << toptflow_orb_EG01_fwbw;
				opFile << ss.str() + ",";
				ss.str("");

				ss << toptflowEG0_1;
				opFile << ss.str() + ",";
				ss.str("");

				ss << toptflow_DS;
				opFile << ss.str() + ",";
				ss.str("");

				opFile << "\n";
			}
			cout << "done!" << endl;
		}

		opFile.close();
	}

	int MultiImage_FinCorpus_Scale()
	{
		Mat img1, img2;

		//get all the folders
		string folderpath = "C:\\Projects\\Noctacam\\Registration\\FinCorpus\\";
		List listDir = lsfiles(folderpath);

		//remove . and ..
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), "."));
		listDir.folders.erase(find(listDir.folders.begin(), listDir.folders.end(), ".."));

		//write csv output file
		ofstream opFile;
		ostringstream ss;
		int64 e1, e2;

		opFile.open(folderpath + "MultiImage_FinCorpus2.csv");

		Mat avgImg;
		opFile << " ,optflow-psnr,optflow-EG-psnr,optflow-FWBW-psnr,optflow-ORB-psnr,orbratio-psnr,optflow-time,optflow-EG-time,optflow-FWBW-time,optflow-ORB-time,orbratio-time,\n";
		for (int i = 0; i < listDir.folders.size(); i++)
		{
			string newFolderPath = folderpath + listDir.folders[i] + "\\";

			//get only *.jpg
			vector<string> inputImgNames;
			List listDir1 = lsfiles(newFolderPath);

			for (int l = 0; l < listDir1.files.size(); l++)
			{
				//if (listDir1.files[l].find("jpg") != string::npos)
				if (listDir1.files[l].length() == 11)//use only 'input *.jpg' not the saved output files
				{
					inputImgNames.push_back(listDir1.files[l]);
				}
			}

			std::sort(inputImgNames.begin(), inputImgNames.end());

			img1 = imread(newFolderPath + inputImgNames[0], IMREAD_UNCHANGED);
			if (!img1.data)
			{
				cout << "Could not open or find file" << endl;
				return -1;
			}

			for (int j = 1; j < inputImgNames.size(); j++)
			{
				string imgName = listDir.folders[i] + "-" + inputImgNames[0] + "-" + inputImgNames[j];
				opFile << imgName + ",";
				ss.str("");

				cout << "******************************************" << endl;
				cout << "for image:" + imgName << endl;

				img2 = imread(newFolderPath + inputImgNames[j], IMREAD_UNCHANGED);
				if (!img2.data)
				{
					cout << "Could not open or find file" << endl;
					return -1;
				}

				Mat img2Aligned = Mat::zeros(img2.size(), img2.type());
				Rect roi;
				string line1, line2;

				/********optical flow with fwbw 0.5 Scale*******/
				cout << "Applying optical flow with fwbw 0.5 Scale algorithm" << endl;
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				e1 = cv::getTickCount();
				OpticalFlowAlignment optFlowAlignment(img1, img2, 0.5);
				int retVal = optFlowAlignment.RunOpticalFlowAlignment_OPT(true, false, false);

				string tmp = imgName;
				double psnrOptFlow_FWBW05, toptflow_orb_FWBW05;
				if (retVal == -1)
				{
					img2Aligned = img2;
					psnrOptFlow_FWBW05 = -1;
					tmp.insert(0, "FAIL");
				}
				else
				{
					img2Aligned = optFlowAlignment.GetAlignedImage();
					psnrOptFlow_FWBW05 = 0;
				}

				e2 = cv::getTickCount();
				toptflow_orb_FWBW05 = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-OptFlow_FWBW_0_5_Scale" + "-blend.jpg";
				ss << "PSNR:" << psnrOptFlow_FWBW05 << " Exe TIme:" << toptflow_orb_FWBW05;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				/********optical flow fwbw 0.25 scale thresh*******/
				cout << "Applying optical flow with fwbw 0.25 scale algorithm" << endl;
				img2Aligned = Mat::zeros(img2.size(), img2.type());
				avgImg = Mat::zeros(img2.size(), img2.type());

				OpticalFlowAlignment optFlowAlignment1(img1, img2, 0.25);
				e1 = cv::getTickCount();
				retVal = optFlowAlignment1.RunOpticalFlowAlignment_OPT(true, false, false);

				tmp = imgName;
				double psnrOptFlowFWBW025, toptflowFWBW025;
				if (retVal == -1)
				{
					img2Aligned = img2;
					psnrOptFlowFWBW025 = -1;
					tmp.insert(0, "FAIL");
				}
				else
				{
					img2Aligned = optFlowAlignment1.GetAlignedImage();
					psnrOptFlowFWBW025 = 0;
				}

				e2 = cv::getTickCount();
				toptflowFWBW025 = (e2 - e1) / cv::getTickFrequency();

				line1 = tmp + "-OptFlow-FWBW_0_25_Scale" + "-blend.jpg";
				ss << "PSNR:" << psnrOptFlowFWBW025 << " Exe TIme:" << toptflowFWBW025;
				line2 = ss.str();
				ss.str("");
				if (!Utils::LEANEXECUTION)
				{
					//write avg image as output
					cv::addWeighted(img1, 0.5, img2Aligned, 0.5, 0.0, avgImg);
					OverlayText(avgImg, line1, line2);
					imwrite(newFolderPath + line1, avgImg);
				}

				ss << psnrOptFlow_FWBW05;
				opFile << ss.str() + ",";
				ss.str("");

				ss << psnrOptFlowFWBW025;
				opFile << ss.str() + ",";
				ss.str("");

				ss << toptflow_orb_FWBW05;
				opFile << ss.str() + ",";
				ss.str("");

				ss << toptflowFWBW025;
				opFile << ss.str() + ",";
				ss.str("");

				opFile << "\n";
			}
			cout << "done!" << endl;
		}

		opFile.close();
	}
}