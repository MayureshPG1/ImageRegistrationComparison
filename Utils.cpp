#include "Utils.h"

//namespace Utils
//{
using namespace std;
using namespace cv;

//double Utils::GetPSNR(const Mat& I1, const Mat& I2)
//{
//	Mat s1;
//	absdiff(I1, I2, s1);       // |I1 - I2|
//	s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
//	s1 = s1.mul(s1);           // |I1 - I2|^2
//
//	Scalar s = sum(s1);         // sum elements per channel
//
//	double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels
//
//	if (sse <= 1e-10) // for small values return zero
//		return 0;
//	else
//	{
//		double  mse = sse / (double)(I1.channels() * I1.total());
//		double psnr = 10.0*log10((255 * 255) / mse);
//		return psnr;
//	}
//}
//
//double TestRuns::GetSSIM(const Mat& i1, const Mat& i2)
//{
//	const double C1 = 6.5025, C2 = 58.5225;
//	/***************************** INITS **********************************/
//	int d = CV_32F;
//
//	Mat I1, I2;
//	i1.convertTo(I1, d);           // cannot calculate on one byte large values
//	i2.convertTo(I2, d);
//
//	Mat I2_2 = I2.mul(I2);        // I2^2
//	Mat I1_2 = I1.mul(I1);        // I1^2
//	Mat I1_I2 = I1.mul(I2);        // I1 * I2
//
//								   /***********************PRELIMINARY COMPUTING ******************************/
//
//	Mat mu1, mu2;   //
//	GaussianBlur(I1, mu1, Size(11, 11), 1.5);
//	GaussianBlur(I2, mu2, Size(11, 11), 1.5);
//
//	Mat mu1_2 = mu1.mul(mu1);
//	Mat mu2_2 = mu2.mul(mu2);
//	Mat mu1_mu2 = mu1.mul(mu2);
//
//	Mat sigma1_2, sigma2_2, sigma12;
//
//	GaussianBlur(I1_2, sigma1_2, Size(11, 11), 1.5);
//	sigma1_2 -= mu1_2;
//
//	GaussianBlur(I2_2, sigma2_2, Size(11, 11), 1.5);
//	sigma2_2 -= mu2_2;
//
//	GaussianBlur(I1_I2, sigma12, Size(11, 11), 1.5);
//	sigma12 -= mu1_mu2;
//
//	///////////////////////////////// FORMULA ////////////////////////////////
//	Mat t1, t2, t3;
//
//	t1 = 2 * mu1_mu2 + C1;
//	t2 = 2 * sigma12 + C2;
//	t3 = t1.mul(t2);              // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))
//
//	t1 = mu1_2 + mu2_2 + C1;
//	t2 = sigma1_2 + sigma2_2 + C2;
//	t1 = t1.mul(t2);               // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))
//
//	Mat ssim_map;
//	divide(t3, t1, ssim_map);      // ssim_map =  t3./t1;
//
//	Scalar mssim = mean(ssim_map); // mssim = average of ssim map
//								   //return average ssim across channels
//	double avgSSIM = (mssim.val[0] + mssim.val[1] + mssim.val[2]) / 3.0;
//	return avgSSIM;
//}


//}
//
////All of the hard work
//List lsfiles(string folder)  //(c) http://stackoverflow.com/a/20847429/1009816
//{
//	vector<string> files; //Will be added to List
//	vector<string> folders; //Will be added to List
//	char search_path[200];
//	sprintf_s(search_path, "%s*.*", folder.c_str());
//	WIN32_FIND_DATA fd;
//	HANDLE hFind = ::FindFirstFile(search_path, &fd);
//	if (hFind != INVALID_HANDLE_VALUE)
//	{
//		do
//		{
//			// read all (real) files in current folder, delete '!' read other 2 default folder . and ..
//			if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
//			{
//
//				files.push_back(fd.cFileName);
//			}
//			else //Put folders into vector
//			{
//				folders.push_back(fd.cFileName);
//			}
//		} while (::FindNextFile(hFind, &fd));
//		::FindClose(hFind);
//	}
//	List me;
//	me.files = files;
//	me.folders = folders;
//
//	return me;
//}