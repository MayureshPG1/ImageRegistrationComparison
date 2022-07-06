#include "TestRuns.h"

using namespace cv;
using namespace std;

int main(void)
{
	try
	{
		TestRuns::MultiImage_FinCorpus_MultiMaxBlend();
	}
	catch (Exception e)
	{
		cout << "Exception occured!!!";
		cout << e.what() << endl;
	}

	system("pause");
}

