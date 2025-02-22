#include <opencv2/opencv.hpp>
#include<C:/Users/zwy/source/repos/CMakeProject1/include/CMakeProject1.h>
#include <math.h>

using namespace std;
using namespace cv;

int main()
{
	double length;//单位为mm 
	double lenpbcol;
	cout << "请输入待测物体高度：" << endl;
	cin >> length;
	cout << "请输入待测物体长宽比：" << endl;
	cin >> lenpbcol;
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		cerr << "Can not open the camera." << endl;
		return -1;
	}
	ObjectRecognition obj;
	Mat frame;
	Mat mirror;
	while (true) {
		cap >> frame;
		if (frame.empty()) {
			cerr << "Failed to capture frame." << endl;
			break;
		}
		obj.recognize(frame,length,lenpbcol);
		flip(frame, mirror, 1);
		imshow("Output",mirror);
		if (waitKey(1) == 27)break;
	}
	cap.release();
	destroyAllWindows();
	return 0;
}
