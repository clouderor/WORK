#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	VideoCapture inputVideo(0);
	if (!inputVideo.isOpened())
	{
		cout << "Could not open the input video " << endl;
		return -1;
	}

	Mat frame;
	string imgname;
	int f = 1;
	while (1)
	{
		inputVideo >> frame;
		if (frame.empty())
			break;
		imshow("Camera", frame);
		char key = waitKey(1);
		if (key == 'q' || key == 'Q') // 退出运行
			break;
		if (key == 'k' || key == 'K') // 截取图片
		{
			cout << "frame:" << f << endl;
			imgname = to_string(f++) + ".jpg";
			imwrite(imgname, frame);
		}
	}
	cout << "Finished writing" << endl;
	return 0;
}
