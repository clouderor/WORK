// object_recognition.h
#ifndef OBJECT_RECOGNITION_H
#define OBJECT_RECOGNITION_H

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class ObjectRecognition {
public:
    ObjectRecognition() {}

    // 识别物体并绘制轮廓，打印信息的函数
    void recognize(cv::Mat& image, double length, double lenpbcol);

    // 计算轮廓内的平均 BGR 颜色
    Scalar getAverageBGRColor(const cv::Mat& image, const std::vector<cv::Point>& poly);

    // 计算轮廓内的平均 HSV 颜色
    Scalar getAverageHSVColor(const cv::Mat& image, const std::vector<cv::Point>& poly);

private:
    int iLowH=0; int iHighH=0; int iLowS=0; int iHighS=0; int iLowV=0; int iHighV=0;
};
#endif // OBJECT_RECOGNITION_H