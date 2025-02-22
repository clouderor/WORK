#include<opencv2/opencv.hpp>
#include<C:/Users/zwy/source/repos/CMakeProject1/include/CMakeProject1.h>

cv::Scalar ObjectRecognition::getAverageBGRColor(const cv::Mat& image, const std::vector<cv::Point>& poly) {
    cv::Rect rect = cv::boundingRect(poly);
    cv::Mat roi = image(rect);
    cv::Scalar meanColor = cv::mean(roi);
    return meanColor;
}

// 计算轮廓内的平均 HSV 颜色
cv::Scalar ObjectRecognition::getAverageHSVColor(const cv::Mat& image, const std::vector<cv::Point>& poly) {
    cv::Rect rect = cv::boundingRect(poly);
    cv::Mat roi = image(rect);
    cv::Mat hsvRoi;
    cv::cvtColor(roi, hsvRoi, cv::COLOR_BGR2HSV);
    cv::Scalar meanColor = cv::mean(hsvRoi);
    return meanColor;
}