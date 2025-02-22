#include<opencv2/opencv.hpp>
#include<iostream>
#include<C:/Users/zwy/source/repos/CMakeProject1/include/CMakeProject1.h>
#include<math.h>

using namespace std;
using namespace cv;

void ObjectRecognition::recognize(cv::Mat& image, double length, double lenpbcol) {
    double proportion;//每一像素所占高度  mm
    //设置待测物体长宽比
    bool flag = true;
    //滑块调整识别颜色阈值 
    createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &iHighH, 179);

    createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &iHighS, 255);

    createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    createTrackbar("HighV", "Control", &iHighV, 255);
    
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH,iHighS,iHighV), mask);
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask, mask, MORPH_OPEN, element);
    morphologyEx(mask, mask, MORPH_CLOSE, element);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    ObjectRecognition obj;
    for (size_t i = 0; i < contours.size(); i++) {
        cv::Rect rect = cv::boundingRect(contours[i]);
        cv::rectangle(image, rect, cv::Scalar(0, 255, 0), 2);
        std::vector<cv::Point> approx;
        Point2d picture, center_img;
        cv::approxPolyDP(contours[i], approx, 3, true);
        if (approx.size() == 4&&flag == true) {
            double perimeter = cv::arcLength(contours[i], true);
            double area = cv::contourArea(contours[i]);
            cv::Scalar bgrColor = obj.getAverageBGRColor(image, approx);
            cv::Scalar hsvColor = obj.getAverageHSVColor(image, approx);
            std::vector<cv::Point2d> poly;
            for (size_t j = 0; j < approx.size(); j++) {
                cv::circle(image, approx[j], 5, cv::Scalar(0, 0, 255), -1);
                //像素坐标
                std::cout << "像素坐标：Corner " << j << " : (" << approx[j].x << ", " << approx[j].y << ")" << std::endl;
                poly.push_back(cv::Point2d(approx[j].x, approx[j].y));
                picture.x = approx[j].x + picture.x;
                picture.y = approx[j].y + picture.y;
            }
            //图像坐标 
            center_img.x = -(picture.x / 4 - 329) / 663.1414;
            center_img.y = -(picture.y / 4 - 235) / 640.3547;
            cout << "图像坐标：" << center_img << endl;

            // 3D 特征点世界坐标，与像素坐标对应，单位是mm
            std::vector<Point3d> model_points;
            proportion = length / (approx[0].y - approx[2].y);

            model_points.push_back(Point3d((approx[0].x - 329) * proportion, (approx[0].y - 235) * proportion, 0));
            model_points.push_back(Point3d((approx[1].x - 329) * proportion, (approx[1].y - 235) * proportion, 0));
            model_points.push_back(Point3d((approx[2].x - 329) * proportion, (approx[2].y - 235) * proportion, 0));
            model_points.push_back(Point3d((approx[3].x - 329) * proportion, (approx[3].y - 235) * proportion, 0));
            cout << "X,Y" << (model_points[0] + model_points[1] + model_points[2] + model_points[3]) / 4 << endl;
            // 相机内参矩阵
            Mat camera_matrix = (Mat_<double>(3, 3) << 663.1414, 0, 329.6919,
                0, 640.3547, 234.8499,
                0, 0, 1);
            // 相机畸变系数
            Mat dist_coeffs = (Mat_<double>(5, 1) << 0.0109, 1.0312,
                -6.6990, -0.0011, 2.2860);

            //cout << "Camera Matrix " << endl << camera_matrix << endl << endl;
            // 旋转向量
            Mat rotation_vector;
            // 平移向量
            Mat translation_vector;

            // pnp求解
            solvePnP(model_points, poly, camera_matrix, dist_coeffs,
                rotation_vector, translation_vector, 0, SOLVEPNP_ITERATIVE);

            //cout << "Rotation Vector " << endl << rotation_vector << endl << endl;
            //cout << "Translation Vector" << endl << translation_vector << endl << endl;

            Mat Rvec;
            Mat_<float> Tvec;
            rotation_vector.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
            translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式 

            Mat_<float> rotMat(3, 3);
            Rodrigues(Rvec, rotMat);
            // 旋转向量转成旋转矩阵
            //cout << "rotMat" << endl << rotMat << endl << endl;

            Mat P_oc;
            P_oc = -rotMat.inv() * Tvec;
            // 求解相机的世界坐标
            cout << "P_oc" << endl << P_oc << endl;
            //std::cout << "Perimeter: " << perimeter << std::endl;
            //std::cout << "Area: " << area << std::endl;
            //std::cout << "Average BGR Color (B, G, R): (" << bgrColor[0] << ", " << bgrColor[1] << ", " << bgrColor[2] << ")" << std::endl;
            //std::cout << "Average HSV Color (H, S, V): (" << hsvColor[0] << ", " << hsvColor[1] << ", " << hsvColor[2] << ")" << std::endl;
        }
    }
}