
#ifndef __LINE_H_
#define __LINE_H_


#include <array>
#include <vector>

#include <opencv2/opencv.hpp>


struct Line1
{
    std::array<float, 4>  line_endpoint;    // 线特征端点
    std::array<double, 3> line_equation;    // 线特征方程
    std::array<float, 2>  center;           // 线特征中心点

    float length;

    int numOfPixels;

    std::vector<cv::Point2f> kps;           // 关键点
    std::vector<cv::Point2f> kps_init;      // 关键点初始化
    std::vector<cv::Vec2f> dirs;            // 方向，留意这里的定义

};



#endif