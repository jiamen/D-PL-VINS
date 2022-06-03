#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "../parameters.h"
#include "../utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f& pt);

void reduceVector(vector<cv::Point2f>& v, vector<uchar> status);
void reduceVector(vector<int>& v, vector<uchar> status);


class FeatureTracker
{
public:
    FeatureTracker();

    // void readImage(const cv::Mat& _img);         // zlc添加：PL-VINS中原函数
    void readImage(const cv::Mat &_img, const cv::Mat &_depth, double _cur_time);   // △△△△△ 添加深度 △△△△△
    void readImage(const cv::Mat &_img, const cv::Mat &_depth, double _cur_time, Matrix3d relative_R);
                                                                                    // △△△△△ FAST添加IMU位姿 △△△△△

    void setMask();

    void addPoints();
    void addPoints(int n_max_cnt);                  // zlc添加：FAST中使用的添加点函数

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string& calib_file);

    void showUndistortion(const string& name);
    void showUndistortion();

    void rejectWithF();

    void undistortedPoints();                           // △ zlc添加：RGBD中函数
    // vector<cv::Point2f> undistortedPoints();         // △ zlc添加：PL-VINS中函数

    void predictPtsInNextFrame(Matrix3d relative_R);    // △ zlc添加：FAST中函数
    Eigen::Vector3d get3dPt(const cv::Mat &_depth, const cv::Point2f &pt);      // △ zlc添加：FAST中函数


    cv::Mat mask;
    cv::Mat mask_exp;                                   // △△△△△ zlc： FAST中新增加的深度值 △△△△△
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img; //  prev : i-1 时刻，  cur: i 时刻， forw： i+1时刻
    cv::Mat prev_depth, cur_depth, forw_depth;          // △△△△△ zlc： 新增加的深度值 △△△△△


    vector<cv::Point2f> n_pts;                      // 存放补充提取的点，比如150点，跟踪之后当前帧还有100点，那么再提取50个点进行补充
    vector<cv::KeyPoint> Keypts;                    // △△ zlc添加：FAST中添加的变量，保存关键点
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;// cur_pts上一帧的点坐标，forw_pts当前帧的点坐标

    vector<cv::Point2f> predict_pts, unstable_pts;  // △△ zlc添加：FAST中添加的变量

    vector<cv::Point2f> prev_un_pts, cur_un_pts;    // △ zlc添加：cur_un_pts：最新帧的相机系 归一化坐标
    vector<cv::Point2f> pts_velocity;               // △ zlc添加


    vector<int> ids;                                //  每个特征点的id
    vector<int> track_cnt;                          //  记录某个特征已经跟踪多少帧了，即被多少帧看到了


    map<int, cv::Point2f> cur_un_pts_map;           // △△△△△ PL-VINS中去除了这两个变量 △△△△△
    map<int, cv::Point2f> prev_un_pts_map;          // △△△△△ PL-VINS中去除了这两个变量 △△△△△

    camodocal::CameraPtr m_camera;
    double cur_time;                                // △ zlc添加：当前帧时间
    double prev_time;                               // △ zlc添加：上一帧时间

    static int n_id;


    bool hasPrediction = false;
    cv::Ptr<cv::FastFeatureDetector> p_fast_feature_detector;
};
