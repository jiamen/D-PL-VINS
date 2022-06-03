#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;



const int LINE_MIN_OBS = 5;             // △  线特征最少被多少帧看到
const double LOOP_INFO_VALUE = 50.0;
// #define DEPTH_PRIOR
// #define GT
// #define UNIT_SPHERE_ERROR            // △  zlc注释



extern double BASE_LINE;


extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string VINS_FOLDER_PATH;


extern int LOOP_CLOSURE;
extern int MIN_LOOP_NUM;
extern int MAX_KEYFRAME_NUM;
extern std::string PATTERN_FILE;
extern std::string VOC_FILE;

// extern std::string CAM_NAMES;
extern std::vector<std::string> CAM_NAMES; // △△△△△ zlc: 这里按照双目来写 △△△△△
extern std::string IMAGE_TOPIC;
extern std::string DEPTH_TOPIC;            // △△△△△ zlc: FAST中新添加的 △△△△△
extern std::string IMU_TOPIC;

extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;


// △△△△△ zlc：下列代码都是视觉跟踪中添加的 △△△△△
extern int IMAGE_SIZE;

extern double DEPTH_MIN_DIST;
extern double DEPTH_MAX_DIST;
extern int MAX_CNT;
extern int MAX_CNT_SET;
extern int MIN_DIST;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern std::string FISHEYE_MASK;
extern int STEREO_TRACK;
extern bool PUB_THIS_FRAME;

extern std::vector<std::string> SEMANTIC_LABEL;
extern std::vector<std::string> STATIC_LABEL;
extern std::vector<std::string> DYNAMIC_LABEL;
// △△△△△ zlc：上述代码都是视觉跟踪中添加的 △△△△△



void readParameters(ros::NodeHandle& n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1,
    SIZE_LINE = 4
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
