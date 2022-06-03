
#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>


#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

#include "parameters.h"
#include "tic_toc.h"

// #include <opencv2/line_descriptor.hpp>       // △ zlc添加：不用库函数实验了
#include <opencv2/features2d.hpp>

#include "opencv2/ximgproc.hpp"                 // △ zlc添加：测试EDLines
#include "opencv2/imgcodecs.hpp"                // △ zlc添加：测试EDLines


#include "./line_descriptor/include/line_descriptor_custom.hpp"
#include "./EDLines/include/edline_detector.h"

using namespace cv::line_descriptor;
using namespace std;
using namespace cv;
using namespace camodocal;
using namespace cv::ximgproc;                   // △ zlc添加：测试EDLines



// 代码中给线特征很多属性，仅有两个端点的坐标和线段长度被用到，
// 在运行LineFeatureTracker::readImage(const cv::Mat& _img)读取图片并提取线段时，
// 将由LSD库的线段检测算子所提取的KeyLine类线特征转换成本框架下所定义的 Line类线特征，并赋值端点坐标和线段长度。
struct Line
{
	Point2f StartPt;
    Point3f StartPt_depth;          // △ zlc添加
	Point2f EndPt;
    Point3f EndPt_depth;            // △ zlc添加

    float lineWidth;
	Point2f Vp;

	Point2f Center;
    Point3f Center_depth;            // △ zlc添加
	Point2f unitDir;                // [cos(theta), sin(theta)]
	float length;
	float theta;

	// para_a * x + para_b * y + c = 0
	float para_a;
	float para_b;
	float para_c;

	float image_dx;
	float image_dy;
    float line_grad_avg;

	float xMin;
	float xMax;
	float yMin;
	float yMax;
	unsigned short id;
	int colorIdx;
};


// 包括一帧图像中所提取出的所有KeyLine线特征和对应的LBD描述子，以及转换后的Line线特征及它们对应的ID号
class FrameLines
{
public:
    int frame_id;
    Mat img;
    
    vector<Line> vecLine;
    vector< int > lineID;

    // opencv3 lsd+lbd
    std::vector<KeyLine> keylsd;
    Mat lbd_descr;
};
typedef shared_ptr< FrameLines > FrameLinesPtr;


// 线特征跟踪
class LineFeatureTracker
{
public:
    LineFeatureTracker();

    void readIntrinsicParameter(const string& calib_file);
    void NearbyLineTracking(const vector<Line> forw_lines, const vector<Line> cur_lines, vector<pair<int, int> >& lineMatches);

    vector<Line> undistortedLineEndPoints();

    void readImage(const cv::Mat& _img);

    FrameLinesPtr curframe_, forwframe_;

    cv::Mat undist_map1_, undist_map2_ , K_;

    camodocal::CameraPtr m_camera;       // pinhole camera

    int frame_cnt;
    vector<int> ids;                     // 每个特征点的id
    vector<int> linetrack_cnt;           // 记录某个特征已经跟踪多少帧了，即被多少帧看到了
    int allfeature_cnt;                  // 用来统计整个地图中有了多少条线，它将用来赋值

    double sum_time;
    double mean_time;
};
