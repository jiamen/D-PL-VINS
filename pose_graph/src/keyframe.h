#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include "parameters.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"

#define MIN_LOOP_NUM 25

using namespace Eigen;
using namespace std;
using namespace DVision;


class BriefExtractor
{
public:
    virtual void operator()(const cv::Mat& im, vector<cv::KeyPoint>& keys, vector<BRIEF::bitset>& descriptors) const;
    BriefExtractor(const std::string& pattern_file);

    DVision::BRIEF m_brief;
};

class KeyFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // △△△△△ RGBD新添加的 △△△△△
	KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
			 vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_normal, 
			 vector<double> &_point_id, int _sequence);


    // △ zlc：这里有两个关键帧的构建函数：这里的构造函数在process()中应用
    KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
             vector<cv::Point3f> &_point_3d_depth,
             vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_normal,
             vector<double> &_point_id, int _sequence);         // △△△△△ 新添加的深度变量 △△△△△

    // △ zlc：这里的构造函数在loadPoseGraph()函数中被调用
    KeyFrame(double _time_stamp, int _index, Vector3d& _vio_T_w_i, Matrix3d& _vio_R_w_i, Vector3d& _T_w_i, Matrix3d& _R_w_i,
			 cv::Mat& _image, int _loop_index, Eigen::Matrix<double, 8, 1 >& _loop_info,
			 vector<cv::KeyPoint>& _keypoints, vector<cv::KeyPoint>& _keypoints_norm, vector<BRIEF::bitset>& _brief_descriptors);

    bool findConnection(KeyFrame* old_kf);
	void computeWindowBRIEFPoint();
	void computeBRIEFPoint();
	// void extractBrief();
	int HammingDis(const BRIEF::bitset& a, const BRIEF::bitset& b);
	bool searchInAera(const BRIEF::bitset window_descriptor,
	                  const std::vector<BRIEF::bitset>& descriptors_old,
	                  const std::vector<cv::KeyPoint>& keypoints_old,
	                  const std::vector<cv::KeyPoint>& keypoints_old_norm,
	                  cv::Point2f& best_match,
	                  cv::Point2f& best_match_norm);
	void searchByBRIEFDes(std::vector<cv::Point2f>& matched_2d_old,
						  std::vector<cv::Point2f>& matched_2d_old_norm,
                          std::vector<uchar>& status,
                          const std::vector<BRIEF::bitset>& descriptors_old,
                          const std::vector<cv::KeyPoint>& keypoints_old,
                          const std::vector<cv::KeyPoint>& keypoints_old_norm);
	void FundmantalMatrixRANSAC(const std::vector<cv::Point2f>& matched_2d_cur_norm,
                                const std::vector<cv::Point2f>& matched_2d_old_norm,
                                vector<uchar>& status);
	void PnPRANSAC(const vector<cv::Point2f>& matched_2d_old_norm,
	               const std::vector<cv::Point3f>& matched_3d,
	               std::vector<uchar>& status,
	               Eigen::Vector3d& PnP_T_old, Eigen::Matrix3d& PnP_R_old);
	void getVioPose(Eigen::Vector3d& _T_w_i, Eigen::Matrix3d& _R_w_i);
	void getPose(Eigen::Vector3d& _T_w_i, Eigen::Matrix3d& _R_w_i);
	void updatePose(const Eigen::Vector3d& _T_w_i, const Eigen::Matrix3d& _R_w_i);
	void updateVioPose(const Eigen::Vector3d& _T_w_i, const Eigen::Matrix3d& _R_w_i);
	void updateLoop(Eigen::Matrix<double, 8, 1 >& _loop_info);

	Eigen::Vector3d getLoopRelativeT();
	double getLoopRelativeYaw();
	Eigen::Quaterniond getLoopRelativeQ();



	double time_stamp; 
	int index;
	int local_index;
	Eigen::Vector3d vio_T_w_i; 
	Eigen::Matrix3d vio_R_w_i; 
	Eigen::Vector3d T_w_i;
	Eigen::Matrix3d R_w_i;
	Eigen::Vector3d origin_vio_T;	        // 原始VIO结果的位姿
	Eigen::Matrix3d origin_vio_R;

    cv::Mat image;
	cv::Mat thumbnail;
	vector<cv::Point3f> point_3d; 
	vector<cv::Point2f> point_2d_uv;
	vector<cv::Point2f> point_2d_norm;
    vector<cv::Point3f> point_3d_depth;     // △△△△△ zlc新添加的深度值 △△△△△

	vector<double> point_id;
	vector<cv::KeyPoint> keypoints;         // fast角点的像素坐标
	vector<cv::KeyPoint> keypoints_norm;    // fast角点对应的归一化相机系坐标
	vector<cv::KeyPoint> window_keypoints;
	vector<BRIEF::bitset> brief_descriptors;            // 额外提取的fast特征点的描述子
	vector<BRIEF::bitset> window_brief_descriptors;     // 原来光流追踪的特征点的描述子
	bool has_fast_point;
	int sequence;


	bool has_loop;
	int loop_index;

	Eigen::Matrix<double, 8, 1 > loop_info;
    // 回环确定：特征检测部分通过匹配子检测后的，再通过PnP检测后，将确定此部分回环信息
    // loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
    //	    	    relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
    //	    	    relative_yaw;
};

