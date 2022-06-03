#ifndef _FEATURE_MANAGER_H_
#define _FEATURE_MANAGER_H_


#pragma once


// std
#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;


// ros
#include <ros/console.h>
#include <ros/assert.h>


// self
#include "frontend/frontend_data.h"
#include "parameters.h"
#include "utility/line_geometry.h"



extern Vector2d image_uv1;

class FeaturePerFrame
{
public:
    FeaturePerFrame(const Vector3d& _point)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
    }
    // FeaturePerFrame(const Eigen::Matrix<double, 5, 1>& _point)
    FeaturePerFrame(const Eigen::Matrix<double, 6, 1>& _point)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        depth  = _point(5);
        // std::cout<<"00000000 = "<<uv<<std::endl;
    }

    double cur_td;          // △ zlc添加：时延相关参数 △
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;      // △ zlc添加：时延相关参数 △
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;

    double depth;           // △△△△△ 这里添加了深度值 △△△△△
};


class FeaturePerId
{
public:
    const int feature_id;
    int start_frame;            // zlc：这个与下面的有什么不同？？？
    vins::TimeFrameId kf_id;    // zlc：host frame id  主帧号

    
    //  feature_per_frame 是个向量容器，存着这个特征在每一帧上的观测量。 
    //                    如：feature_per_frame[0]，存的是ft在start_frame上的观测值; feature_per_frame[1]存的是start_frame+1上的观测
    vector<FeaturePerFrame> feature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    bool is_plane_point = false;        // △△ zlc添加：判断该点是否为地面点 △△

    double estimated_depth;
    int solve_flag;         // 0 haven't solve yet; 1 solve succ; 2 solve fail;



    // △△△△△ 下面的两个变量是新添加的 △△△△△
    double measured_depth;          // △ zlc添加
    int estimate_flag;              // △ zlc添加： 0 initial; 1 by depth image; 2 by triangulate

    Vector3d gt_p;

    // FeaturePerId(int _feature_id, int _start_frame)
    FeaturePerId(int _feature_id, int _start_frame, double _measured_depth)     // △△△△△ 这里添加了深度值 △△△△△
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0),
          estimated_depth(-1.0),
          measured_depth(_measured_depth),  // △ zlc添加 △
          solve_flag(0),
          estimate_flag(0)                  // △ zlc添加 △
    {

    }

    // △△△△△ zlc添加：本类的构造函数有改动 △△△△△
    /*
    FeaturePerId(int _feature_id, int _start_frame, double _measured_depth)     // △△△△△ 这里添加了深度值 △△△△△
            : feature_id(_feature_id), start_frame(_start_frame),
              used_num(0), estimated_depth(-1.0), measured_depth(_measured_depth), solve_flag(0), estimate_flag(0)
    {
    }
    */

    int endFrame();
};


/**
 *   PL-VINS新增的类，添加了线特征
 **/
class lineFeaturePerFrame
{
public:
    lineFeaturePerFrame(const Vector4d& line)
    {
        lineobs = line;
    }

    lineFeaturePerFrame(const Eigen::Matrix<double, 6, 1>& line)
    {
        lineobs_depth = line;                       // △ zlc添加：起始点归一化值+深度值
    }
    lineFeaturePerFrame(const Vector8d& line)
    {
        lineobs   = line.head<4>();
        lineobs_R = line.tail<4>();
    }
    Vector4d lineobs;                               // 每一帧上的观测 O=(ψ, φ)^T 角度表示
    Eigen::Matrix<double, 6, 1> lineobs_depth;      // △ zlc添加：观测点与深度值
    Vector4d lineobs_R;
    Eigen::Matrix<double, 6, 1> lineobs_R_depth;      // △ zlc添加：观测点与深度值
    double z;
    bool   is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};


class lineFeaturePerId
{
public:
    const int feature_id;
    int start_frame;

    //  feature_per_frame 是个向量容器，存着这个特征在每一帧上的观测量。
    //                    如：feature_per_frame[0]，存的是ft在start_frame上的观测值; feature_per_frame[1]存的是start_frame+1上的观测
    vector<lineFeaturePerFrame> linefeature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    bool is_triangulation;
    Vector6d line_plucker;

    Vector4d obs_init;
    Vector4d obs_j;
    Vector6d line_plk_init; // used to debug
    Vector3d ptw1;          // used to debug
    Vector3d ptw2;          // used to debug
    Eigen::Vector3d tj_;    // tij
    Eigen::Matrix3d Rj_;
    Eigen::Vector3d ti_;    // tij
    Eigen::Matrix3d Ri_;
    int removed_cnt;
    int all_obs_cnt;        // 总共观测多少次了？

    int solve_flag;         // 0 haven't solve yet; 1 solve succ; 2 solve fail;


    double measured_startpt_depth;
    double measured_endpt_depth;
    int estimate_flag;      // 0 initial; 1 by depth image; 2 by triangulate

    double estimated_startpt_depth;
    double estimated_endpt_depth;

    lineFeaturePerId(int _feature_id, int _start_frame, double _measured_startpt_depth, double _measured_endpt_depth)
            : feature_id(_feature_id), start_frame(_start_frame),
              used_num(0),
              measured_startpt_depth(_measured_startpt_depth),
              measured_endpt_depth(_measured_endpt_depth),
              estimated_startpt_depth(-1),
              estimated_endpt_depth(-1),
              solve_flag(0),
              estimate_flag(0),
              is_triangulation(false)
    {
        removed_cnt = 0;
        all_obs_cnt = 1;
    }

    lineFeaturePerId(int _feature_id, int _start_frame)
            : feature_id(_feature_id), start_frame(_start_frame),
              used_num(0),
              solve_flag(0),
              is_triangulation(false)
    {
        removed_cnt = 0;
        all_obs_cnt = 1;
    }


    int endFrame();
};


class FeatureManager
{
public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);

    void clearState();

    int getFeatureCount();
    int getLineFeatureCount();

    MatrixXd getLineOrthVector(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void setLineOrth(MatrixXd x, Vector3d Ps[], Matrix3d Rs[],Vector3d tic[], Matrix3d ric[]);
    MatrixXd getLineOrthVectorInCamera();
    void setLineOrthInCamera(MatrixXd x);


    double reprojection_error( Vector4d obs, Matrix3d Rwc, Vector3d twc, Vector6d line_w );
    void removeLineOutlier(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void removeLineOutlier();


    // mono point
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Vector3d>>>& image);
    // mono line
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Vector3d>>>& image,
                                 const map<int, vector<pair<int, Vector4d>>>& lines);


    // bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Matrix<double, 5, 1>>>> &image, const map<int, vector<pair<int, Vector4d>>> &lines);
    bool addFeatureCheckParallax(int frame_count,
                                 const map<int, vector<pair<int, Matrix<double, 6, 1>>>>& image,
                                 const map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>>& lines);
    bool addFeatureCheckParallax(int frame_count,
                                 const map<int, vector<pair<int, Matrix<double, 6, 1>>>>& image);

    // stereo line
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Vector3d>>>& image,
                                 const map<int, vector<pair<int, Vector8d>>>& lines);


    void debugShow();

    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    vector<pair<Vector3d, Vector3d>> getCorrespondingWithDepth(int frame_count_l, int frame_count_r);   // △△△△△ zlc新添加的函数 △△△△△


    // △△△△△ zlc添加：将点分为平面点和非平面点 △△△△△
    void planePointClassify(double* T_w_i_ptr,
                            double* T_i_c_ptr,
                            double plane_d_w);
    int planePointClassify(Eigen::Vec3d Ps[],
                           Eigen::Vec3d tic[],
                           Matrix3d ric[],
                           double plane_d_w);



    // void updateDepth(const VectorXd& x);
    void setDepth(const VectorXd& x);
    void removeFailures();
    void clearDepth(const VectorXd& x);
    VectorXd getDepthVector();

    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void triangulateWithDepth(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);       // △△△△△ 已有深度值的情况下的三角化 △△△△△
    void triangulateLine(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void triangulateLine(double baseline);      // stereo line

    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier();

    list<FeaturePerId> feature;                 // 所有特征点的序号
    list<lineFeaturePerId> linefeature;         // 所有特征线的序号
    int last_track_num;

public:
    // zlc添加的改写内容：estimator interface
    std::set<double>* local_activate_frames_ = nullptr;
    std::map<int, double>* int_frameid2_time_frameid_ = nullptr;    // 帧号到时间戳
    std::map<double, int>* time_frameid2_int_frameid_ = nullptr;    // 时间戳到帧号




private:
    double compensatedParallax2(const FeaturePerId& it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[NUM_OF_CAM];
};

#endif