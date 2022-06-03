#pragma once


#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"


#include "./feature_tracker/feature_tracker.h"        // △△△△ zlc新添加的特征跟踪头文件 △△△

// ros
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

// std
#include <unordered_map>
#include <queue>

// opencv
#include <opencv2/core/eigen.hpp>


#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/marginalization_factor.h"

#include "factor/line_parameterization.h"
#include "factor/line_projection_factor.h"


#include "factor/projection_plane_factor.h"     // 地面约束添加的头文件
#include "factor/relativepose_factor.h"         // 地面约束添加的头文件


// 打印格式相关头文件
#include "utility/logging.h"



struct RetriveData
{
    /* data */
    int old_index;
    int cur_index;
    double header;

    Vector3d P_old;
    Matrix3d R_old;
    vector<cv::Point2f> measurements;
    vector<int> features_ids; 
    bool relocalized;
    bool relative_pose;
    Vector3d relative_t;
    Quaterniond relative_q;
    double relative_yaw;
    double loop_pose[7];
};


class Estimator
{
public:
    Estimator();

    void setParameter();
    void setParameter1();

    // interface
    void processIMU(double t, const Vector3d& linear_acceleration, const Vector3d& angular_velocity);

    void processImage(const map<int, vector<pair<int, Vector3d>>>& image,
                      const map<int, vector<pair<int, Vector4d>>>& lines,
                      const std_msgs::Header& header);

    void processImage1(const map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>>& image,
                      const std_msgs::Header& header);

    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>>& image,
                      const map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>>& lines,
                      const std_msgs::Header& header);

    void processImage(const map<int, vector<pair<int, Vector3d>>>& image, const map<int, vector<pair<int, Vector8d>>>& lines, const std_msgs::Header& header);
    void processImage(const map<int, vector<pair<int, Vector3d>>>& image, const std_msgs::Header &header);

    // internal
    void clearState();                              // Estimator 重置

    // visual and imu initial
    bool initialStructure();
    bool visualInitialAlign();
    bool visualInitialAlignWithDepth();             // △△△△△ 有深度信息后的初始化 △△△△△
    bool relativePose(Matrix3d& relative_R, Vector3d& relative_T, int& l);

    void slideWindow();
    void solveOdometry();
    void solveOdometry1();
    void slideWindowNew();
    void slideWindowOld();

    void optimization();
    void optimizationwithLine();
    void onlyLineOpt();

    void LineBA();
    void LineBAincamera();

    void vector2double();
    void double2vector();
    void double2vector2();

    bool failureDetection();

    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d>& _match_points, Vector3d _relo_t, Matrix3d _relo_r);

    // 梳理局部滑窗特征点的id
    void recomputeFrameId();


    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };


    // FeatureTracker featureTracker;              // △△△△△ zlc添加：前端特征跟踪类 △△△△△
    FeatureTracker trackerData[NUM_OF_CAM];     // △△△△△ zlc添加：前端特征跟踪类 △△△△△


    double frame_cnt_ = 0;
    double sum_solver_time_  = 0.0;
    double mean_solver_time_ = 0.0;
    double sum_marg_time_  = 0.0;
    double mean_marg_time_ = 0.0;


    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

    // extrinsic 外参，相机的内外参
    Matrix3d ric[NUM_OF_CAM];
    Vector3d tic[NUM_OF_CAM];


    // VIO state vector 滑窗 PVQB
    Vector3d Ps[(WINDOW_SIZE + 1)];     // Ps 位移
    Vector3d Vs[(WINDOW_SIZE + 1)];     // Vs 速度
    Matrix3d Rs[(WINDOW_SIZE + 1)];     // Rs 旋转
    Vector3d Bas[(WINDOW_SIZE + 1)];    // Bas
    Vector3d Bgs[(WINDOW_SIZE + 1)];    // Bgs
    double   td;                        // △ zlc添加回来的


    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];


    // 预积分值
    IntegrationBase* pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;                    // 关键帧数量（计数）
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    // 特征点管理器
    FeatureManager f_manager;                       // zlc：包含所有点特征和线特征
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    static constexpr double prior_weight = 1e8;     // zlc：地面检测添加
    double plane_d_w_detect;                        // zlc：地面检测添加
    bool   bapply_prior = true;                     // zlc：地面检测添加
    std::array<double, 1> param_plane{};            // zlc：地面检测添加
    bool b_plane_init_success = false;              // zlc：地面检测添加


    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;



    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double baseline_;
    // 用于优化的临时变量
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];               // 对应P跟R
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];     // 对应上面的V和Bas、Bgs
    double para_Feature[NUM_OF_F][SIZE_FEATURE];                // 对应特征点的逆深度 λ

    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];                 // camera与imu的外参
    double para_Retrive_Pose[SIZE_POSE];

    double para_Td[1][1];                                       // △△△△△ zlc：PL-VINS中删除，这里添加回来 △△△△△
    double para_Tr[1][1];                                       // △△△△△ zlc：PL-VINS中删除，这里添加回来 △△△△△
    double para_LineFeature[NUM_OF_F][SIZE_LINE];


    RetriveData retrive_pose_data, front_pose;
    vector<RetriveData> retrive_data_vector;

    int   loop_window_index;

    bool relocalize;
    Vector3d relocalize_t;
    Matrix3d relocalize_r;


    MarginalizationInfo* last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;



    map<double, ImageFrame> all_image_frame;

    Eigen::aligned_map<double, ImageFrame> localWindowFrames;   // zlc添加：局部滑动窗口
    std::set<double> local_active_frames;                       // zlc添加：局部活动帧的时间戳
    std::map<int, double> int_frameid2_time_frameid;            // zlc添加：由帧号到时间戳
    std::map<double, int> time_frameid2_int_frameid;            // zlc添加：由时间戳到帧号

    IntegrationBase* tmp_pre_integration;

    // relocalization variable  重定位变量
    bool   relocalization_info;
    double relo_frame_stamp;
    double relo_frame_index;
    int    relo_frame_local_index;
    vector<Vector3d> match_points;
    double relo_Pose[SIZE_POSE];
    Matrix3d drift_correct_r;
    Vector3d drift_correct_t;
    Vector3d prev_relo_t;
    Matrix3d prev_relo_r;
    Vector3d relo_relative_t;
    Quaterniond relo_relative_q;
    double relo_relative_yaw;


    // TODO: 增加开始optimization的标志位
    bool b_first_marginization_old = false;
    Transformd T_w_origin;


    // 增加视觉的voxel_map
    int64_t global_frame_cnt = 0;
    Eigen::aligned_map<vins::TimeFrameId, Transformd> local_map_poses;
};