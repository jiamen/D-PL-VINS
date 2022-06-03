#include "estimator.h"
// #define LINEINCAM
#define RGBD

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");

    f_manager.int_frameid2_time_frameid_ = &int_frameid2_time_frameid;
    f_manager.time_frameid2_int_frameid_ = &time_frameid2_int_frameid;
    f_manager.local_activate_frames_ = &local_active_frames;

    clearState();
    failure_occur = 0;
}


/**
 * @brief 外参，重投影置信度，延时设置
 *
 */
void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i ++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }

    f_manager.setRic(ric);
    // 这里可以看到虚拟相机的用法
    ProjectionFactor::sqrt_info     = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    lineProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    // lineProjectionFactor::sqrt_info =  Matrix2d::Identity();

    td = TD;                        // △△△△△ PL-VINS中删除 △△△△△
    g = G;

    // trackerData defined as global parameter   type: FeatureTracker list   size: 1
    for (int i = 0; i < NUM_OF_CAM; i ++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);


    if(FISHEYE)             // FISHEYE==0，不是鱼眼相机
    {
        for (int i = 0; i < NUM_OF_CAM; i ++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    baseline_ = BASE_LINE;
}


void Estimator::setParameter1()
{
    for (int i = 0; i < NUM_OF_CAM; i ++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }

    f_manager.setRic(ric);
    // 这里可以看到虚拟相机的用法
    ProjectionFactor::sqrt_info     = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    // lineProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    // lineProjectionFactor::sqrt_info =  Matrix2d::Identity();

    td = TD;                        // △△△△△ PL-VINS中删除 △△△△△
    g = G;

    // trackerData defined as global parameter   type: FeatureTracker list   size: 1
    for (int i = 0; i < NUM_OF_CAM; i ++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);


    if(FISHEYE)             // FISHEYE==0，不是鱼眼相机
    {
        for (int i = 0; i < NUM_OF_CAM; i ++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    baseline_ = BASE_LINE;
}


// 所有状态全部重置
void Estimator::clearState()
{
    // zlc：添加地面功能后的添加的部分
    local_active_frames.clear();
    int_frameid2_time_frameid.clear();
    time_frameid2_int_frameid.clear();

    global_frame_cnt = 0;

    // 滑动窗口中的各帧均置0
    for (int i = 0; i < WINDOW_SIZE + 1; i ++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i ++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    //  △△△△△ PL-VINS中没有将下面的all_image_frame清除 △△△△△
    for (auto& it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    solver_flag = SolverFlag::INITIAL;
    first_imu = false;
    sum_of_back  = 0;
    sum_of_front = 0;
    frame_count  = 0;

    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;                                     // △△△△△ zlc: PL-VINS中删除了这个变量 △△△△△

    //
    bapply_prior = true;
    b_plane_init_success = false;



    relocalize = false;
    retrive_data_vector.clear();
    relocalize_t = Eigen::Vector3d(0, 0, 0);
    relocalize_r = Eigen::Matrix3d::Identity();



    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;


    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();
}


/**
 * @brief 对imu数据进行处理，包括更新预积分量，和提供优化状态量的初始值
 *
 * @param[in] dt
 * @param[in] linear_acceleration
 * @param[in] angular_velocity
 */
void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    // 如果 预积分数组pre_integrations 里还没有指向第frame_count帧的预积分量，那就创建
    // 滑窗中保留11帧，frame_count表示现在处理第几帧，一般处理到第11帧时就保持不变了
    // 由于预积分是帧间约束，因此第1个预积分量实际上是用不到的
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] =
            new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]}; // acc_0, gyr_0 上一时刻的imu测量值
    }

    // 所以只有大于0才处理
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);   // 它会记录滑动窗口里两个关键帧之间的预积分量
        // if(solver_flag != NON_LINEAR)
            // 这个量用来做初始化用的
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);  // 在每一个新的图像帧上，会创建这个东西。它只记录两个图像之间的预积分量

        // 保存传感器数据
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);


        // 用imu预测这个时刻的状态量p,q,v. 这些状态量会在swf里优化
        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr   = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();              // R_{wi+1} = R_{wi} * R_{ii+1}
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc   = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}


// Eigen::Matrix<double, 5, 1>  ==>  Eigen::Matrix<double, 6, 1>
void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>>& image,
                             const map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>>& lines,
                             const std_msgs::Header& header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    // FeaturePerFrame
    // FeaturePerId
    // feature

    local_active_frames.insert(header.stamp.toSec());     // 局部活动帧的时间戳
    recomputeFrameId();                                     // 梳理局部滑窗中frame的id
    LOG(INFO) << GREEN << "local_active_frames.size: " << YELLOW << local_active_frames.size() << NO_COLOR;


    // Step 1 将特征点信息加到f_manager这个特征点管理器中，同时进行是否关键帧的检查
    // if (f_manager.addFeatureCheckParallax(frame_count, image))           // 当视差较大时，marg 老的关键帧
    if(f_manager.addFeatureCheckParallax(frame_count, image, lines))        // 对检测到的特征进行存放处理
        marginalization_flag = MARGIN_OLD;
    else                                                                    // 当视差较小时，比如静止，marg 新的图像帧
        marginalization_flag = MARGIN_SECOND_NEW;


    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;


    // all_image_frame用来做初始化相关操作，他保留滑窗起始到当前的所有帧
    // 有一些帧会因为不是KF，被MARGIN_SECOND_NEW，但是及时较新的帧被margin，他也会保留在这个容器中，因为初始化要求使用所有的帧，而非只要KF
    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;            // tmp_pre_integration 记录了上次图像到这次图像之间的积分量
    // 这里就是简单的把图像和预积分绑定在一起，这里预积分就是两帧之间的，滑窗中实际上是两个KF之间的
    // 实际上是准备用来初始化的相关数据
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};  // 在新的图像上，重新开始一个预积分量

    // 没有外参初值
    // Step 2： 外参初始化
    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            // 这里标定imu和相机的旋转外参的初值
            // 因为预积分是相邻帧的约束，因为这里得到的图像关联也是相邻的
#ifdef RGBD
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorrespondingWithDepth(frame_count - 1, frame_count);
#else
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
#endif
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                // 标志位设置成可信的外参初值
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)         // 有足够的帧数
        {
            bool result = false;
            // 要有可信的外参值，同时距离上次初始化不成功至少相邻0.1s

            // Step 3： VIO初始化
            if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
                result = initialStructure();
                initial_timestamp = header.stamp.toSec();
            }
            // if init sfm success
            if(result)
            {
                // std::cout << "estimator.cpp/processImage(): init sfm success!" << std::endl;
                solver_flag = NON_LINEAR;
                // Step 4： 非线性优化求解VIO
                // std::cout << "estimator.cpp/processImage(): init sfm success Step 4!" << std::endl;
                solveOdometry();                        // 三角化新特征 并 swf优化

                // Step 5： 滑动窗口
                // std::cout << "estimator.cpp/processImage(): init sfm success Step 5!" << std::endl;
                slideWindow();

                // Step 6： 移除无效地图点
                f_manager.removeFailures();             // 移除无效地图点
                // ROS_INFO("Initialization finish!");
                // std::cout << "estimator.cpp/processImage(): Initialization finish!" << std::endl;
                last_R = Rs[WINDOW_SIZE];               // 滑窗里最新的位姿
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];                        // 滑窗里最老的位姿
                last_P0 = Ps[0];
            }
            else
            {
                // std::cout << "estimator.cpp/processImage(): init sfm failed~ start sildeWindow()" << std::endl;
                slideWindow();
            }
        }
        else
            frame_count ++;
    }
    else
    {
        TicToc t_solve;
        // Step 4： 非线性优化求解VIO
        solveOdometry();            // 启动里程计计算， 三角化新特征 并 swf优化
        ROS_DEBUG("solver costs: %fms", t_solve.toc());
        // std::cout << "solver costs: " << t_solve.toc() << endl;

        // 检测VIO是否正常
        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            std::cout << "failure detection!" << endl;
            failure_occur = 1;
            // 如果异常，重启VIO
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            std::cout << " system reboot!" << endl;
            return;
        }

        TicToc t_margin;
        // Step 5： 滑动窗口
        slideWindow();

        // Step 6： 移除无效地图点
        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        // std::cout << "marginalization costs: " << t_margin.toc() << endl;

        // prepare output of VINS
        // 给可视化用的
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i ++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }

    recomputeFrameId();
}


void Estimator::recomputeFrameId()
{
    int_frameid2_time_frameid.clear();
    time_frameid2_int_frameid.clear();

    int localwindow_id = 0;
    std::string output;

    for (const auto tid : local_active_frames)
    {
        int_frameid2_time_frameid[localwindow_id] = tid;    // 对应帧号 写入 时间戳
        time_frameid2_int_frameid[tid] = localwindow_id;    // 对应时间戳 写入 帧号
        output += std::to_string(localwindow_id) + "->" + std::to_string(tid) + "\n";

        localwindow_id ++;
    }
    // LOG(INFO) << "WINDOW Frame: \n" << output;

}   // recomputeFrameId



void Estimator::processImage1(const map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>>& image,
    const std_msgs::Header& header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    // FeaturePerFrame
    // FeaturePerId
    // feature

    // Step 1 将特征点信息加到f_manager这个特征点管理器中，同时进行是否关键帧的检查
    if (f_manager.addFeatureCheckParallax(frame_count, image))           // 当视差较大时，marg 老的关键帧
    // if(f_manager.addFeatureCheckParallax(frame_count, image, lines))        // 对检测到的特征进行存放处理
        marginalization_flag = MARGIN_OLD;
    else                                                                    // 当视差较小时，比如静止，marg 新的图像帧
        marginalization_flag = MARGIN_SECOND_NEW;


    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;


    // all_image_frame用来做初始化相关操作，他保留滑窗起始到当前的所有帧
    // 有一些帧会因为不是KF，被MARGIN_SECOND_NEW，但是及时较新的帧被margin，他也会保留在这个容器中，因为初始化要求使用所有的帧，而非只要KF
    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;            // tmp_pre_integration 记录了上次图像到这次图像之间的积分量
    // 这里就是简单的把图像和预积分绑定在一起，这里预积分就是两帧之间的，滑窗中实际上是两个KF之间的
    // 实际上是准备用来初始化的相关数据
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};  // 在新的图像上，重新开始一个预积分量

    // 没有外参初值
    // Step 2： 外参初始化
    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            // 这里标定imu和相机的旋转外参的初值
            // 因为预积分是相邻帧的约束，因为这里得到的图像关联也是相邻的
#ifdef RGBD
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorrespondingWithDepth(frame_count - 1, frame_count);
#else
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
#endif
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                // 标志位设置成可信的外参初值
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)         // 有足够的帧数
        {
            bool result = false;
            // 要有可信的外参值，同时距离上次初始化不成功至少相邻0.1s

            // Step 3： VIO初始化
            if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
                result = initialStructure();
                initial_timestamp = header.stamp.toSec();
            }
            // if init sfm success
            if(result)
            {
                // std::cout << "estimator.cpp/processImage(): init sfm success!" << std::endl;
                solver_flag = NON_LINEAR;
                // Step 4： 非线性优化求解VIO
                // std::cout << "estimator.cpp/processImage(): init sfm success Step 4!" << std::endl;
                solveOdometry();
                // solveOdometry1();                        // 三角化新特征 并 swf优化

                // Step 5： 滑动窗口
                // std::cout << "estimator.cpp/processImage(): init sfm success Step 5!" << std::endl;
                slideWindow();

                // Step 6： 移除无效地图点
                f_manager.removeFailures();             // 移除无效地图点
                // ROS_INFO("Initialization finish!");
                // std::cout << "estimator.cpp/processImage(): Initialization finish!" << std::endl;
                last_R = Rs[WINDOW_SIZE];               // 滑窗里最新的位姿
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];                        // 滑窗里最老的位姿
                last_P0 = Ps[0];
            }
            else
            {
                // std::cout << "estimator.cpp/processImage(): init sfm failed~ start sildeWindow()" << std::endl;
                slideWindow();
            }
        }
        else
            frame_count ++;
    }
    else
    {
        TicToc t_solve;
        // Step 4： 非线性优化求解VIO
        solveOdometry();            // 启动里程计计算， 三角化新特征 并 swf优化
        // solveOdometry1();
        ROS_DEBUG("solver costs: %fms", t_solve.toc());
        std::cout << "solver costs: " << t_solve.toc() << endl;

        // 检测VIO是否正常
        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            // std::cout << "estimator.cpp/processImage(): failure detection!" << endl;
            failure_occur = 1;
            // 如果异常，重启VIO
            clearState();
            setParameter1();
            ROS_WARN("system reboot!");
            // std::cout << "estimator.cpp/processImage(): system reboot!" << endl;
            return;
        }

        TicToc t_margin;
        // Step 5： 滑动窗口
        // std::cout << "estimator.cpp/processImage(): start slideWindow()!!!!!" << endl;
        slideWindow();

        // Step 6： 移除无效地图点
        // std::cout << "estimator.cpp/processImage(): start removeFailures()!!!!!" << endl;
        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        std::cout << "marginalization costs: " << t_margin.toc() << endl;

        // prepare output of VINS
        // 给可视化用的
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i ++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}


// void Estimator::processImage(const map<int, vector<pair<int, Vector3d>>> &image, const map<int, vector<pair<int, Vector4d>>> &lines, const std_msgs::Header &header)
// // void Estimator::processImage(const map<int, vector<pair<int, Vector3d>>> &image,const map<int, vector<pair<int, Vector3d>>> &image1, const map<int, vector<pair<int, Vector4d>>> &lines, const std_msgs::Header &header)
// {
//     ROS_DEBUG("new image coming ------------------------------------------");
//     ROS_DEBUG("Adding feature points %lu", image.size());
//     //if (f_manager.addFeatureCheckParallax(frame_count, image))           // 当视差较大时，marg 老的关键帧
//     if(f_manager.addFeatureCheckParallax(frame_count, image, lines))       // 对检测到的特征进行存放处理
//         marginalization_flag = MARGIN_OLD;
//     else                                                                   // 当视差较小时，比如静止，marg 新的图像帧
//         marginalization_flag = MARGIN_SECOND_NEW;

//     ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
//     ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
//     ROS_DEBUG("Solving %d", frame_count);
//     ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
//     Headers[frame_count] = header;

//     ImageFrame imageframe(image, header.stamp.toSec());
//     imageframe.pre_integration = tmp_pre_integration;            // tmp_pre_integration 记录了上次图像到这次图像之间的积分量
//     all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
//     tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};  // 在新的图像上，重新开始一个预积分量

//     if(ESTIMATE_EXTRINSIC == 2)
//     {
//         ROS_INFO("calibrating extrinsic param, rotation movement is needed");
//         if (frame_count != 0)
//         {
//             vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
//             Matrix3d calib_ric;
//             if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
//             {
//                 ROS_WARN("initial extrinsic rotation calib success");
//                 ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
//                 ric[0] = calib_ric;
//                 RIC[0] = calib_ric;
//                 ESTIMATE_EXTRINSIC = 1;
//             }
//         }
//     }

//     if (solver_flag == INITIAL)
//     {
//         if (frame_count == WINDOW_SIZE)
//         {
//             bool result = false;
//             if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
//             {
//                 result = initialStructure();
//                 initial_timestamp = header.stamp.toSec();
//             }
//             if(result)
//             {
//                 solver_flag = NON_LINEAR;
//                 solveOdometry();          // 三角化新特征 并 swf优化
//                 slideWindow();
//                 f_manager.removeFailures();
//                 ROS_INFO("Initialization finish!");
//                 last_R = Rs[WINDOW_SIZE];
//                 last_P = Ps[WINDOW_SIZE];
//                 last_R0 = Rs[0];
//                 last_P0 = Ps[0];

//             }
//             else
//                 slideWindow();
//         }
//         else
//             frame_count++;
//     }
//     else
//     {
//         TicToc t_solve;
//         solveOdometry();      // 三角化新特征 并 swf优化
//         ROS_DEBUG("solver costs: %fms", t_solve.toc());

//         if (failureDetection())
//         {
//             ROS_WARN("failure detection!");
//             failure_occur = 1;
//             clearState();
//             setParameter();
//             ROS_WARN("system reboot!");
//             return;
//         }

//         TicToc t_margin;
//         slideWindow();
//         f_manager.removeFailures();
//         ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
//         // prepare output of VINS
//         key_poses.clear();
//         for (int i = 0; i <= WINDOW_SIZE; i++)
//             key_poses.push_back(Ps[i]);

//         last_R = Rs[WINDOW_SIZE];
//         last_P = Ps[WINDOW_SIZE];
//         last_R0 = Rs[0];
//         last_P0 = Ps[0];
//     }
// }

/* △ zlc注释
void Estimator::processImage(const map<int, vector<pair<int, Vector3d>>> &image,
                             const map<int, vector<pair<int, Vector8d>>> &lines,
                             const std_msgs::Header &header)
{

    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    //if (f_manager.addFeatureCheckParallax(frame_count, image))           // 当视差较大时，marg 老的关键帧
    if(f_manager.addFeatureCheckParallax(frame_count, image, lines))       // 对检测到的特征进行存放处理
        marginalization_flag = MARGIN_OLD;
    else                                                                   // 当视差较小时，比如静止，marg 新的图像帧
        marginalization_flag = MARGIN_SECOND_NEW;

    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;            // tmp_pre_integration 记录了上次图像到这次图像之间的积分量
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};  // 在新的图像上，重新开始一个预积分量

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
                result = initialStructure();
                initial_timestamp = header.stamp.toSec();
            }
            if(result)
            {
                solver_flag = NON_LINEAR;
                solveOdometry();          // 三角化新特征 并 swf优化
                slideWindow();
                f_manager.removeFailures();
                ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];

            }
            else
                slideWindow();
        }
        else
            frame_count++;
    }
    else
    {
        TicToc t_solve;
        solveOdometry();      // 三角化新特征 并 swf优化
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}

void Estimator::processImage(const map<int, vector<pair<int, Vector3d>>> &image, const std_msgs::Header &header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image))           // 当视差较大时，marg 老的关键帧
        marginalization_flag = MARGIN_OLD;
    else                                                                 // 当视差较小时，比如静止，marg 新的图像帧
        marginalization_flag = MARGIN_SECOND_NEW;

    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;            // tmp_pre_integration 记录了上次图像到这次图像之间的积分量
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};  // 在新的图像上，重新开始一个预积分量

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
               result = initialStructure();
               initial_timestamp = header.stamp.toSec();
            }
            if(result)
            {
                solver_flag = NON_LINEAR;
                solveOdometry();          // 三角化新特征 并 swf优化
                slideWindow();
                f_manager.removeFailures();
                ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
                
            }
            else
                slideWindow();
        }
        else
            frame_count++;
    }
    else
    {
        TicToc t_solve;
        solveOdometry();      // 三角化新特征 并 swf优化
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}
*/


// △ zlc：现在运行这个函数 △
bool Estimator::initialStructure()
{
    TicToc t_sfm;
    // Step 1 check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;

        // 从第二帧开始检查imu
        for (frame_it = all_image_frame.begin(), frame_it ++; frame_it != all_image_frame.end(); frame_it ++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            // 累加每一帧带重力加速度的deltav
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it ++; frame_it != all_image_frame.end(); frame_it ++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            // 求方差
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            // cout << "frame g " << tmp_g.transpose() << endl;
        }
        // 得到的标准差
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        // ROS_WARN("IMU variation %f!", var);
        // 实际上检查结果并没有用
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            // std::cout << "estimator.cpp/initialStructure(): IMU excitation not enouth!" << endl;
            // return false;
        }
    }


    // Step 2 global sfm
    // 做一个纯视觉slam
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;                   // 保存每个特征点的信息

    // Step 2.1 : 遍历所有的特征点，每个特征点构造一个sfmFeature
    for (auto& it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;  // 这个跟imu无关，就是存储观测特征点的帧的索引
        SFMFeature tmp_feature;                 // 用来后续做sfm
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto& it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j ++;
            Vector3d pts_j = it_per_frame.point;
            // 帧号索引以及各自坐标系下的坐标
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
#ifdef RGBD
            tmp_feature.observation_depth.push_back(make_pair(imu_j, it_per_frame.depth));     // △△△△△  zlc新添加的  △△△△△
#endif
        }
        sfm_f.push_back(tmp_feature);
    }

    // Step 2.2 ：求解当前帧与滑窗内最近的参考帧的相对位姿
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))   // 再说一遍，这里是 当前帧与滑窗内最近的参考帧的相对位姿 ，比如第10帧跟第5帧参考帧的相对变换
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        // std::cout << "estimator.cpp/initialStructure(): Not enough features or parallax; Move device around!" << endl;
        return false;
    }


    GlobalSFM sfm;
    // Step 2.3： 构造sfm问题，进行sfm的求解
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        // std::cout << "estimator.cpp/initialStructure(): global SFM failed!" << endl;
        marginalization_flag = MARGIN_OLD;
        return false;
    }


    // Step 3 solve pnp for all frame
    // 只是针对KF进行sfm，初始化需要all_image_frame中的所有元素，因此下面通过KF来求解其他的非KF的位姿
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    // i代表跟这个帧最近的KF的索引
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it ++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        // 这一帧本身就是KF，因此可以直接得到位姿
        if((frame_it->first) == Headers[i].stamp.toSec())
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();  // 得到Rw_ci
            frame_it->second.T = T[i];      // 初始化不估计平移外参
            i ++;
            continue;
        }
        if((frame_it->first) > Headers[i].stamp.toSec())
        {
            i ++;
        }

        // 最近的KF提供一个初始值，Twc -> Tcw
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];

        // eigen -> cv
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;

        // points: map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
        // 遍历这一帧对应的特征点
        for (auto& id_pts : frame_it->second.points)    // map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>>& _points,
        {   // points定义： map<int, vector< pair<int, Eigen::Matrix<double, 6, 1>> > > points;

            int feature_id = id_pts.first;                // zlc添加：特征点的序号
            // cout << "feature id " << feature_id;
            // 由于是单目，这里id_pts.second大小就是1,   观测特征点的相机是哪一个，如果是单目就为1
            for (auto& i_p : id_pts.second)     // zlc添加：id_pts=[特征点号，特征在该帧上的观测值]
            {
                // cout << " pts image_frame " << (i_p.second.head<2>() * 460 ).transpose() << endl;
                it = sfm_tracked_points.find(feature_id);     // 有对应的三角化出来的3d点  sfm_tracked_points类型是：map<int, Vector3d>
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;            // 地图点的世界坐标
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    // Vector2d img_pts = i_p.second.head<2>();
                    // cv::Point2f pts_2(img_pts(0), img_pts(1));  // 归一化平面
                    // cout << "!!!!!!!!!!!pts_2: " << pts_2 << endl;
                    // Vector6d img_pts;
                    // img_pts << i_p.second.head<6>();
                    // cv::Point2f pts_2(img_pts(3), img_pts(4));                      // 像素（观测）平面
                    // cv::Point2f pts_2(id_pts.second[0].second(3), id_pts.second[0].second(4));   // 像素（观测）平面
                    cv::Point2f pts_2(i_p.second(3), i_p.second(4));   // 像素（观测）平面
                    pts_2_vector.push_back(pts_2);
                }
            }
        }

        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if(pts_3_vector.size() < 6)
        {
            // cout << "estimator.cpp/initialStructure(): pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }

        // 依然是调用opencv求解pnp接口
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }

        // cv -> eigen,同时Tcw -> Twc
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);

        // Twc -> Twi
        // 由于尺度未恢复，因此平移暂时不转到imu系
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }


    // 到此就求解出用来做视觉惯性对齐的所有视觉帧的位姿
    // Step 4 视觉惯性对齐
    // Rs Ps ric init
#ifdef RGBD
    if (visualInitialAlignWithDepth())
#else
    if (visualInitialAlign())
#endif
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    // solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);       // △ zlc：这里藏了一个带深度信息的函数
    if(!result)
    {
        ROS_WARN("solve g failed!");
        // std::cout << "estimator.cpp/visualInitialAlignWithDepth(): solve g failed!" << endl;
        return false;
    }

    // change state
    // 首先把对齐后KF的位姿附给滑窗中的值，Rwi twc
    for (int i = 0; i <= frame_count; i ++)
    {
        Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
        Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
    }

    VectorXd dep = f_manager.getDepthVector();      // 根据有效特征点数初始化这个动态向量

    for (int i = 0; i < dep.size(); i ++)
        dep[i] = -1;                                // 深度预设都是-1
    f_manager.clearDepth(dep);                   // 特征管理器把所有的特征点逆深度也设置为-1

    // triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for(int i = 0; i < NUM_OF_CAM; i ++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    // f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));
    // 多约束三角化所有的特征点，注意，仍带是尺度模糊的
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));    // △△△△△ zlc更改过的三角化函数 △△△△△

    double s = (x.tail<1>())(0);
    // ROS_DEBUG("the scale is %f\n", s);
    // do repropagate here  将滑窗中的预积分重新计算
    for (int i = 0; i <= WINDOW_SIZE; i ++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }

    // △△△△△ zlc：下面不一样，RGBD中去掉了估计的尺度获取，下面开始把所有的状态对齐到第0帧的imu坐标系 △△△△△
    for (int i = frame_count; i >= 0; i --)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);


    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    // 把求解出来的KF的速度赋给滑窗中
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i ++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv ++;
            // 当时求得速度是imu系，现在转到world系
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    // △△△△△ zlc添加：RGBD中已经有尺度信息了，所以下面的for循环删除了 △△△△△
    for (auto& it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }


    // 所有的P V Q全部对齐到第0帧的，同时和对齐到重力方向
    Matrix3d R0 = Utility::g2R(g);                                      // g是枢纽帧下的重力方向，得到R_w_j
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();                     // Rs[0]实际上是R_j_0
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;  // 第一帧yaw赋0
    g = R0 * g;
    // Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i ++)
    {
        Ps[i] = rot_diff * Ps[i];       // shan 转到世界坐标系下？ 没明白
        Rs[i] = rot_diff * Rs[i];       // 全部对齐到重力下，同时yaw角对齐到第一帧
        Vs[i] = rot_diff * Vs[i];
        // ROS_ERROR("%d farme's t is %f | %f | %f\n",i, Ps[i].x(), Ps[i].y(), Ps[i].z());//shan add
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    return true;
}


// △ zlc：现在运行的这个函数 △
bool Estimator::visualInitialAlignWithDepth()
{
    TicToc t_g;
    VectorXd x;

    // solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_ERROR("estimator.cpp/visualInitialAlignWithDepth(): solve g failed!");
        // std::cout << "estimator.cpp/visualInitialAlignWithDepth(): solve g failed!" << endl;
        return false;
    }

    // change state
    // 首先把对齐后KF的位姿附给滑窗中的值，Rwi twc
    for (int i = 0; i <= frame_count; i ++)
    {
        Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
        Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
    }

    VectorXd dep = f_manager.getDepthVector();  // 根据有效特征点数初始化这个动态向量

    for (int i = 0; i < dep.size(); i ++)
        dep[i] = -1;                   // 深度预设都是-1
    f_manager.clearDepth(dep);      // 特征管理器把所有的特征点逆深度也设置为-1

    // triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for(int i = 0; i < NUM_OF_CAM; i ++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    // f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));
    // 多约束三角化所有的特征点，注意，仍是带尺度模糊的
    f_manager.triangulateWithDepth(Ps, &(TIC_TMP[0]), &(RIC[0]));       // △△△△△ 这里添加了深度信息 △△△△△


    // double s = (x.tail<1>())(0);                 // △△△△△ PL-VINS和VINS中都有这部分，这里尺度信息已知 △△△△△
    // ROS_DEBUG("the scale is %f\n", s);
    // do repropagate here  将滑窗中的预积分重新计算
    for (int i = 0; i <= WINDOW_SIZE; i ++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }

    // △△△△△ 下面不一样，去掉了估计的尺度获取，下面开始把所有的状态对齐到第0帧的imu坐标系 △△△△△
    // ROS_ERROR("before %f | %f | %f\n", Ps[1].x(), Ps[1].y(), Ps[1].z());  //shan add
    for (int i = frame_count; i >= 0; i --)
        Ps[i] = Ps[i] - Rs[i] * TIC[0] - (Ps[0] - Rs[0] * TIC[0]);  // twi - tw0 = toi,就是把所有的平移对齐到滑窗中的第0帧
    // ROS_ERROR("after  %f | %f | %f\n", Ps[1].x(), Ps[1].y(), Ps[1].z());  //shan add

    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    // 把求解出来的KF的速度赋给滑窗中
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i ++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv ++;
            // 当时求得速度是imu系，现在转到world系
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }


    // 所有的P V Q全部对齐到第0帧的，同时和对齐到重力方向
    Matrix3d R0 = Utility::g2R(g);                                      // g是枢纽帧下的重力方向，得到R_w_j
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();                     // Rs[0]实际上是R_j_0
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;  // 第一帧yaw赋0
    g = R0 * g;

    // Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i ++)
    {
        Ps[i] = rot_diff * Ps[i];       // shan 转到世界坐标系下？ 没明白
        Rs[i] = rot_diff * Rs[i];       // 全部对齐到重力下，同时yaw角对齐到第一帧
        Vs[i] = rot_diff * Vs[i];
        // ROS_ERROR("%d farme's t is %f | %f | %f\n",i, Ps[i].x(), Ps[i].y(), Ps[i].z());//shan add
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    return true;
}


// △ zlc：现在运行的这个函数 △
bool Estimator::relativePose(Matrix3d& relative_R, Vector3d& relative_T, int& l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    // 优先从最前面开始
    for (int i = 0; i < WINDOW_SIZE; i ++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
#ifdef RGBD
        corres = f_manager.getCorrespondingWithDepth(i, WINDOW_SIZE);   // △△△△△ zlc新添加的函数：RGB-D加入了深度信息 △△△△△
#else
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
#endif

        // Step1：找参考帧，要求与最后帧的共视特征点足够多且视差足够大的帧
        // std::cout << corres.size()<<"\n";
        if (corres.size() > 20)  // 20
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j ++)
            {
                // △△△△△ 归一化了 △△△△△ 有变化
#ifdef RGBD
                Vector2d pts_0(corres[j].first(0)/corres[j].first(2), corres[j].first(1)/corres[j].first(2));
                Vector2d pts_1(corres[j].second(0)/corres[j].second(2), corres[j].second(1)/corres[j].second(2));
#else
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
#endif
                double parallax = (pts_0 - pts_1).norm();       // 计算了视差
                sum_parallax = sum_parallax + parallax;

            }
            // 计算每个特征点的平均视差
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            // std::cout << "average_parallax: "<<average_parallax * 460 << std::endl;

            // Step 2 ：求解基础矩阵
            // 有足够的视差在通过本质矩阵恢复第i帧和最后一帧之间的 R t T_i_last
            // if(average_parallax * 460 > 15 && m_estimator.solveRelativeRT(corres, relative_R, relative_T)) // 30
#ifdef RGBD
            // if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT_PNP(corres, relative_R, relative_T))  // △△△△△ 新添加的函数 △△△△△
            if (m_estimator.solveRelativeRT_PNP(corres, relative_R, relative_T))
#else
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
#endif
            {
                // Matrix3d relative_R2; Vector3d relative_T2;
                // m_estimator.solveRelativeRT(corres, relative_R2, relative_T2);
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}


// △ zlc：现在运行这个函数 △
void Estimator::solveOdometry()
{
    // 保证滑窗中帧数满了
    if (frame_count < WINDOW_SIZE)
        return;

    // 其次要求初始化完成
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;

#ifdef RGBD
        f_manager.triangulateWithDepth(Ps, tic, ric);   // △△△△△ 这个函数需要重点看，先把应该三角化但是没有三角化的特征点三角化 △△△△△
#else
        f_manager.triangulate(Ps, tic, ric);
#endif

        if(baseline_ > 0) // stereo
            f_manager.triangulateLine(baseline_);
        else
            f_manager.triangulateLine(Ps, tic, ric);


        LOG(INFO) << GREEN << "plane_d_w_detect: " << YELLOW << plane_d_w_detect << NO_COLOR;

        if ( ((plane_d_w_detect - Ps[WINDOW_SIZE].z()) > -1.5) && ((plane_d_w_detect - Ps[WINDOW_SIZE].z()) < -0.8) )
        {
            LOG(INFO) << GREEN << "--------------------------" << NO_COLOR;
            int num_plane_point =
                    f_manager.planePointClassify(Ps, tic, ric, -plane_d_w_detect);


            LOG(INFO) << GREEN << "num_plane_point: " << num_plane_point << NO_COLOR;

            // if (num_plane)
            if (num_plane_point > 100 && !b_plane_init_success
                && std::abs(plane_d_w_detect-Ps[WINDOW_SIZE].z() + 1) < 0.2)
            {
                b_plane_init_success = true;
                param_plane[0] = -plane_d_w_detect;
                LOG(INFO) << GREEN << "--------------------\n"
                          << "--------------------\n"
                          << "--------------------\n"
                          << "--------------------\n"
                          << NO_COLOR;
            }
            LOG(INFO) << GREEN << "--------------------" << NO_COLOR;
        }

        ROS_DEBUG("triangulation costs %f", t_tri.toc());
        // std::cout << "triangulation costs: " << t_tri.toc() << endl;

        // optimization();

        onlyLineOpt();              // 三角化以后，优化一把
        optimizationwithLine();

#ifdef LINEINCAM
        LineBAincamera();
#else
//        LineBA();
#endif
    }
}


void Estimator::solveOdometry1()
{
    // 保证滑窗中帧数满了
    if (frame_count < WINDOW_SIZE)
        return;

    // 其次要求初始化完成
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;


        f_manager.triangulateWithDepth(Ps, tic, ric);   // △△△△△ 这个函数需要重点看，先把应该三角化但是没有三角化的特征点三角化 △△△△△
        f_manager.triangulate(Ps, tic, ric);



        ROS_DEBUG("triangulation costs %f", t_tri.toc());
        std::cout << "triangulation costs: " << t_tri.toc() << endl;

        optimization();

#ifdef LINEINCAM
        LineBAincamera();
#else
//        LineBA();
#endif
    }
}


void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

#ifdef LINEINCAM
    MatrixXd lineorth = f_manager.getLineOrthVectorInCamera();
#else
    MatrixXd lineorth = f_manager.getLineOrthVector(Ps,tic,ric);
#endif

    for (int i = 0; i < f_manager.getLineFeatureCount(); ++i) {
        para_LineFeature[i][0] = lineorth.row(i)[0];
        para_LineFeature[i][1] = lineorth.row(i)[1];
        para_LineFeature[i][2] = lineorth.row(i)[2];
        para_LineFeature[i][3] = lineorth.row(i)[3];
        if(i > NUM_OF_F)
            std::cerr << " 1000  1000 1000 1000 1000 \n\n";
    }

}

// △ zlc：现在也运行这个函数 △
void Estimator::double2vector()
{
    // 相机姿态需要变化考虑优化以后，把yaw量旋转回去
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);    // 优化之前的0th的姿态
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    // 优化以后的0th的姿态
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                      para_Pose[0][3],
                                                      para_Pose[0][4],
                                                      para_Pose[0][5]).toRotationMatrix());

    // 优化前后，yaw的变化                                                      
    double y_diff = origin_R0.x() - origin_R00.x();    
    // TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));

    // 由于VI系统的（绝对位置x,y,z,以及yaw）是不可观的。而优化过程中没有固定yaw角，因此yaw会朝着使得误差函数最小的方向优化，但这不一定是正确的。
    // 所以这里把 yaw角的变化量给旋转回去。
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        // Position 也转移到yaw角优化前的 0th坐在的世界坐标下
        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                para_Pose[i][1] - para_Pose[0][1],
                                para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    // 跟yaw没关系，所以不用管优化前后yaw的变化
    for (int i = 0; i < NUM_OF_CAM; i ++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }


    // std::cout <<"----------\n"<< Rwow1 <<"\n"<<twow1<<std::endl;
    MatrixXd lineorth_vec(f_manager.getLineFeatureCount(),4);
    for (int i = 0; i < f_manager.getLineFeatureCount(); ++ i)
    {
        Vector4d orth(para_LineFeature[i][0],
                      para_LineFeature[i][1],
                      para_LineFeature[i][2],
                      para_LineFeature[i][3]);

        lineorth_vec.row(i) = orth;
    }

#ifdef LINEINCAM
    f_manager.setLineOrthInCamera(lineorth_vec);
#else
    f_manager.setLineOrth(lineorth_vec, Ps, Rs, tic, ric);
#endif


    // 重新设置各个特征点的逆深度
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i ++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if (ESTIMATE_TD)            // 保留VINS-RGBD中的同步时间估计，实在不用可以置为0
        td = para_Td[0][0];

    // relative info between two loop frame
    if(relocalization_info)     // 类似进行一个调整
    { 
        Matrix3d relo_r;
        Vector3d relo_t;
        relo_r = rot_diff * Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5]).normalized().toRotationMatrix();
        relo_t = rot_diff * Vector3d(relo_Pose[0] - para_Pose[0][0],
                                     relo_Pose[1] - para_Pose[0][1],
                                     relo_Pose[2] - para_Pose[0][2]) + origin_P0;
        double drift_correct_yaw;
        drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
        drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
        drift_correct_t = prev_relo_t - drift_correct_r * relo_t;   
        relo_relative_t = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
        relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
        relo_relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
        //cout << "vins relo " << endl;
        //cout << "vins relative_t " << relo_relative_t.transpose() << endl;
        //cout << "vins relative_yaw " <<relo_relative_yaw << endl;
        relocalization_info = 0;    

    }

}

// △ zlc：现在运行这个函数 △
void Estimator::double2vector2()
{
    // 六自由度优化的时候，整个窗口会在空间中任意优化，这时候我们需要把第一帧在yaw,position上的增量给去掉，因为vins在这几个方向上不可观，他们优化的增量也不可信。
    // 所以这里的操作过程就相当于是 fix 第一帧的 yaw 和 postion, 使得整个轨迹不会在空间中任意飘。
    // 相机姿态需要变化考虑优化以后，把yaw量旋转回去
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);    //优化之前的0th的姿态
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    // 优化以后的第一帧（0th）的姿态
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5]).toRotationMatrix());

    // 优化前后，yaw的变化
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));

    // 由于VI系统的（绝对位置x,y,z,以及yaw）是不可观的。而优化过程中没有固定yaw角，因此yaw会朝着使得误差函数最小的方向优化，但这不一定是正确的。
    // 所以这里把 yaw角的变化量给旋转回去。
    for (int i = 0; i <= WINDOW_SIZE; i ++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        // Position 也转移到yaw角优化前的 0th坐在的世界坐标下
        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    // 跟yaw没关系，所以不用管优化前后yaw的变化
    for (int i = 0; i < NUM_OF_CAM; i ++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }


    // 先把line旋转到相机坐标系下
    Matrix3d Rwow1 = rot_diff;
    Vector3d tw1b(para_Pose[0][0],para_Pose[0][1],para_Pose[0][2]);
    Vector3d twow1 = -Rwow1 * tw1b + origin_P0;

    // std::cout <<"----------\n"<< Rwow1 <<"\n"<<twow1<<std::endl;
    MatrixXd lineorth_vec(f_manager.getLineFeatureCount(),4);;
    for (int i = 0; i < f_manager.getLineFeatureCount(); ++ i)
    {
        Vector4d orth(para_LineFeature[i][0],
                      para_LineFeature[i][1],
                      para_LineFeature[i][2],
                      para_LineFeature[i][3]);

        // 将line_w优化以后的角度变化yaw的变化旋转回去
        Vector6d line_w1 = orth_to_plk(orth);
        Vector6d line_wo = plk_to_pose(line_w1, Rwow1,twow1);
        orth = plk_to_orth(line_wo);

        lineorth_vec.row(i) = orth;
    }

    f_manager.setLineOrth(lineorth_vec,Ps,Rs,tic,ric);

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i ++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if (ESTIMATE_TD)            // △ zlc添加：PL-VINS中没有这一步，保留VINS-RGBD中的同步时间估计，实在不用可以置为0
        td = para_Td[0][0];
}




bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2)       // 地图点数目是否足够
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        return true;                        // △△△△△ zlc：VINS-RGBD将这一句删掉了 △△△△△
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)      // 零偏是否正常
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)           // 两帧之间运动是否过大
    {
        ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)    // 重力方向运动是否过大
    {
        ROS_INFO(" big z translation");
        return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)                    // 两帧姿态变化是否过大
    {
        ROS_INFO(" big delta_angle ");
        // return true;                         // △△△△△ zlc：VINS-RGBD：这里把这个去掉了，不中断程序 △△△△△
    }
    return false;
}


// △△△△△ PL-VINS 中添加的非线性优化函数，添加了线特征约束 △△△△△
void  Estimator::onlyLineOpt()
{
    // 固定pose， 只优化line的参数，用来调试line的一些参数，看ba优化出来的最好line地图是啥样
    ceres::Problem problem;
    ceres::LossFunction* loss_function;
    loss_function = new ceres::CauchyLoss(1.0);

    // Step 1：定义待优化的参数块，类似g2o的顶点
    // 参数块1：滑窗中位子包括位置和姿态，共11帧
    for (int i = 0; i < WINDOW_SIZE + 1; i ++)    // 将窗口内的 p,q 加入优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // p,q
        // 固定 pose
        problem.SetParameterBlockConstant(para_Pose[i]);
    }
    // 参数块2：相机imu间的外参
    for (int i = 0; i < NUM_OF_CAM; i ++)         // 外参数
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);

        // 固定 外参数， 如果不需要优化外参数就设置为fix
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);

    }
    vector2double();        // eigen --> double：将那些保存在 vector向量里的参数 移到 double指针数组里去

    // 所有特征
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto& it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
        // if (!(it_per_id.is_triangulation))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++ feature_index;            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

        //std::cout << para_LineFeature[feature_index][0] <<" "
        //        << para_LineFeature[feature_index][1] <<" "
        //        << para_LineFeature[feature_index][2] <<" "
        //        << para_LineFeature[feature_index][3] <<"\n";

        ceres::LocalParameterization* local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock( para_LineFeature[feature_index], SIZE_LINE, local_parameterization_line);  // p,q

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        for (auto& it_per_frame : it_per_id.linefeature_per_frame)
        {
            imu_j ++;
            if (imu_i == imu_j)
            {
                // continue;
            }
            Vector4d obs = it_per_frame.lineobs;                             // 在第j帧图像上的观测
            // Eigen::Matrix<double, 6, 1> obs = it_per_frame.lineobs_depth;
            lineProjectionFactor* f = new lineProjectionFactor(obs);       // 特征重投影误差
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_LineFeature[feature_index]);
            f_m_cnt ++;
        }
    }

    if(feature_index < 3)
    {
        return;
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve (options, &problem, & summary);

    // std::cout <<"!!!!!!!!!!!!!estimator.cpp/onlyLineOpt()!!!!!!!!!!!!!\n";
    double2vector();
    // std::cout << "estimator.cpp/onlyLineOpt(): " << summary.FullReport()<<std::endl;

    f_manager.removeLineOutlier(Ps,tic,ric);


}


void  Estimator::LineBA()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)    // 将窗口内的 p,q 加入优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // p,q
        // 固定 pose
        // problem.SetParameterBlockConstant(para_Pose[i]);
    }
    //  fix the first camera pose in window
    // problem.SetParameterBlockConstant( para_Pose[0] );
    // problem.SetParameterBlockConstant( para_Pose[1] );
    // problem.SetParameterBlockConstant( para_Pose[2] );

    for (int i = 0; i < NUM_OF_CAM; i++)         // 外参数
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);

        // 固定 外参数
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);

    }
    vector2double();// 将那些保存在 vector向量里的参数 移到 double指针数组里去
    // std::cout << Ps[9] <<"\n" << Ps[10]<<"\n";

    // 窗口里各帧之间的 imu factor
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)      // 由于有时候会有静止的情况出现，这时候视差一直不够，关键帧一直没有选，预积分量一直累计，可能出现时间超过10s的情况？
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);            // 预积分误差项: 误差，雅克比的计算
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }


//     所有特征
//    int f_m_cnt = 0;
//    int feature_index = -1;
//    for (auto &it_per_id : f_manager.feature)
//    {
//        it_per_id.used_num = it_per_id.feature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
//        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
//            continue;
//
//        ++feature_index;                     // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式
//
//        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
//
//        Vector3d pts_i = it_per_id.feature_per_frame[0].point;            // 图像上第一次观测到这个特征的坐标
//
//        for (auto &it_per_frame : it_per_id.feature_per_frame)
//        {
//            imu_j++;
//            if (imu_i == imu_j)
//            {
//                continue;
//            }
//            Vector3d pts_j = it_per_frame.point;                          // 在第j帧图像上的观测
//            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);     // 特征重投影误差
//            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
//            f_m_cnt++;
//        }
//    }

/////////////////////////////////////
    // Line feature
    // std::cout << "estimator.cpp/optimization(): start process Line feature!" << std::endl;
    int line_m_cnt = 0;
    int linefeature_index = -1;
    for (auto &it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++ linefeature_index;            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock( para_LineFeature[linefeature_index], SIZE_LINE, local_parameterization_line);  // p,q

        int imu_i = it_per_id.start_frame,imu_j = imu_i - 1;
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)
        {
            imu_j ++;
            if (imu_i == imu_j)
            {
                // continue;
            }
            // std::cout << "estimator.cpp/optimization(): lineobs start!!!!" << std::endl;
            Vector4d obs = it_per_frame.lineobs;                          // 在第j帧图像上的观测
            lineProjectionFactor *f = new lineProjectionFactor(obs);     // 特征重投影误差
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_LineFeature[linefeature_index]);
            line_m_cnt ++;
            // std::cout << "estimator.cpp/optimization(): lineobs end!!!!" << std::endl;
        }
    }

    // std::cout << "------------ linefeature_index : " << linefeature_index <<"\n";
    if(linefeature_index < 3)
    {
        // return;
    }

////////////////////////////////////////

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve (options, &problem, & summary);

    double2vector2();
    // std::cout << Ps[9] <<"\n" << Ps[10]<<"\n";
    // std::cout << summary.FullReport()<<std::endl;

    f_manager.removeLineOutlier(Ps,tic,ric);


}


void  Estimator::LineBAincamera()
{
    // 固定pose， 只优化line的参数，用来调试line的一些参数，看ba优化出来的最好line地图是啥样
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)    // 将窗口内的 p,q 加入优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // p,q
        // 固定 pose
        // problem.SetParameterBlockConstant(para_Pose[i]);
    }
    //  fix the first camera pose in window
    // problem.SetParameterBlockConstant( para_Pose[0] );
    // problem.SetParameterBlockConstant( para_Pose[1] );
    // problem.SetParameterBlockConstant( para_Pose[2] );

    for (int i = 0; i < NUM_OF_CAM; i++)         // 外参数
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);

        // 固定 外参数
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);

    }
    vector2double();// 将那些保存在 vector向量里的参数 移到 double指针数组里去
    // std::cout << Ps[9] <<"\n" << Ps[10]<<"\n";
    // std::cout<<"11111111111\n";
    // 窗口里各帧之间的 imu factor
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)      // 由于有时候会有静止的情况出现，这时候视差一直不够，关键帧一直没有选，预积分量一直累计，可能出现时间超过10s的情况？
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);            // 预积分误差项: 误差，雅克比的计算
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }


    // 所有特征
    int f_m_cnt = 0;
//    int feature_index = -1;
//    for (auto &it_per_id : f_manager.feature)
//    {
//        it_per_id.used_num = it_per_id.feature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
//        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
//            continue;
//
//        ++feature_index;                     // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式
//
//        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
//
//        Vector3d pts_i = it_per_id.feature_per_frame[0].point;            // 图像上第一次观测到这个特征的坐标
//        for (auto &it_per_frame : it_per_id.feature_per_frame)
//        {
//            imu_j++;
//            if (imu_i == imu_j)
//            {
//                continue;
//            }
//            Vector3d pts_j = it_per_frame.point;                          // 在第j帧图像上的观测
//            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);     // 特征重投影误差
//            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
//            f_m_cnt++;
//        }
//    }

/////////////////////////////////////
    // Line feature
    int line_m_cnt = 0;
    int linefeature_index = -1;
    for (auto &it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++linefeature_index;            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock( para_LineFeature[linefeature_index], SIZE_LINE, local_parameterization_line);  // p,q

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)
        {
            imu_j++;
            Vector4d obs = it_per_frame.lineobs;                          // 在第j帧图像上的观测
            if (imu_i == imu_j)
            {
                lineProjectionFactor_instartframe *f = new lineProjectionFactor_instartframe(obs);     // 特征重投影误差
                problem.AddResidualBlock(f, loss_function,
                                         para_LineFeature[linefeature_index]);
            } else
            {
                lineProjectionFactor_incamera *f = new lineProjectionFactor_incamera(obs);     // 特征重投影误差
                problem.AddResidualBlock(f, loss_function,
                                         para_Pose[imu_i],             // 特征都是第i帧初始化的
                                         para_Pose[imu_j],
                                         para_Ex_Pose[0],
                                         para_LineFeature[linefeature_index]);

            }
            line_m_cnt ++;
        }
    }

    // std::cout << "------------ linefeature_index : " << linefeature_index <<"\n";
    if(linefeature_index < 3)
    {
        //    return;
    }
////////////////////////////////////////

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve (options, &problem, & summary);

    double2vector();
    //  std::cout << summary.FullReport()<<std::endl;

    f_manager.removeLineOutlier(Ps,tic,ric);


}


// #define DebugFactor
// △ zlc：现在用的这个函数 △
void Estimator::optimizationwithLine()
{
    frame_cnt_ ++;                                      // △△△△△ 这里是PL-VINS中添加的，注意与frame_count的区别 △△△△△

    // zlc：1. 构建问题
    ceres::Problem problem;                             // 借助ceres进行非线性化
    ceres::LossFunction* loss_function;

    loss_function = new ceres::HuberLoss(1.0);        // △ zlc：PLVINS这里用的Huber核
    // loss_function = new ceres::CauchyLoss(1.0);

    // Step  1：定义待优化的参数块，类似g2o的顶点
    // 参数块 1：滑窗中位姿包括位置和姿态，共11帧
    for (int i = 0; i < WINDOW_SIZE + 1; i ++)          // 将窗口内的 p,q,v,ba,bg加入优化变量
    {
        // 由于姿态不满足正常的加法，也就是李群上没有加法，因此需要自己定义 它的加法
        ceres::LocalParameterization* local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // p,q
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);                // v,ba,bg
    }
    // 参数块 2：相机imu间的外参
    for (int i = 0; i < NUM_OF_CAM; i ++)               // 外参数
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            // std::cout << "estimator.cpp/optimization(): fix extinsic param" << std::endl;
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }

    /* zlc添加下面的部分
    // △△△△△ PL-VINS中直接将传感器之间的时间校准部分给删掉了，没有不同传感器数据的时间同步 △△△△△
    if (ESTIMATE_TD)
    {
        problem.AddParameterBlock(para_Td[0], 1);
        // problem.SetParameterBlockConstant(para_Td[0]);
    }
    // 实际上还有地图点，其实平凡的参数块不需要调用AddParameterBlock，增加残差块接口时自动绑定
    */

    if (b_plane_init_success)
    {
        problem.AddParameterBlock(param_plane.data(), 1);
    }


    TicToc t_whole, t_prepare;      // 统计程序运行时间
    TicToc t_solver;
    vector2double();                // 将那些保存在 vector向量里的参数 移到 double指针数组里去

#ifdef DebugFactor
    /* Debug: 监视 prior factor*/
    std::vector<ceres::ResidualBlockId> residual_block_ids;
    int marg_residual_size=0;
    std::vector<ceres::ResidualBlockId> imufactor_residual_block_ids;
    //end debug
#endif

    // Step 2：通过残差约束（误差项）来添加残差块，类似g2o的边
    // 2.1 上一次的边缘化结果作为这一次的先验，    滑动窗口marg以后，上一次的prior factor
    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        // std::cout << "estimator.cpp/optimization(): last_marginalization_info" << std::endl;
        MarginalizationFactor* marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        // ceres::ResidualBlockId block_id =
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
#ifdef DebugFactor
        marg_residual_size = marginalization_factor->num_residuals();  // used to debug
        residual_block_ids.push_back(block_id);           // used to debug
#endif
    }

    // 2.2 imu预积分的约束，  窗口里各帧之间的 imu factor
    for (int i = 0; i < WINDOW_SIZE; i ++)
    {
        int j = i + 1;
        // 时间过长这个约束就不可信了
        // 由于有时候会有静止的情况出现，这时候视差一直不够，关键帧一直没有选，预积分量一直累计，可能出现时间超过10s的情况？
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);            // 预积分误差项: 误差，雅克比的计算
        // ceres::ResidualBlockId block_id =
        problem.AddResidualBlock(imu_factor, NULL,
                                 para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
#ifdef DebugFactor
        imufactor_residual_block_ids.push_back(block_id);
#endif
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 所有特征
    int f_m_cnt = 0;
    int feature_index = -1;
    // 2.3 视觉重投影的约束
    for (auto& it_per_id : f_manager.feature)               // 遍历每一个特征点
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();         // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        // 下面进行特征有效性的检查
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++ feature_index;   // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

        // 第一个观测到这个特征点的帧idx
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;            // 图像上第一次观测到这个特征的坐标

        // 遍历看到这个特征点的所有KF
        for (auto& it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j ++;
            if (imu_i == imu_j)     // 自己跟自己不能形成重投影
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;                          // 在第j帧图像上的观测
            ProjectionFactor* f = new ProjectionFactor(pts_i, pts_j);     // 特征重投影误差

            // △△△△△ zlc添加：这里不带对 时延参数 的估计 △△△△△
            problem.AddResidualBlock(f,
                                     loss_function,
                                     para_Pose[imu_i],
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_Feature[feature_index]);

#ifdef RGBD
            // △△△△△ zlc新添加的 △△△△△
            if (it_per_id.estimate_flag == 1)
                problem.SetParameterBlockConstant(para_Feature[feature_index]);
#endif
            f_m_cnt ++;
        }
    }


/////////////////////////////////////
    // Line feature
    // std::cout << "estimator.cpp/optimization(): start process Line feature!" << std::endl;
    int line_m_cnt = 0;
    int linefeature_index = -1;
    for (auto& it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();    // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
        // if (!(it_per_id.is_triangulation))
            continue;

        // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式
        ++ linefeature_index;
        // std::cout << "estimator.cpp/optimization(): " << linefeature_index << std::endl;

        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock( para_LineFeature[linefeature_index], SIZE_LINE, local_parameterization_line);  // p,q

        int imu_i = it_per_id.start_frame,imu_j = imu_i - 1;
        for (auto& it_per_frame : it_per_id.linefeature_per_frame)
        {
            imu_j ++;
            if (imu_i == imu_j)
            {
                // continue;
            }
            // std::cout << "estimator.cpp/optimization(): lineobs start!!!!" << std::endl;
            Vector4d obs = it_per_frame.lineobs;                                // 在第j帧图像上的观测
            lineProjectionFactor *f = new lineProjectionFactor(obs);      // 特征重投影误差
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_LineFeature[linefeature_index]);
            line_m_cnt ++;
            // std::cout << "estimator.cpp/optimization(): lineobs end!!!!" << std::endl;
        }
    }
    // ROS_INFO("lineFactor: %d, pointFactor:%d", line_m_cnt, f_m_cnt);
    // std::cout << "lineFactor: " << line_m_cnt << ", pointFactor: " << f_m_cnt << std::endl;

    // if(line_m_cnt > 20)
    // {
    //     double scale = std::min(f_m_cnt /(2. * line_m_cnt), 10.);
    //     lineProjectionFactor::sqrt_info =  scale * FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    //     std::cout << "========== Line factor weight ========== \n" << scale  << std::endl; 
    // }

////////////////////////////////////////

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());
    // std::cout << "estimator.cpp/optimization(): visual measurement count: " << f_m_cnt << std::endl;
    // std::cout << "estimator.cpp/optimization(): prepare for ceres: " << t_prepare.toc() << std::endl;

    // △△△△△ 增加平面约束 △△△△△
    if (b_plane_init_success)
    {
        for (auto& it_per_id : f_manager.feature)
        {
            if (it_per_id.is_plane_point == false)
                continue;
            it_per_id.used_num = it_per_id.feature_per_frame.size();    // 已经被多少帧观测到

            if (!(it_per_id.used_num >= 2))
                continue;

            // auto host_id = it_per_id.start_frame;   // 第一个观测值对应的就是主导帧
            int host_id = it_per_id.start_frame;
            int target_id = host_id - 1;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point;            // 图像上第一次观测到这个特征的坐标

            // 遍历看到这个特征点的所有KF
            for (auto& it_per_frame : it_per_id.feature_per_frame)
            {
                target_id ++;
                if (host_id == target_id)     // 自己跟自己不能形成重投影
                {
                    continue;
                }
                Vector3d pts_j = it_per_frame.point;                          // 在第j帧图像上的观测

                /*
                    ProjectionFactor* f = new ProjectionFactor(pts_i, pts_j);     // 特征重投影误差
                    // △△△△△ zlc添加：这里不带对 时延参数 的估计 △△△△△
                    problem.AddResidualBlock(f,
                                             loss_function,
                                             para_Pose[host_id],
                                             para_Pose[target_id],
                                             para_Ex_Pose[0],
                                         para_Feature[feature_index]);

#ifdef RGBD
                // △△△△△ zlc新添加的 △△△△△
                if (it_per_id.estimate_flag == 1)
                    problem.SetParameterBlockConstant(para_Feature[feature_index]);
#endif
                */

                // ProjectionPlaneFactorAutoDiff* factor = ProjectionPlaneFactorAutoDiff::Create(pts_i, pts_j);
                auto factor = ProjectionPlaneFactorAutoDiff::Create(pts_i, pts_j);
                problem.AddResidualBlock(factor,
                                         loss_function,
                                         para_Pose[host_id],
                                         para_Pose[target_id],
                                         para_Ex_Pose[0],
                                         param_plane.data()
                                         );

            }
        }
    }



    // 2.4 回环检测相关
    if(relocalization_info)
    {
        // std::cout << "estimator.cpp/optimization(): set relocalization factor! \n";
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);
        int retrive_feature_index = 0;
        int feature_index = -1;
        // 遍历现有地图点
        for (auto& it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            ++ feature_index;
            int start = it_per_id.start_frame;
            if(start <= relo_frame_local_index)     // 这个地图点能被对应的当前帧看到
            {
                // 寻找回环帧能看到的地图点
                while((int)match_points[retrive_feature_index].z() < it_per_id.feature_id)
                {
                    retrive_feature_index ++;
                }
                // 这个地图点也能被回环帧看到
                if((int)match_points[retrive_feature_index].z() == it_per_id.feature_id)
                {
                    // 构建一个重投影约束，这个地图点的起始帧和该回环帧之间
                    Vector3d pts_j = Vector3d(match_points[retrive_feature_index].x(), match_points[retrive_feature_index].y(), 1.0);
                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                    
                    ProjectionFactor* f = new ProjectionFactor(pts_i, pts_j);
                    problem.AddResidualBlock(f, loss_function, para_Pose[start], relo_Pose, para_Ex_Pose[0], para_Feature[feature_index]);
                    retrive_feature_index ++;
                }     
            }
        }

    }

    // Step 3：ceres 优化求解
    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    // options.use_explicit_schur_complement = true;
    // options.minimizer_progress_to_stdout = true;
    // options.use_nonmonotonic_steps = true;

    // △ zlc：这里取消了注释，原本PL-VINS中不要这部分了
    if (marginalization_flag == MARGIN_OLD)
        // 下面的边缘化老帧的操作比较多，因此给他优化时间就少一些
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);  // △△△△△ ceres优化求解 △△△△△
    // std::cout << "estimator.cpp/optimization(): " << summary.BriefReport() << endl;     // △ zlc添加 △


    // ROS_INFO("Points Lines Iterations : %d", static_cast<int>(summary.iterations.size()));
    sum_solver_time_ += t_solver.toc();
    mean_solver_time_ = sum_solver_time_/frame_cnt_;
    ROS_INFO("Points Lines solver costs: %f", mean_solver_time_);
    // std::cout << "estimator.cpp/optimization(): Points Lines solver costs: " << mean_solver_time_ << std::endl;

    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());


#ifdef DebugFactor
    //----------- debug --------------------
    // marg factor
    Eigen::VectorXd err(marg_residual_size);
    err.setZero();
    ceres::Problem::EvaluateOptions opts;
    opts.residual_blocks = residual_block_ids;
    double total_cost = 0.0;
    vector<double> residuals;
    problem.Evaluate(opts,&total_cost, &residuals, nullptr, nullptr);
    for(int i=0; i< marg_residual_size; i++)
    {
        //std::cout << residuals[i]<<std::endl;
        err[i] = residuals[i];
    }
    /*
    std::ofstream file;
    file.open("/home/hyj/VINS-err-before.txt",std::ofstream::app);
    file.setf(ios::fixed, ios::floatfield);
    file.precision(5);
    file << err.squaredNorm() <<" "<<marg_residual_size<< std::endl;
    file.close();
     */
    //std::cout <<"error size: "<<residuals.size() <<" "<<marg_residual_size<<std::endl;
    std::cout<<" error befor: " << err.squaredNorm()<<" " << total_cost <<std::endl;

    // imu factor
    ceres::Problem::EvaluateOptions imufactor_opts;
    imufactor_opts.residual_blocks = imufactor_residual_block_ids;
    double total_cost_imufactor = 0.0;
    vector<double> residuals_imufactor;
    problem.Evaluate(imufactor_opts,&total_cost_imufactor, &residuals_imufactor, nullptr, nullptr);
    Eigen::VectorXd imufactor_err(residuals_imufactor.size());
    imufactor_err.setZero();
    for(int i=0; i< residuals_imufactor.size(); i++)
    {
        //std::cout << residuals[i]<<std::endl;
        imufactor_err[i] = residuals_imufactor[i];
    }
    std::cout<<" IMU error befor: " << imufactor_err.squaredNorm()<<" " << total_cost_imufactor <<std::endl;
    // --------------------  end debug -------------------------
#endif

    // double2vector();

    double2vector2();           // Line pose change

    TicToc t_culling;
    f_manager.removeLineOutlier(Ps,tic,ric);   // remove Line outlier
    // ROS_INFO("culling line feautre: %f ms", t_culling.toc());

#ifdef DebugFactor
    // ----------------  debug  ----------------------
    vector2double();
    Eigen::VectorXd err2(marg_residual_size);
    err2.setZero();
    vector<double> residuals2;
    problem.Evaluate(opts,&total_cost, &residuals2, nullptr, nullptr);
    for(int i=0; i< marg_residual_size; i++)
    {
        err[i] = residuals2[i];
    }
    /*
    std::ofstream file;
    file.open("/home/hyj/VINS-err-after.txt",std::ofstream::app);
    file.setf(ios::fixed, ios::floatfield);
    file.precision(5);
    file << err.squaredNorm() <<" "<<marg_residual_size<< std::endl;
    file.close();
     */
    //std::cout <<"error size: "<<residuals.size() <<" "<<marg_residual_size<<std::endl;
    std::cout<<"error after: " << err.squaredNorm()<<" "<< total_cost <<std::endl;
    // imu factor
    double total_cost_imufactor2 = 0.0;
    vector<double> residuals_imufactor2;
    problem.Evaluate(imufactor_opts,&total_cost_imufactor2, &residuals_imufactor2, nullptr, nullptr);
    Eigen::VectorXd imufactor_err2(residuals_imufactor2.size());
    imufactor_err2.setZero();
    for(int i=0; i< residuals_imufactor2.size(); i++)
    {
        //std::cout << residuals[i]<<std::endl;
        imufactor_err2[i] = residuals_imufactor2[i];
    }
    std::cout<<" IMU error after: " << imufactor_err2.squaredNorm()<<" " << total_cost_imufactor <<std::endl;
    //------------------- end debug  --------------------------------
#endif

    // 将优化以后要marg掉的部分转为prior factor
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        // 构建一个新的 prior info
        MarginalizationInfo* marginalization_info = new MarginalizationInfo();
        vector2double();

        /*
           将最老帧上约束转变为 prior, 那有哪些约束是跟这个最老的帧相关的呢？
           1. 上一次优化以后留下的 prior 里可能存在 
           2. 跟最老帧 存在 预积分imu 约束
           3. 最老帧上有很多特征观测约束
        */
        // 1. 上一次优化以后留下的 prior 里可能存在约束
        if (last_marginalization_info)     
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i ++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])    // 最老的一帧给丢掉
                    drop_set.push_back(i);
            }
            // 处理方式和其他残差块相同
            // construct new marginlization_factor
            MarginalizationFactor* marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        // 2. 只有第1个预积分和待边缘化参数块相连，  最老的两帧之间的 预积分 factor
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                // 跟构建ceres约束问题一样，这里也需要得到残差和雅克比   IMU因子：第0帧、第1帧的PQ VB
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});  // vector<int>{0, 1} 表示要marg的参数下表，比如这里对应para_Pose[0], para_SpeedBias[0]
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        // 3. 遍历视觉重投影的约束，   最老帧上有很多特征观测约束
        {
            int feature_index = -1;
            for (auto& it_per_id : f_manager.feature)    // 遍历所有特征
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++ feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                // 只找能被第0帧看到的特征点
                if (imu_i != 0)             // 如果这个特征的初始帧 不对应 要marg掉的最老帧0, 那就不用marg这个特征。即marg掉帧的时候，我们marg掉这帧上三角化的那些点
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                // 遍历看到这个特征点的所有KF，通过这个特征点，建立和第0帧的约束
                for (auto& it_per_frame : it_per_id.feature_per_frame)   //遍历这个特征的所有观测
                {
                    imu_j ++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;

                    ProjectionFactor* f = new ProjectionFactor(pts_i, pts_j);
                    // 待优化变量：第0帧和共视帧的PQ、外参、逆深度
                    ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                   vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                   vector<int>{0, 3});// vector<int>{0, 3} 表示要marg的参数下标，比如这里对应para_Pose[imu_i], para_Feature[feature_index]
                    // [Ti, Tj, Tbc, λ]中的下标  [0, 3]， 表示要marg掉的序号
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }
        }

        {
            // Line feature
            int linefeature_index = -1;
            for (auto& it_per_id : f_manager.linefeature)
            {
                it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
                if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
                    continue;
                ++linefeature_index;        // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

                int imu_i = it_per_id.start_frame,imu_j = imu_i - 1;
                if (imu_i != 0)             // 如果这个特征的初始帧 不对应 要marg掉的最老帧0, 那就不用marg这个特征。即marg掉帧的时候，我们marg掉这帧上三角化的那些点
                    continue;

                for (auto& it_per_frame : it_per_id.linefeature_per_frame)
                {
                    imu_j ++;

                    std::vector<int> drop_set;
                    if(imu_i == imu_j)
                    {
//                        drop_set = vector<int>{0, 2};   // marg pose and feature,  !!!! do not need marg, just drop they  !!!
                        continue;
                    }else
                    {
                        drop_set = vector<int>{2};      // marg feature
                    }

                    Vector4d obs = it_per_frame.lineobs;                          // 在第j帧图像上的观测
                    lineProjectionFactor* f = new lineProjectionFactor(obs);     // 特征重投影误差

                    ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                   vector<double *>{para_Pose[imu_j], para_Ex_Pose[0], para_LineFeature[linefeature_index]},
                                                                                   drop_set);// vector<int>{0, 2} 表示要marg的参数下标，比如这里对应para_Pose[imu_i], para_Feature[feature_index]
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }

        }


        if (b_plane_init_success)
        {
            for (auto& it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2))
                    continue;

                if (it_per_id.is_plane_point == false)
                    continue;

                int host_id = it_per_id.start_frame, target_id = host_id - 1;
                // 只找能被第0帧看到的特征点
                if (host_id != 0)   // 如果这个特征的初始帧 不对应 要marg掉的最老帧0, 那就不用marg这个特征。即marg掉帧的时候，我们marg掉这帧上三角化的那些点
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                // 遍历看到这个特征点的所有KF，通过这个特征点，建立和第0帧的约束
                for (auto& it_per_frame : it_per_id.feature_per_frame)   //遍历这个特征的所有观测
                {
                    target_id ++;
                    if (host_id == target_id)
                        continue;

                    Vector3d pts_j = it_per_frame.point;

                    auto f = ProjectionPlaneFactorAutoDiff::Create(pts_i, pts_j);
                    ResidualBlockInfo* residual_block_info =
                            new ResidualBlockInfo(f,
                                                  loss_function,
                                                  vector<double*>{para_Pose[host_id],
                                                                  para_Pose[target_id],
                                                                  para_Ex_Pose[0],
                                                                  param_plane.data()},
                                                          vector<int>{0});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }
        }


        // 所有的残差块都收集好了
        TicToc t_pre_margin;

        // 进行预处理
        marginalization_info->preMarginalize();
        // ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        // 边缘化操作
        marginalization_info->marginalize();
        // ROS_DEBUG("marginalization %f ms", t_margin.toc());

        // 即将滑窗，因此记录新地址对应的老地址
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i ++)
        {
            // 位姿和速度都要滑窗移动
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];   //
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        // 外参和时间延时不变
        for (int i = 0; i < NUM_OF_CAM; i ++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        // △△△△△ zlc：地面新添加的 △△△△△
        if (b_plane_init_success)
        {
            addr_shift[reinterpret_cast<long>(param_plane.data())] = param_plane.data();
        }

        // parameter_blocks实际上就是addr_shift的索引的集合及搬进去的新地址
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;           // 本次边缘化的所有信息
        last_marginalization_parameter_blocks = parameter_blocks;   // 代表该次边缘化对某些参数块形成约束，这些参数块在滑窗之后的地址
        
    }       // △△△△△ 边缘化第一帧 △△△△△
    else    // △△△△△ 边缘化倒数第二帧 △△△△△
    {
        // 要求有上一次边缘化的结果同时，即将被margin掉的在上一次边缘化后的约束中
        // 预积分结果合并，因此只有位姿margin掉
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks),
                       std::end(last_marginalization_parameter_blocks),
                       para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo* marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i ++)
                {
                    // 速度零偏只会margin第1个，不可能出现倒数第二个
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    // 这种case只会margin掉倒数第二个位姿
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                // 这里只会更新一下margin factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            // 这里的操作如出一辙
            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i ++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)  // 滑窗，最新帧成为次新帧
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else                        // 其他不变
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i ++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            if (b_plane_init_success)
            {
                addr_shift[reinterpret_cast<long>(param_plane.data())] = param_plane.data();
            }

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    sum_marg_time_ += t_whole_marginalization.toc();
    mean_marg_time_ = sum_marg_time_/frame_cnt_;
    // ROS_INFO("whole marginalization costs: %f", mean_marg_time_);

    // ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}


void Estimator::optimization()
{
    frame_cnt_ ++;
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)    // 将窗口内的 p,q,v,ba,bg加入优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // p,q
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);                // v,ba,bg
    }
    for (int i = 0; i < NUM_OF_CAM; i++)         // 外参数
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }

    TicToc t_whole, t_prepare;    // 统计程序运行时间
    TicToc t_solver;
    vector2double();// 将那些保存在 vector向量里的参数 移到 double指针数组里去

#ifdef DebugFactor
    /* Debug: 监视 prior factor*/
    std::vector<ceres::ResidualBlockId> residual_block_ids;
    int marg_residual_size=0;
    std::vector<ceres::ResidualBlockId> imufactor_residual_block_ids;
    //end debud
#endif
    // 滑动窗口marg以后，上一次的prior factor
    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        ceres::ResidualBlockId block_id = problem.AddResidualBlock(marginalization_factor, NULL,
                                                                   last_marginalization_parameter_blocks);
#ifdef DebugFactor
        marg_residual_size = marginalization_factor->num_residuals();  // used to debug
        residual_block_ids.push_back(block_id);           // used to debug
#endif
    }

    // 窗口里各帧之间的 imu factor
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)      // 由于有时候会有静止的情况出现，这时候视差一直不够，关键帧一直没有选，预积分量一直累计，可能出现时间超过10s的情况？
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);            // 预积分误差项: 误差，雅克比的计算
        // ceres::ResidualBlockId block_id = problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
#ifdef DebugFactor
        imufactor_residual_block_ids.push_back(block_id);
#endif
    }

    // 所有特征
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++feature_index;                     // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;            // 图像上第一次观测到这个特征的坐标

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j ++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;                          // 在第j帧图像上的观测
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);     // 特征重投影误差
            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j],
                                     para_Ex_Pose[0], para_Feature[feature_index]);

            if (it_per_id.estimate_flag == 1)
                problem.SetParameterBlockConstant(para_Feature[feature_index]);

            f_m_cnt++;
        }
    }

//    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
//    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
//    if (marginalization_flag == MARGIN_OLD)
//        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
//    else
//        options.max_solver_time_in_seconds = SOLVER_TIME;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    sum_solver_time_ += t_solver.toc();
    mean_solver_time_ = sum_solver_time_/frame_cnt_;
    ROS_INFO("whole solver costs: %f", mean_solver_time_);
    //cout << summary.BriefReport() << endl;
//    ROS_INFO("Only Points Iterations : %d", static_cast<int>(summary.iterations.size()));
//    ROS_INFO("Only Points solver costs: %f", t_solver.toc());

#ifdef DebugFactor
    //----------- debug --------------------
    // marg factor
    Eigen::VectorXd err(marg_residual_size);
    err.setZero();
    ceres::Problem::EvaluateOptions opts;
    opts.residual_blocks = residual_block_ids;
    double total_cost = 0.0;
    vector<double> residuals;
    problem.Evaluate(opts,&total_cost, &residuals, nullptr, nullptr);
    for(int i=0; i< marg_residual_size; i++)
    {
        //std::cout << residuals[i]<<std::endl;
        err[i] = residuals[i];
    }
    /*
    std::ofstream file;
    file.open("/home/hyj/VINS-err-before.txt",std::ofstream::app);
    file.setf(ios::fixed, ios::floatfield);
    file.precision(5);
    file << err.squaredNorm() <<" "<<marg_residual_size<< std::endl;
    file.close();
     */
    //std::cout <<"error size: "<<residuals.size() <<" "<<marg_residual_size<<std::endl;
    std::cout<<" error befor: " << err.squaredNorm()<<" " << total_cost <<std::endl;

    // imu factor
    ceres::Problem::EvaluateOptions imufactor_opts;
    imufactor_opts.residual_blocks = imufactor_residual_block_ids;
    double total_cost_imufactor = 0.0;
    vector<double> residuals_imufactor;
    problem.Evaluate(imufactor_opts,&total_cost_imufactor, &residuals_imufactor, nullptr, nullptr);
    Eigen::VectorXd imufactor_err(residuals_imufactor.size());
    imufactor_err.setZero();
    for(int i=0; i< residuals_imufactor.size(); i++)
    {
        //std::cout << residuals[i]<<std::endl;
        imufactor_err[i] = residuals_imufactor[i];
    }
    std::cout<<" IMU error befor: " << imufactor_err.squaredNorm()<<" " << total_cost_imufactor <<std::endl;
    // --------------------  end debug -------------------------
#endif

    double2vector();

#ifdef DebugFactor
    // ----------------  debug  ----------------------
    vector2double();
    Eigen::VectorXd err2(marg_residual_size);
    err2.setZero();
    vector<double> residuals2;
    problem.Evaluate(opts,&total_cost, &residuals2, nullptr, nullptr);
    for(int i=0; i< marg_residual_size; i++)
    {
        err[i] = residuals2[i];
    }
    /*
    std::ofstream file;
    file.open("/home/hyj/VINS-err-after.txt",std::ofstream::app);
    file.setf(ios::fixed, ios::floatfield);
    file.precision(5);
    file << err.squaredNorm() <<" "<<marg_residual_size<< std::endl;
    file.close();
     */
    //std::cout <<"error size: "<<residuals.size() <<" "<<marg_residual_size<<std::endl;
    std::cout<<"error after: " << err.squaredNorm()<<" "<< total_cost <<std::endl;
    // imu factor
    double total_cost_imufactor2 = 0.0;
    vector<double> residuals_imufactor2;
    problem.Evaluate(imufactor_opts,&total_cost_imufactor2, &residuals_imufactor2, nullptr, nullptr);
    Eigen::VectorXd imufactor_err2(residuals_imufactor2.size());
    imufactor_err2.setZero();
    for(int i=0; i< residuals_imufactor2.size(); i++)
    {
        //std::cout << residuals[i]<<std::endl;
        imufactor_err2[i] = residuals_imufactor2[i];
    }
    std::cout<<" IMU error after: " << imufactor_err2.squaredNorm()<<" " << total_cost_imufactor <<std::endl;
    //------------------- end debug  --------------------------------
#endif



    // 将优化以后要marg掉的部分转为prior factor
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        // 构建一个新的 prior info
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        /*
           将最老帧上约束转变为 prior, 那有哪些约束是跟这个最老的帧相关的呢？
           1. 上一次优化以后留下的 prior 里可能存在
           2. 跟最老帧 存在 预积分imu 约束
           3. 最老帧上有很多特征观测约束
        */
        // 1. 上一次优化以后留下的 prior 里可能存在约束
        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])    // 最老的一帧给丢掉
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        // 2. 最老的两帧之间的 预积分 factor
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                               vector<int>{0, 1});  // vector<int>{0, 1} 表示要marg的参数下标，比如这里对应para_Pose[0], para_SpeedBias[0]
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        // 3. 最老帧上有很多特征观测约束
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)    // 遍历所有特征
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)             // 如果这个特征的初始帧 不对应 要marg掉的最老帧0, 那就不用marg这个特征。即marg掉帧的时候，我们marg掉这帧上三角化的那些点
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)   //遍历这个特征的所有观测
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                   vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                   vector<int>{0, 3});// vector<int>{0, 3} 表示要marg的参数下标，比如这里对应para_Pose[imu_i], para_Feature[feature_index]
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;

    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;

        }
    }

    sum_marg_time_ += t_whole_marginalization.toc();
    mean_marg_time_ = sum_marg_time_/frame_cnt_;
    ROS_INFO("whole marginalization costs: %f", mean_marg_time_);
//    ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());

    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    // 根据边缘化种类(也就是需要将滑窗内的那一帧给边缘化掉)的不同，进行滑窗的方式也不同
    if (marginalization_flag == MARGIN_OLD)     // 边缘化最老的一帧关键帧
    {
        double t_0 = Headers[0].stamp.toSec();
        back_R0 = Rs[0];
        back_P0 = Ps[0];

        // 必须是填满了滑窗才可以
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i ++)
            {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }

            // 经过上面一系列的swap操作，WINDOW_SIZE - 1 这时是保存的 最新一帧图像的状态
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            // 预积分量就得置零
            delete pre_integrations[WINDOW_SIZE];      // 经过上面一系列的swap操作，最后这个就是多余的重复的
            // 但是要注意，最新一帧上的预积分量要累计下去
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            // buffer清空，等待新的数据来填
            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            // 清空all_image_frame最老帧之前的状态
            if (true || solver_flag == INITIAL)
            {
                // 预积分量是堆上的空间，因此需要手动释放
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                it_0->second.pre_integration = nullptr;     // △△△△△ zlc添加：PL-VINS中没有指向空指针 △△△△△

                // △△△△△ zlc添加：PL-VINS中没有下面这部分指向空指针 △△△△△
                for (map<double, ImageFrame>::iterator it = all_image_frame.begin(); it != it_0; ++ it)
                {
                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;
                }

                // 释放完空间之后再erase
                all_image_frame.erase(all_image_frame.begin(), it_0);
                all_image_frame.erase(t_0);             // △△△△△ PL-VINS中没有处理这部分 △△△△△

            }

            slideWindowOld();           // 移除窗口中老的一帧
        }
    }
    else    // 当前帧不是关键帧，边缘化次新帧
    {
        if (frame_count == WINDOW_SIZE)
        {
            // 将最后两个预积分观测合并成一个
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i ++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            // marg 掉的是倒数第2帧，这时候只需要把倒数第二帧的数据给替换掉，简单的滑窗交换
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            // reset 最新预积分量
            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            // clear相关的buffer
            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();       // 移除窗口中次新的一帧
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front ++;
    f_manager.removeFront(frame_count);
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back ++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)        // 如果初始化过了
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;

        R0 = back_R0 * ric[0];   // R0 窗口里marg掉的那个第i帧的相机坐标系到世界坐标系的变换矩阵Rwci
        R1 = Rs[0] * ric[0];     // 由于窗口移动了，丢掉最老的第i帧，数据都左移动。现在窗口里的第0帧是之前窗口里第1帧，这里Rs[0]对应以前的i+1帧，Rwi * Ric = Rwci+1
        P0 = back_P0 + back_R0 * tic[0];    // 被移除的相机的位置
        P1 = Ps[0] + Rs[0] * tic[0];        // 当前最老的相机位置

        // 下面要做的事情把移除帧看见地图点的管理权交给当前的最老帧
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();

}

void Estimator::setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r)
{
    relo_frame_stamp = _frame_stamp;
    relo_frame_index = _frame_index;
    match_points.clear();
    match_points = _match_points;
    prev_relo_t = _relo_t;
    prev_relo_r = _relo_r;
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        if(relo_frame_stamp == Headers[i].stamp.toSec())
        {
            relo_frame_local_index = i;
            relocalization_info = 1;
            for (int j = 0; j < SIZE_POSE; j++)
                relo_Pose[j] = para_Pose[i][j];
        }
    }
}