#include <stdio.h>

#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>

// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
// opencv
#include <opencv2/opencv.hpp>

#define BOOST_SYSTEM_NO_DEPRECATED     // 防止出现`boost::system::throws'被多次定义的错误

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"



#include "loop-closure/loop_closure.h"
#include "loop-closure/keyframe.h"
#include "loop-closure/keyframe_database.h"
#include "camodocal/camera_models/CameraFactory.h"

#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"


// △ zlc：下面是zlc添加的
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "./feature_tracker/feature_tracker.h"


Estimator estimator;


std::condition_variable con;
std::condition_variable con_tracker;        // △ zlc添加：FAST中添加
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
#ifdef Compressed
queue<pair<sensor_msgs::CompressedImageConstPtr, sensor_msgs::CompressedImageConstPtr>> img_depth_buf;
#else
queue<pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr>> img_depth_buf;
#endif
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
queue<sensor_msgs::PointCloudConstPtr> linefeature_buf;


std::mutex m_posegraph_buf;
queue<int> optimize_posegraph_buf;
queue<KeyFrame*>   keyframe_buf;
queue<RetriveData> retrive_data_buf;


int sum_of_wait = 0;


std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;             // △ zlc添加
std::mutex m_tracker;               // △ zlc添加：FAST中添加


std::mutex m_loop_drift;
std::mutex m_keyframedatabase_resample;
std::mutex m_update_visualization;
std::mutex m_keyframe_buf;
std::mutex m_retrive_data_buf;


double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;


bool init_feature = 0;      // zlc添加
bool init_imu = 1;          // zlc添加
double last_imu_t = 0;      // zlc添加


queue<pair<cv::Mat, double>> image_buf;
LoopClosure *loop_closure;
KeyFrameDatabase keyframe_database;

int global_frame_cnt = 0;
// camera param
camodocal::CameraPtr m_camera;
vector<int> erase_index;
std_msgs::Header cur_header;
Eigen::Vector3d  relocalize_t{Eigen::Vector3d(0, 0, 0)};
Eigen::Matrix3d  relocalize_r{Eigen::Matrix3d::Identity()};


ros::Publisher pub_restart;                 // △ zlc：PL-VINS中没有 △
double frame_cnt = 0;                       // △ zlc：PL-VINS添加
double sum_time = 0.0;                      // △ zlc：PL-VINS添加
double mean_time = 0.0;                     // △ zlc：PL-VINS添加
ros::Publisher pub_img, pub_match;


/**
 * @brief 根据当前imu数据预测当前位姿
 *      使用mid-point方法对imu状态量进行预测
 * @param[in] imu_msg
 */
Matrix3d relative_R = Matrix3d::Identity();         // △△△△△ zlc添加：FAST中添加 △△△△△
void predict(const sensor_msgs::ImuConstPtr& imu_msg)
{
    double t  = imu_msg->header.stamp.toSec();
    double dt = t - latest_time;
    latest_time = t;

    // zlc添加：PL-VINS将这部分删除了
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }

    // 得到加速度
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    // 得到角速度
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    // 上一时刻世界坐标系下加速度值
    // Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba - tmp_Q.inverse() * estimator.g);  乘开就是下面这句
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba - tmp_Q.inverse() * estimator.g);  // Qwi * (ai - ba - Qiw * g)

    // 中值陀螺仪的结果
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;                   // (gyro0 + gyro1)/2 - bg
    // 更新姿态
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);           //  Qwi * [1, 1/2 w dt]  注意：1/2的操作在函数deltaQ中


    // △△△△△ zlc添加 △△△△△
    // Transform the mean angular velocity from the IMU
    // frame to the cam0 frames.
    Vector3d cam0_mean_ang_vel = RIC.back().transpose() * un_gyr;

    // Compute the relative rotation.
    Vector3d cam0_angle_axisd = cam0_mean_ang_vel * dt;
    relative_R *= AngleAxisd(cam0_angle_axisd.norm(), cam0_angle_axisd.normalized()).toRotationMatrix().transpose();
    // △△△△△ zlc添加 △△△△△


    // 当前时刻世界坐标系下的加速度值
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba - tmp_Q.inverse() * estimator.g);
    // 加速度中值积分的值
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    // 经典物理中位置，速度更新方程
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

// 用最新VIO结果更新最新imu对应的位姿
void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = relocalize_r * estimator.Ps[WINDOW_SIZE] + relocalize_t;
    tmp_Q = relocalize_r * estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;  // 遗留的imu的buffer，因为下面需要pop，所以copy了一份
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());               // 得到最新imu时刻的可靠的位姿

}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
        std::pair<sensor_msgs::PointCloudConstPtr,sensor_msgs::PointCloudConstPtr> >>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
                std::pair<sensor_msgs::PointCloudConstPtr, sensor_msgs::PointCloudConstPtr> >> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty() || linefeature_buf.empty())
            return measurements;

        std::cout << "-------------------------------------\n";
        // std::cout << imu_buf.front()->header.stamp.toSec() << " " << imu_buf.back()->header.stamp.toSec()<<" "<<imu_buf.size() << "\n";
        // std::cout << feature_buf.front()->header.stamp.toSec() << " " << feature_buf.back()->header.stamp.toSec() << "\n";
        if (!(imu_buf.back()->header.stamp > feature_buf.front()->header.stamp)) // 如果imu最新数据的时间戳不大于最旧图像的时间戳，那得等imu数据
        {
            ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait ++;
            return measurements;
        }


        if (!(imu_buf.front()->header.stamp < feature_buf.front()->header.stamp)) // 如果imu最老的数据时间戳不小于最旧图像的时间，那得把最老的图像丢弃
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            linefeature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();      // 此时就保证了图像前一定有imu数据
        feature_buf.pop();
        sensor_msgs::PointCloudConstPtr linefeature_msg = linefeature_buf.front();
        linefeature_buf.pop();


        // 一般第一帧不会严格对齐，但是后面就都会对齐，当然第一帧也不会用到      遍历两个图像之间所有的imu数据
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp <= img_msg->header.stamp)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }

        // 保留图像时间戳后一个imu数据，但不会从buffer中扔掉
        // imu    *   *
        // image    *
        // IMUs.emplace_back(imu_buf.front());          // △△△△△ zlc删除：PL-VINS中删除的 △△△△△
        if (IMUs.empty())
        {
            ROS_WARN("no imu between two image");
            std::cout << "no imu between two image\n";
        }
        // std::cout << "measurements size: "<<measurements.size() <<"\n";
        measurements.emplace_back(IMUs, std::make_pair(img_msg,linefeature_msg) );
    }
    return measurements;
}


/**
 * @brief imu消息存进buffer，同时按照imu频率预测位姿并发送，这样就可以提高里程计频率
 *
 * @param[in] imu_msg
 */
bool first_image_flag = true;   // △△△△△ zlc添加：FAST中添加 △△△△△
double first_image_time;        // △△△△△ zlc添加：FAST中添加 △△△△△
double last_image_time = 0;     // △△△△△ zlc添加：FAST中添加 △△△△△
double prev_image_time = 0;     // △△△△△ zlc添加：FAST中添加 △△△△△
bool init_pub = 0;              // △△△△△ zlc添加：FAST中添加 △△△△△
int pub_count = 1;              // △△△△△ zlc添加：FAST中添加 △△△△△

void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    // zlc添加：错误判断，当前IMU的时间戳＜上一帧IMU的时间戳，那么IMU时间肯定错了
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        // △△△△△ 这里是新添加的，打印了imu的时间戳 △△△△△
        ROS_WARN("imu message in disorder! %f",imu_msg->header.stamp.toSec() );
        return;
    }

    // 讲一下线程锁 条件变量用法
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();


    // △△△△△ zlc:pub "imu_propagate" △△△△△
    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

// void raw_image_callback(const sensor_msgs::ImageConstPtr &img_msg)
// {
//     cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
//     //image_pool[img_msg->header.stamp.toNSec()] = img_ptr->image;
//     if(LOOP_CLOSURE)
//     {
//         i_buf.lock();
//         image_buf.push(make_pair(img_ptr->image, img_msg->header.stamp.toSec()));
//         i_buf.unlock();
//     }
// }


/**
 * @brief 将前端信息送进buffer
 *
 * @param[in] feature_msg
 */
void feature_callback(const sensor_msgs::PointCloudConstPtr& feature_msg)
{
    // △△△△△ zlc添加：PL-VINS将下面初始特征判断删除了 △△△△△
    if (!init_feature)
    {
        // skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }

    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}


void linefeature_callback(const sensor_msgs::PointCloudConstPtr& feature_msg)
{
    m_buf.lock();
    linefeature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}


void send_imu(const sensor_msgs::ImuConstPtr& imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (current_time < 0)
        current_time = t;
    double dt = t - current_time;
    current_time = t;

    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};

    double dx = imu_msg->linear_acceleration.x - ba[0];
    double dy = imu_msg->linear_acceleration.y - ba[1];
    double dz = imu_msg->linear_acceleration.z - ba[2];

    double rx = imu_msg->angular_velocity.x - bg[0];
    double ry = imu_msg->angular_velocity.y - bg[1];
    double rz = imu_msg->angular_velocity.z - bg[2];
    // ROS_DEBUG("IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);

    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}


/**
 * @brief 将vins估计器复位
 *
 * @param[in] restart_msg
 */
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        std::cout << "restart the estimator!\n";

        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}


#ifdef Compressed
void img_callback(const sensor_msgs::CompressedImageConstPtr &color_msg,
                  const sensor_msgs::CompressedImageConstPtr &depth_msg)
#else
void img_callback(const sensor_msgs::ImageConstPtr& color_msg,
                  const sensor_msgs::ImageConstPtr& depth_msg)
#endif
{
    m_tracker.lock();
    img_depth_buf.emplace(color_msg, depth_msg);
    m_tracker.unlock();
    con_tracker.notify_one();
}


void relocalization_callback(const sensor_msgs::PointCloudConstPtr& points_msg)
{
    // printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

/*
// thread: visual-inertial odometry
void process()
{
    while (true)    // 这个线程是会一直循环下去
    {
        // △△△△△ 这里是VINS中的数据，原本是只有图像点和IMU数据 △△△△△
        // std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
                std::pair<sensor_msgs::PointCloudConstPtr, sensor_msgs::PointCloudConstPtr> >> measurements;

        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();            // 数据buffer的锁解锁，回调可以继续塞数据了


        m_estimator.lock();     // 进行后端求解，不能和复位重启冲突    △△△△△ zlc添加：PL-VINS中将这个锁删掉了 △△△△△


        // 给予范围的for循环，这里就是遍历每组image imu组合
        for (auto& measurement : measurements)
        {
            for (auto& imu_msg : measurement.first)
                send_imu(imu_msg);                     // 处理imu数据, 预测 pose

            // set relocalization frame     重定位相关部分
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())       // 取出最新的回环帧
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)           // 有效回环信息
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();        // 回环的当前帧时间戳
                for (unsigned int i = 0; i < relo_msg->points.size(); i ++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;     // 回环帧的归一化坐标和地图点idx
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                // 回环帧的位姿
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }


            auto point_and_line_msg = measurement.second;
            auto img_msg = point_and_line_msg.first;
            auto line_msg = point_and_line_msg.second;
            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            // 特征点id -> 特征点信息（相机标号，特征点xyz_uv 5维信息）
            map<int, vector<pair<int, Matrix<double, 5, 1>>>> image2;

            map<int, vector<pair<int, Vector3d>>> image;
            map<int, vector<pair<int, Vector4d>>> image1;

            for (unsigned int i = 0; i < img_msg->points.size(); i ++)
            {

                int v = img_msg->channels[0].values[i] + 0.5;

                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;         // 被几号相机观测到的，如果是单目，camera_id = 0

                double x = img_msg->points[i].x;        // 去畸变后归一化相机坐标系坐标
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];    // 特征点像素坐标
                double p_v = img_msg->channels[2].values[i];

                ROS_ASSERT(z == 1);                     // 检查是不是归一化

                image[feature_id].emplace_back(camera_id, Vector3d(x, y, z));
                image1[feature_id].emplace_back(camera_id, Vector4d(x, y, z, 0));

                Eigen::Matrix<double, 5, 1> xyz_uv;
                xyz_uv << x, y, z, p_u, p_v;
                image2[feature_id].emplace_back(camera_id, xyz_uv);
            }
            // map<int, vector<pair<int, Matrix<double, 5, 1>>>> image;
            // for (unsigned int i = 0; i < img_msg->points.size(); i++)
            // {
            //     int v = img_msg->channels[0].values[i] + 0.5;
            //     int feature_id = v / NUM_OF_CAM;
            //     int camera_id = v % NUM_OF_CAM;        // 被几号相机观测到的，如果是单目，camera_id = 0
            //     double x = img_msg->points[i].x;
            //     double y = img_msg->points[i].y;
            //     double z = img_msg->points[i].z;
            //     double p_u = img_msg->channels[1].values[i];
            //     double p_v = img_msg->channels[2].values[i];
            //     Eigen::Matrix<double, 5, 1> xyz_uv;
            //     xyz_uv << x, y, z, p_u, p_v;

            //     ROS_ASSERT(z == 1);
            //     image[feature_id].emplace_back(camera_id, xyz_uv);
            // }
            map<int, vector<pair<int, Vector4d>>> lines;
            for (unsigned int i = 0; i < line_msg->points.size(); i ++)
            {
                int v = line_msg->channels[0].values[i] + 0.5;
                // std::cout<< "receive id: " << v / NUM_OF_CAM << "\n";
                int feature_id = v / NUM_OF_CAM;
                int camera_id  = v % NUM_OF_CAM;                // 被几号相机观测到的，如果是单目，camera_id = 0
                double x_startpoint = line_msg->points[i].x;
                double y_startpoint = line_msg->points[i].y;
                double x_endpoint = line_msg->channels[1].values[i];
                double y_endpoint = line_msg->channels[2].values[i];
                // ROS_ASSERT(z == 1);
                lines[feature_id].emplace_back(camera_id, Vector4d(x_startpoint, y_startpoint, x_endpoint, y_endpoint));
            }

            // estimator.processImage(image,lines, img_msg->header);   // 处理image数据，这时候的image已经是特征点数据，不是原始图像了。
            estimator.processImage(image2, lines, img_msg->header);   // 处理image数据，这时候的image已经是特征点数据，不是原始图像了。


            // 一些打印以及topic的发送
            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";
            cur_header = header;

            m_loop_drift.lock();
            pubOdometry(estimator, header, relocalize_t, relocalize_r);
            pubKeyPoses(estimator, header, relocalize_t, relocalize_r);
            pubCameraPose(estimator, header, relocalize_t, relocalize_r);
            pubLinesCloud(estimator, header, relocalize_t, relocalize_r);
            pubPointCloud(estimator, header, relocalize_t, relocalize_r);
            pubTF(estimator, header, relocalize_t, relocalize_r);
            pubKeyframe(estimator);

            if (relo_msg != NULL)
                pubRelocalization(estimator);

            m_loop_drift.unlock();
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }

        m_estimator.unlock();       // △△△△△ zlc添加：PL-VINS中将这个锁删掉了 △△△△△

        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}
*/


// thread: feature tracker  前端特征跟踪线程
void process_tracker()
{
    while (1)
    {
#ifdef Compressed
        sensor_msgs::CompressedImageConstPtr color_msg = NULL;
        sensor_msgs::CompressedImageConstPtr depth_msg = NULL;
#else
        sensor_msgs::ImageConstPtr color_msg = NULL;
        sensor_msgs::ImageConstPtr depth_msg = NULL;
#endif
        std::unique_lock<std::mutex> lk_tracker(m_tracker);
        con_tracker.wait(lk_tracker, [&]
        {
            return !img_depth_buf.empty();
        });

        color_msg = img_depth_buf.front().first;
        depth_msg = img_depth_buf.front().second;
        img_depth_buf.pop();
        lk_tracker.unlock();


        Matrix3d tmp_relative_R = relative_R;               // △△△△△ zlc添加：IMU辅助判断特征区域 △△△△△
        relative_R = Matrix3d::Identity();                  // △△△△△ zlc添加：重新置为单位阵 △△△△△


        if(first_image_flag)        // 对第一帧图像的基本操作
        {
            first_image_flag = false;
            first_image_time = color_msg->header.stamp.toSec();
            last_image_time  = color_msg->header.stamp.toSec();     // △ zlc：原本PL-VINS中删除
            continue;                                               // △ zlc：FAST中讲return改为continue
        }

        // detect unstable camera stream   检测到不稳定的视频流
        // 检查时间戳是否正常，这里认为超过一秒或者错乱就异常
        // 图像时间差太多，光流追踪就会失败，这里没有描述子匹配，因此对时间戳要求就高
        if (color_msg->header.stamp.toSec() - last_image_time > 1.0 || color_msg->header.stamp.toSec() < last_image_time)
        {
            // 一些常规的reset操作
            ROS_WARN("image discontinue! reset the feature tracker!");
            first_image_flag = true;
            last_image_time = 0;
            pub_count = 1;

            // △△△△△ zlc添加：下面是FAST中添加的重置 △△△△△
            ROS_WARN("restart the estimator!");
            m_buf.lock();
            while (!feature_buf.empty())
                feature_buf.pop();
            while (!imu_buf.empty())
                imu_buf.pop();
            m_buf.unlock();
            m_estimator.lock();
            estimator.clearState();
            estimator.setParameter();
            m_estimator.unlock();
            current_time = -1;
            last_imu_t = 0;
            // △△△△△ zlc添加：上面是FAST中添加的重置 △△△△△

            std_msgs::Bool restart_flag;
            restart_flag.data = true;
            pub_restart.publish(restart_flag);              // 告诉其他模块要重启了

            continue;
        }

        prev_image_time = last_image_time;
        last_image_time = color_msg->header.stamp.toSec();          // 更新上一帧图像时间


        // frequency control, 如果图像频率低于一个值，就发送         控制一下发给后端的频率
        if (round(1.0 * pub_count / (color_msg->header.stamp.toSec() - first_image_time)) <= FREQ)  // 保证发给后端的跑不超过这个频率
        {
            PUB_THIS_FRAME = true;
            // reset the frequency control
            // 这段时间的频率和预设频率十分接近，就认为这段时间很棒，重启一下，避免 delta t 太大
            if (abs(1.0 * pub_count / (color_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
            {
                first_image_time = color_msg->header.stamp.toSec();
                pub_count = 0;
            }
        }
        else
            PUB_THIS_FRAME = false;

        // 即使不发布也是正常做光流追踪的！ 光流对图像的变化要求尽可能小
        // encodings in ros: http://docs.ros.org/diamondback/api/sensor_msgs/html/image__encodings_8cpp_source.html
        // color has encoding RGB8   把ros message 转成 cv::Mat
        cv_bridge::CvImageConstPtr ptr;
        if (color_msg->encoding == "8UC1") // shan:why 8UC1 need this operation? Find answer:https://github.com/ros-perception/vision_opencv/issues/175
        {
            sensor_msgs::Image img;
            img.header = color_msg->header;
            img.height = color_msg->height;
            img.width  = color_msg->width;
            img.is_bigendian = color_msg->is_bigendian;
            img.step = color_msg->step;
            img.data = color_msg->data;
            img.encoding = "mono8";         // 更改color_msg彩色图像帧消息的编码方式
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else    // 如果是mono8编码方式直接复制就可以
            ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::MONO8);

        // △△△△△ 下面这部分新添加的 △△△△△
        // depth has encoding TYPE_16UC1
        cv_bridge::CvImageConstPtr depth_ptr;
        // debug use     std::cout<<depth_msg->encoding<<std::endl;
        {
            sensor_msgs::Image img;
            img.header = depth_msg->header;
            img.height = depth_msg->height;
            img.width  = depth_msg->width;
            img.is_bigendian = depth_msg->is_bigendian;
            img.step = depth_msg->step;
            img.data = depth_msg->data;
            img.encoding = sensor_msgs::image_encodings::MONO16;        // △△△△△ 更改了图像编码方式 △△△△△
            depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);     // △△△△△ 新更改的指针 △△△△△
        }
        // △△△△△ 上面这部分新添加的 △△△△△


        cv::Mat show_img = ptr->image;
        TicToc t_r;
        // init pts here, using readImage()

        frame_cnt ++;       // PL-VINS 添加的变量操作
        for (int i = 0; i < NUM_OF_CAM; i ++)
        {
            ROS_DEBUG("processing camera %d", i);
            if (i != 1 || !STEREO_TRACK)    // 单目 或者 非双目,   #### readImage： 这个函数里 做了很多很多事, 提取特征 ####
            {
                // △△△△△ 下面这句话有改动 △△△△△
                estimator.trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)),
                                                   depth_ptr->image.rowRange(ROW * i, ROW * (i + 1)),
                                                   color_msg->header.stamp.toSec(),
                                                   tmp_relative_R);
                // rowRange(i,j) 取图像的i～j行
            }
            else
            {
                if (EQUALIZE)   // 对图像进行直方图均衡化处理
                {
                    std::cout <<" EQUALIZE " << std::endl;
                    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                    clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), estimator.trackerData[i].cur_img);
                }
                else
                {
                    estimator.trackerData[i].cur_img   = ptr->image.rowRange(ROW * i, ROW * (i + 1));
                    estimator.trackerData[i].cur_depth = depth_ptr->image.rowRange(ROW * i, ROW * (i + 1));   // △△△△△ 先添加的深度数据 △△△△△
                }

            }
// always 0
#if SHOW_UNDISTORTION
            trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
        }

        // update all id in ids[]
        // If has ids[i] == -1 (newly added pts by cv::goodFeaturesToTrack), substitute by gloabl id counter (n_id)
        for (unsigned int i = 0; ; i ++)
        {
            bool completed = false;
            for (int j = 0; j < NUM_OF_CAM; j ++)
                if (j != 1 || !STEREO_TRACK)
                    completed |= estimator.trackerData[j].updateID(i);    // 单目的情况下可以直接用=号
            if (!completed)
                break;
        }


        // 给后端喂数据
        if (PUB_THIS_FRAME)
        {
            frame_cnt ++;
            // vector<int> test;
            cv::Mat show_depth = depth_ptr->image;      // △△△△△ 新添加的深度值 △△△△△
            pub_count ++;                               // 图像帧发送成功后，计数器更新

            // http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html
            sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
            sensor_msgs::ChannelFloat32 id_of_point;    //  feature id
            sensor_msgs::ChannelFloat32 u_of_point;     //  u
            sensor_msgs::ChannelFloat32 v_of_point;     //  v
            sensor_msgs::ChannelFloat32 velocity_x_of_point;
            sensor_msgs::ChannelFloat32 velocity_y_of_point;
            // Use round to get depth value of corresponding points
            sensor_msgs::ChannelFloat32 depth_of_point;     // △△△△△ 新添加的变量 △△△△△

            feature_points->header = color_msg->header;
            feature_points->header.frame_id = "world";


            vector<set<int>> hash_ids(NUM_OF_CAM);
            for (int i = 0; i < NUM_OF_CAM; i ++)
            {
                auto& un_pts = estimator.trackerData[i].cur_un_pts;         // 去畸变的归一化相机坐标系
                auto& cur_pts = estimator.trackerData[i].cur_pts;           // 像素坐标
                auto& ids = estimator.trackerData[i].ids;                           // id
                auto& pts_velocity = estimator.trackerData[i].pts_velocity; // 归一化坐标下的速度

                for (unsigned int j = 0; j < ids.size(); j ++)
                {
                    // 只发布追踪大于1的，因为等于1没法构成重投影约束，也没法三角化
                    if (estimator.trackerData[i].track_cnt[j] > 1)
                    {
                        int p_id = ids[j];
                        // not used
                        hash_ids[i].insert(p_id);       // 这个并没有用到
                        geometry_msgs::Point32 p;
                        p.x = un_pts[j].x;
                        p.y = un_pts[j].y;
                        p.z = 1;

                        // push normalized point to pointcloud
                        // 利用这个ros消息的格式进行信息存储
                        feature_points->points.push_back(p);
                        // push other info
                        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                        u_of_point.values.push_back(cur_pts[j].x);
                        v_of_point.values.push_back(cur_pts[j].y);
                        velocity_x_of_point.values.push_back(pts_velocity[j].x);    // PL-VINS 中并没有用这个速度，直接删掉了
                        velocity_y_of_point.values.push_back(pts_velocity[j].y);

                        // nearest neighbor....fastest  may be changed   △△△△△ 新添加的 △△△△△
                        // show_depth: 480*640   y:[0,480]   x:[0,640]
                        depth_of_point.values.push_back(
                                (int) show_depth.at<unsigned short>(round(cur_pts[j].y), round(cur_pts[j].x)));
                        // debug use: print depth pixels
                        // test.push_back((int)show_depth.at<unsigned short>(round(cur_pts[j].y),round(cur_pts[j].x)));
                    }
                }
            }

            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            feature_points->channels.push_back(velocity_x_of_point);
            feature_points->channels.push_back(velocity_y_of_point);
            feature_points->channels.push_back(depth_of_point);         // △△△△△ 新添加的深度 △△△△△
            ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
            // skip the first image; since no optical speed on frist image
            if (!init_pub)
            {
                init_pub = 1;
            }
            else
            {
                if (!init_feature)
                {
                    //skip the first detected feature, which doesn't contain optical flow speed
                    init_feature = 1;
                    continue;
                }
                pub_img.publish(feature_points);     // △△△△△ zlc：这里不再发布，直接存 △△△△△
                                                        // △△△△△ "feature"  前端得到的特征点信息通过这个publisher发布出去 △△△△△
                m_buf.lock();
                feature_buf.push(feature_points);
                m_buf.unlock();
                con.notify_one();
            }


            // Show image with tracked points in rviz (by topic pub_match), 可视化相关操作
            if (SHOW_TRACK)
            {
                ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
                // cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
                cv::Mat stereo_img = ptr->image;

                for (int i = 0; i < NUM_OF_CAM; i ++)
                {
                    cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                    cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);       // ?? seems useless?

                    for (unsigned int j = 0; j < estimator.trackerData[i].cur_pts.size(); j ++)
                    {
                        double len = std::min(1.0, 1.0 * estimator.trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, estimator.trackerData[i].cur_pts[j], 2,
                                   cv::Scalar(255 * (1 - len), 0, 255 * len),
                                   2);
                        // draw speed line
                        /*
                        Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                        Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                        Vector3d tmp_prev_un_pts;
                        tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                        tmp_prev_un_pts.z() = 1;
                        Vector2d tmp_prev_uv;
                        trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                        cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                        */
                        // char name[10];
                        // sprintf(name, "%d", trackerData[i].ids[j]);
                        // cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));



                    }

                    for (unsigned int j = 0; j < estimator.trackerData[i].predict_pts.size(); j ++)
                    {
                        cv::circle(tmp_img, estimator.trackerData[i].predict_pts[j], 1,
                                   cv::Scalar(0, 255, 0), 1);
                    }
                }
                // cv::imshow("vis", stereo_img);
                // cv::waitKey(5);
                // pub_match.publish(ptr->toImageMsg());       // △△△△△ 这里才正式的发布图片消息 △△△△△
                pubTrackImg(ptr);
            }
        }
        sum_time += t_r.toc();
        mean_time = sum_time/frame_cnt;

        std::cout << "!!!!!!!!!shi-thomas feature cost: " << mean_time << endl;

#ifdef Compressed
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
#endif
    }
}




// thread: visual-inertial odometry  后端优化过程
void process()
{
    while (true)    // 这个线程是会一直循环下去
    {
        // △△△△△ 这里是VINS中的数据，原本是只有图像点和IMU数据 △△△△△
        // std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
                std::pair<sensor_msgs::PointCloudConstPtr, sensor_msgs::PointCloudConstPtr> >> measurements;

        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
        {
            return (measurements = getMeasurements()).size() != 0;
        });
        lk.unlock();            // 数据buffer的锁解锁，回调可以继续塞数据了


        m_estimator.lock();     // 进行后端求解，不能和复位重启冲突    △△△△△ zlc添加：PL-VINS中将这个锁删掉了 △△△△△


        // 给予范围的for循环，这里就是遍历每组image imu组合
        for (auto& measurement : measurements)
        {

            auto point_and_line_msg = measurement.second;
            auto img_msg = point_and_line_msg.first;
            auto line_msg = point_and_line_msg.second;
            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());


            // auto img_msg = measurement.second;       // △△△△△ 原VINS的代码，这里封装线特征之后需要修改 △△△△△
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;

            // △△△△△ 遍历imu，这部分在PL-VINS中抽象为了一个send_imu()函数 △△△△△
            for (auto& imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                if (t <= img_t)
                {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;

                    double ba[]{0.0, 0.0, 0.0};     // △△△△△ PL-VINS △△△△△
                    double bg[]{0.0, 0.0, 0.0};     // △△△△△ PL-VINS △△△△△

                    dx = imu_msg->linear_acceleration.x - ba[0];
                    dy = imu_msg->linear_acceleration.y - ba[1];
                    dz = imu_msg->linear_acceleration.z - ba[2];
                    rx = imu_msg->angular_velocity.x - bg[0];
                    ry = imu_msg->angular_velocity.y - bg[1];
                    rz = imu_msg->angular_velocity.z - bg[2];
                    // ROS_DEBUG("IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);

                    // 时间差和imu数据送进去
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else    // 这就是针对最后一个imu数据，需要做一个简单的线性插值
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // set relocalization frame    重定位相关部分
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())   // 取出最新的回环帧
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)       // 有效回环信息
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();        // 回环的当前帧时间戳
                for (unsigned int i = 0; i < relo_msg->points.size(); i ++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;     // 回环帧的归一化坐标和地图点idx
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                // 回环帧的位姿
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }



            TicToc t_s;
            // 特征点id -> 特征点信息（相机标号，特征点xyz_uv_velocity 7维信息） + depth信息
            // map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image;         // △△△△△ 有改动，7维改为8维 △△△△△
            map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>> image;

            map<int, vector<pair<int, Vector3d>>> image1;       // △△△△△ PL-VINS添加 △△△△△
            map<int, vector<pair<int, Vector4d>>> image2;       // △△△△△ PL-VINS添加 △△△△△

            for (unsigned int i = 0; i < img_msg->points.size(); i ++)
            {

                int v = img_msg->channels[0].values[i] + 0.5;

                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;     // 被几号相机观测到的，如果是单目，camera_id = 0

                double x = img_msg->points[i].x;    // 去畸变后归一化相机坐标系坐标
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];                // 特征点像素坐标
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];         // 特征点速度，像素的移动速度
                double velocity_y = img_msg->channels[4].values[i];
                double depth = img_msg->channels[5].values[i] / 1000.0;     // △△△△△ 先添加的深度 △△△△△

                ROS_ASSERT(z == 1);                 // 检查是不是归一化

                image1[feature_id].emplace_back(camera_id, Vector3d(x, y, z));
                image2[feature_id].emplace_back(camera_id, Vector4d(x, y, z, 0));

                // Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;          // △△△△△ 7 --> 8 △△△△△
                // xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;  // △△△△△ 最后新添加了深度值depth △△△△△
                // image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);      // △△△△△ 最后这个值加了深度 △△△△△

                Eigen::Matrix<double, 6, 1> xyz_uv_depth;
                xyz_uv_depth << x, y, z, p_u, p_v, depth;
                image[feature_id].emplace_back(camera_id, xyz_uv_depth);

            }

            // map<int, vector<pair<int, Vector4d>>> lines1;
            map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>> lines;
            for (unsigned int i = 0; i < line_msg->points.size(); i ++)
            {
                int v = line_msg->channels[0].values[i] + 0.5;
                // std::cout << "receive id: " << v / NUM_OF_CAM << "\n";
                int feature_id = v / NUM_OF_CAM;
                int camera_id  = v % NUM_OF_CAM;                // 被几号相机观测到的，如果是单目，camera_id = 0
                double x_startpoint = line_msg->points[i].x;
                double y_startpoint = line_msg->points[i].y;
                double x_endpoint = line_msg->channels[1].values[i];
                double y_endpoint = line_msg->channels[2].values[i];
                double depth_of_startpt = line_msg->channels[3].values[i];  // 线起始点的深度值
                double depth_of_endpt   = line_msg->channels[4].values[i];  // 线起始点的深度值
                // ROS_ASSERT(z == 1);
                // lines[feature_id].emplace_back(camera_id, Vector4d(x_startpoint, y_startpoint, x_endpoint, y_endpoint));

                Eigen::Matrix<double, 6, 1> x_startd_endd;
                x_startd_endd << x_startpoint, y_startpoint, depth_of_startpt, x_endpoint, y_endpoint, depth_of_endpt;
                lines[feature_id].emplace_back(camera_id,x_startd_endd);
            }

            // estimator.processImage1(image, img_msg->header);
            estimator.processImage(image, lines, img_msg->header);


            // 一些打印以及topic的发送
            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";
            cur_header = header;

            m_loop_drift.lock();
            // utility/visualization.cpp
            // pubOdometry(estimator, header);
            // pubKeyPoses(estimator, header);
            // pubCameraPose(estimator, header);
            // pubPointCloud(estimator, header);
            // pubTF(estimator, header);
            pubOdometry(estimator, header, relocalize_t, relocalize_r);
            pubKeyPoses(estimator, header, relocalize_t, relocalize_r);
            pubCameraPose(estimator, header, relocalize_t, relocalize_r);
            pubLinesCloud(estimator, header, relocalize_t, relocalize_r);
            pubPointCloud(estimator, header, relocalize_t, relocalize_r);
            pubTF(estimator, header, relocalize_t, relocalize_r);

            pubKeyframe(estimator); // △△△△△ zlc添加：发布关键帧, pose_graph_node.cpp的main()中会订阅发布的关键帧位姿和关键帧上的特征点 △△△△△

            if (relo_msg != NULL)
                pubRelocalization(estimator);

            m_loop_drift.unlock();
            // ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }

        m_estimator.unlock();       // △△△△△ zlc添加：PL-VINS中将这个锁删掉了 △△△△△

        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();

#ifdef Compressed
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
#endif
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    // 注册一些publisher
    registerPub(n);

    // △△△△△ zlc添加：FAST中添加的代码 △△△△△
#ifdef Compressed
    message_filters::Subscriber<sensor_msgs::CompressedImage> sub_image(n, IMAGE_TOPIC + "/compressed", 1000);
    message_filters::Subscriber<sensor_msgs::CompressedImage> sub_depth(n, DEPTH_TOPIC + "/compressedDepth", 1000);
    // use ApproximateTime to fit fisheye camera
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(100), sub_image, sub_depth);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));
#else
    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, IMAGE_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(n, DEPTH_TOPIC, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_image, sub_depth);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));           // 调用img回调函数
#endif

    // 接受imu消息                                          2000->1000
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 1000, imu_callback, ros::TransportHints().tcpNoDelay());

    // 接受前端视觉光流结果
    // ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    // 接受前端线段特征结果
    ros::Subscriber sub_linefeature = n.subscribe("/linefeature_tracker/linefeature", 2000, linefeature_callback);
    // ros::Subscriber sub_raw_image = n.subscribe(IMAGE_TOPIC, 2000, raw_image_callback);

    // ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    // thread: visual-inertial odometry


    // 接受前端重启命令   PL-VINS中已经删除这部分
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    // 回环检测的fast relocalization响应  topic from pose_graph, notify if there's relocalization    PL-VINS中已经删除这部分
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    // 核心处理线程
    std::thread trackThread{process_tracker};
    std::thread measurement_process{process};

    ros::spin();
    // ros::MultiThreadedSpinner spinner(4);        // △ zlc：FAST中使用
    // spinner.spin();                              // △ zlc：FAST中使用

    return 0;
}
