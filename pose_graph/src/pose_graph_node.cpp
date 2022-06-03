#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>         // zlc新添加的头文件
#include <message_filters/time_synchronizer.h>  // zlc新添加的头文件


#include <message_filters/synchronizer.h>           // △△△△△ zlc新添加的头文件 △△△△△
#include <message_filters/sync_policies/approximate_time.h>     // △△△△△ zlc新添加的头文件 △△△△△


#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#define SKIP_FIRST_CNT 10
using namespace std;

#define RGBD2
queue<sensor_msgs::ImageConstPtr> image_buf;
queue<sensor_msgs::ImageConstPtr> depth_buf;            // zlc新添加的深度信息变量
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag  = 0;
bool start_flag = 0;
double SKIP_DIS = 0;


float PCL_MAX_DIST, PCL_MIN_DIST, RESOLUTION;           // △△△△△ zlc新添加的变量 △△△△△
int U_BOUNDARY, D_BOUNDARY, L_BOUNDARY, R_BOUNDARY;     // △△△△△ zlc新添加的变量 △△△△△


int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int PCL_DIST;                           // △△△△△ zlc新添加的变量 △△△△△
int DEBUG_IMAGE;
int VISUALIZE_IMU_FORWARD;
int LOOP_CLOSURE;
int FAST_RELOCALIZATION;


camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;


Eigen::Matrix<double, 3, 1> ti_d;       // △△△△△ zlc新添加的变量 △△△△△
Eigen::Matrix<double, 3, 3> qi_d;       // △△△△△ zlc新添加的变量 △△△△△


ros::Publisher pub_match_img;
ros::Publisher pub_match_points;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_key_odometrys;
ros::Publisher pub_vio_path;
nav_msgs::Path no_loop_path;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;


double normofTransform(cv::Mat rvec, cv::Mat tvec)
{
    return fabs( min(cv::norm(rvec), 2*M_PI - cv::norm(rvec)) ) + fabs(cv::norm(tvec));
}


/**
 * @brief 新建一个sequence
 *
 */
void new_sequence()
{
    printf("new sequence\n");
    sequence ++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();


    m_buf.lock();

    while(!image_buf.empty())
        image_buf.pop();
#ifdef RGBD2
    while(!depth_buf.empty())     // △ zlc添加
        depth_buf.pop();          // △ zlc添加
#endif
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}


/**
 * @brief 图像的回调函数，暂时没有添加深度信息
 *
 */
#ifdef RGBD2
    void image_callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::ImageConstPtr& depth_msg)
#else
    void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
#endif
{
    // ROS_INFO("image_callback!");
    if(!LOOP_CLOSURE)
        return;

    m_buf.lock();
    image_buf.push(image_msg);
#ifdef RGBD2
    depth_buf.push(depth_msg);   // △ zlc新添加的函数变量
#endif
    m_buf.unlock();
    // printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();

    // 检查时间戳是否错乱以及延时过大
    else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 || image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! detect a new sequence!");
        std::cout << "image discontinue! detect a new sequence!\n";
        new_sequence();             // 如果发生了就新建一个序列 TODO: sequense 是否可以合并？
        // △△△△△ zlc添加 : 以时间判断为基准，只要错误了就新建一个序列，这个序列是新的时间序列，包含错误时间后的所有关键帧？？
        // 只要时间戳不错误，就只有一个时间序列  ？？？？？
    }

    last_image_time = image_msg->header.stamp.toSec();
}


/**
 * @brief 点云的回调函数，  VIO中KF关于地图点的信息
 *
 */
void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    // ROS_INFO("point_callback!");
    if(!LOOP_CLOSURE)
        return;

    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x,
                                                     point_msg->points[i].y,
                                                     point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],
                                                     point_msg->channels[i].values[1]);
    }
    */
}


/**
 * @brief VIO结点KF的信息
 *
 * @param[in] pose_msg
 */
void pose_callback(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
    // ROS_INFO("pose_callback!");
    if(!LOOP_CLOSURE)
        return;

    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}


// not used
/**
 * @brief 发布经过回环修正后的最新的位姿
 *
 * @param[in] forward_msg
 */
void imu_forward_callback(const nav_msgs::Odometry::ConstPtr& forward_msg)
{
    if (VISUALIZE_IMU_FORWARD)
    {
        Vector3d vio_t(forward_msg->pose.pose.position.x, forward_msg->pose.pose.position.y, forward_msg->pose.pose.position.z);
        Quaterniond vio_q;
        vio_q.w() = forward_msg->pose.pose.orientation.w;
        vio_q.x() = forward_msg->pose.pose.orientation.x;
        vio_q.y() = forward_msg->pose.pose.orientation.y;
        vio_q.z() = forward_msg->pose.pose.orientation.z;

        vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
        vio_q = posegraph.w_r_vio * vio_q;

        vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
        vio_q = posegraph.r_drift * vio_q;

        Vector3d vio_t_cam;
        Quaterniond vio_q_cam;
        vio_t_cam = vio_t + vio_q * tic;
        vio_q_cam = vio_q * qic;

        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, forward_msg->header);
    }
}


/**
 * @brief 利用VIO重定位的结果进行修正
 *
 */
void relo_relative_pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    // T_loop_cur
    Vector3d relative_t = Vector3d(pose_msg->pose.pose.position.x,
                                   pose_msg->pose.pose.position.y,
                                   pose_msg->pose.pose.position.z);
    Quaterniond relative_q;
    relative_q.w() = pose_msg->pose.pose.orientation.w;
    relative_q.x() = pose_msg->pose.pose.orientation.x;
    relative_q.y() = pose_msg->pose.pose.orientation.y;
    relative_q.z() = pose_msg->pose.pose.orientation.z;
    double relative_yaw = pose_msg->twist.twist.linear.x;
    int index = pose_msg->twist.twist.linear.y;             // 当前帧的idx

    // printf("receive index %d \n", index );
    Eigen::Matrix<double, 8, 1 > loop_info;
    loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
            relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
            relative_yaw;
    posegraph.updateKeyFrameLoop(index, loop_info);

}


/**
 * @brief 接受的VIO滑窗中最新的位姿，不一定是KF
 *        这里做的都是可视化相关的内容
 *
 */
void vio_callback(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
    // ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio * vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;

    if (!VISUALIZE_IMU_FORWARD)
    {
        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
    }

    odometry_buf.push(vio_t_cam);
    if (odometry_buf.size() > 10)
    {
        odometry_buf.pop();
    }

    visualization_msgs::Marker key_odometrys;
    key_odometrys.header = pose_msg->header;
    key_odometrys.header.frame_id = "world";
    key_odometrys.ns = "key_odometrys";
    key_odometrys.type = visualization_msgs::Marker::SPHERE_LIST;
    key_odometrys.action = visualization_msgs::Marker::ADD;
    key_odometrys.pose.orientation.w = 1.0;
    key_odometrys.lifetime = ros::Duration();

    // static int key_odometrys_id = 0;
    key_odometrys.id = 0;               // key_odometrys_id++;
    key_odometrys.scale.x = 0.1;
    key_odometrys.scale.y = 0.1;
    key_odometrys.scale.z = 0.1;
    key_odometrys.color.r = 1.0;
    key_odometrys.color.a = 1.0;

    for (unsigned int i = 0; i < odometry_buf.size(); i ++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d vio_t;
        vio_t = odometry_buf.front();
        odometry_buf.pop();
        pose_marker.x = vio_t.x();
        pose_marker.y = vio_t.y();
        pose_marker.z = vio_t.z();
        key_odometrys.points.push_back(pose_marker);
        odometry_buf.push(vio_t);
    }
    pub_key_odometrys.publish(key_odometrys);

    // not used
    if (!LOOP_CLOSURE)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = pose_msg->header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = vio_t.x();
        pose_stamped.pose.position.y = vio_t.y();
        pose_stamped.pose.position.z = vio_t.z();
        no_loop_path.header = pose_msg->header;
        no_loop_path.header.frame_id = "world";
        no_loop_path.poses.push_back(pose_stamped);
        pub_vio_path.publish(no_loop_path);
    }
}


/**
 * @brief 实时更新外参
 *
 * @param[in] pose_msg
 */
void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}


/**
 * @brief 回环检测主要处理函数
 *
 */
void process()
{
    if (!LOOP_CLOSURE)      // 不检测回环就啥都不干
        return;

    while (true)
    {
        sensor_msgs::ImageConstPtr image_msg = NULL;
#ifdef RGBD2
        sensor_msgs::ImageConstPtr depth_msg = NULL;         // zlc新添加的变量
#endif
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;

        // find out the messages with same time stamp
        m_buf.lock();

        // get image_msg, pose_msg and point_msg
        // 做一个时间戳对齐，涉及到原图，KF位姿以及KF对应地图点
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            // 上面确保了image_buf <= point_buf && image_buf <= pose_buf
            // 下面根据pose时间找时间戳同步的原图和地图点
            else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec()
                     && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();    // 取出来pose
                pose_buf.pop();
                while (!pose_buf.empty())       // 清空所有的pose，回环的帧率慢一些没关系
                    pose_buf.pop();
                // 找到对应pose的原图
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                {
                    image_buf.pop();
#ifdef RGBD2
                    depth_buf.pop();            // △ zlc添加变量
#endif
                }

                image_msg = image_buf.front();
                image_buf.pop();
#ifdef RGBD2
                depth_msg = depth_buf.front();      // △△△△△ zlc新添加的变量 △△△△△
                depth_buf.pop();                    // △△△△△ zlc新添加的变量 △△△△△
#endif

                // 找到对应的地图点
                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();      // △△△△△△ zlc：与后端优化窗口中的倒数第2关键帧有关的地图点，
                point_buf.pop();                    // 但是这里比较奇怪的是点消息的坐标描述不是在倒数第2关键帧下，
                                                    // 而是在第一个观测（起始观测帧）到它的帧坐标系下  和  9（windows_size-2） -起始帧两帧下描述
            }
        }
        m_buf.unlock();


        // 至此取出了时间戳同步的原图，KF和地图点信息
        if (pose_msg != NULL)       // 判断一下是否有效
        {
            // printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            // printf(" point time %f \n", point_msg->header.stamp.toSec());
            // printf(" image time %f \n", image_msg->header.stamp.toSec());
            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)    // 跳过最开始的SKIP_FIRST_CNT帧
            {
                skip_first_cnt ++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)        // 降频，每隔SKIP_CNT帧处理一次
            {
                skip_cnt ++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            // 通过cvbridge得到opencv格式的图像
            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);


            // △△△△△ zlc下面是新添加的部分 △△△△△
            // depth has encoding TYPE_16UC1

            cv_bridge::CvImageConstPtr depth_ptr;
            // debug use     std::cout<<depth_msg->encoding<<std::endl;
            {
                sensor_msgs::Image img;
                img.header = depth_msg->header;
                img.height = depth_msg->height;
                img.width = depth_msg->width;
                img.is_bigendian = depth_msg->is_bigendian;
                img.step = depth_msg->step;
                img.data = depth_msg->data;
                img.encoding = sensor_msgs::image_encodings::MONO16;
                depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
            }
            //  △△△△△ 上面是新添加的部分 △△△△△



            cv::Mat image = ptr->image;
#ifdef RGBD2
            cv::Mat depth = depth_ptr->image;       // △△△△△ zlc新添加的变量，对应上面新添加的部分 △△△△△
#endif
            // build keyframe  得到KF的位姿，转成eigen格式
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();

            if((T - last_t).norm() > SKIP_DIS)              // 要求KF相隔必要的平移距离
            {
                vector<cv::Point3f> point_3d;               // VIO世界坐标系下的地图点坐标
                vector<cv::Point2f> point_2d_uv;            // 归一化相机坐标系的坐标
                vector<cv::Point2f> point_2d_normal;        // 像素坐标
#ifdef RGBD2
                vector<cv::Point3f> point_3d_depth;         // △△△△△ zlc新添加的变量 △△△△△
#endif
                vector<double> point_id;                    // 地图点的idx

                // 遍历所有的地图点
                for (unsigned int i = 0; i < point_msg->points.size(); i ++)
                {
                    // 世界坐标系下 的 三维特征点
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);               // 转成eigen的格式

                    // 这里是接收的estimator部分的pubKeyframe()函数发布的消息，
                    // 这里的点     前两位是相机坐标系下的归一化点坐标，    后两位是像素坐标系下的像素点位置
                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    // printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }


                // △△△△△ zlc下面是新添加的部分：深度图部分下采样：480*640=30,0000  ==》 48*64=3,000  △△△△△
                // ROW: 480 y  COL: 640 x  L_BOUNDARY=40, R_BOUNDARY=40
                // debug: int count_ = 0;
                for (int i = L_BOUNDARY; i < COL - R_BOUNDARY; i += PCL_DIST)
                {
                    for (int j = U_BOUNDARY; j < ROW - D_BOUNDARY; j += PCL_DIST)
                    {
                        Eigen::Vector2d a(i, j);
                        Eigen::Vector3d b;
                        // depth is aligned
                        m_camera->liftProjective(a, b);
                        float depth_val = ((float)depth.at<unsigned short>(j, i)) / 1000.0;
                        if (depth_val > PCL_MIN_DIST && depth_val < PCL_MAX_DIST)   // PCL_MIN_DIST=0.3，PCL_MAX_DIST=6
                        {
                            // debug: ++ count_;
                            point_3d_depth.push_back(cv::Point3f(b.x() * depth_val, b.y() * depth_val, depth_val));
                        }
                    }
                }
                // debug: ROS_WARN("Depth points count: %d", count_);
                // △△△△△ 上面是新添加的部分  △△△△△



                // 创建回环检测节点的KF，通过frame_index标记对应帧
                // add sparse depth img to this class
#ifdef RGBD2
                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                                  point_3d_depth,
                                                  point_3d, point_2d_uv, point_2d_normal, point_id, sequence); // △△△△△ zlc上面是新添加的变量 △△△△△
#else
                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                                  point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
#endif
                m_process.lock();
                start_flag = 1;
                posegraph.addKeyFrame(keyframe, 1);     // △ zlc添加：回环检测核心入口函数 △
                m_process.unlock();
                frame_index ++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}


/**
 * @brief 接受相关键盘指令
 *
 */
void command()
{
    if (!LOOP_CLOSURE)
        return;
    while(1)
    {
        char c = getchar();
        if (c == 's')           // s就是存储地图
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            printf("program shutting down...\n");    // △△△△△ zlc新添加的变量 △△△△△
            ros::shutdown();                         // △△△△△ zlc这句在PL-VINS中注释掉了 △△△△△
        }
        if (c == 'n')           // n就是新建一个sequence
            new_sequence();


        // △△△△△ zlc下面是新添加的部分 △△△△△
        if (c == 'p')
        {
            TicToc t_filter;
            posegraph.pclFilter(false);
            printf("pclFilter time: %f", t_filter.toc());
        }
        if (c == 'd')
        {
            TicToc t_pcdfile;
            posegraph.save_cloud->width = posegraph.save_cloud->points.size();
            posegraph.save_cloud->height = 1;
            pcl::io::savePCDFileASCII("/home/zlc/catkin_ws/src/VINS-RGBD/output/pcd_file_"+to_string(frame_index)+"keyframes.pcd", *(posegraph.save_cloud));
            printf("Save pcd file done! Time cost: %f", t_pcdfile.toc());
        }
        // △△△△△ 上面是新添加的部分 △△△△△



        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_graph");
    ros::NodeHandle n("~");
    posegraph.registerPub(n);

    // read param  这四个值在launch文件中指定
    n.getParam("visualization_shift_x", VISUALIZATION_SHIFT_X);     // 这两个shift基本都是0
    n.getParam("visualization_shift_y", VISUALIZATION_SHIFT_Y);
    n.getParam("skip_cnt", SKIP_CNT);       // 跳过前SKIP_CNT帧
    n.getParam("skip_dis", SKIP_DIS);       // 两帧距离门限

    std::string config_file;
    n.getParam("config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // 可视化的参数
    double camera_visual_size = fsSettings["visualize_camera_size"];
    cameraposevisual.setScale(camera_visual_size);          // camera_visual_size = 0.4
    cameraposevisual.setLineWidth(camera_visual_size / 10.0);


    // 是否进行回环检测的标识
    LOOP_CLOSURE = fsSettings["loop_closure"];
    std::string IMAGE_TOPIC;
#ifdef RGBD2
    std::string DEPTH_TOPIC;                    // △△△△△ zlc新添加的变量 △△△△△
#endif
    int LOAD_PREVIOUS_POSE_GRAPH;

    // prepare for loop closure (load vocabulary, set topic, etc)
    if (LOOP_CLOSURE)
    {
        ROW = fsSettings["image_height"];           // 图片分辨率
        COL = fsSettings["image_width"];


        PCL_DIST   = fsSettings["pcl_dist"];        // △△△△△ zlc新添加的变量:  pcl_dist = 10  △△△△△
        U_BOUNDARY = fsSettings["u_boundary"];      // △△△△△ zlc新添加的变量:  u_boundary: 10 △△△△△
        D_BOUNDARY = fsSettings["d_boundary"];      // △△△△△ zlc新添加的变量:  d_boundary: 10 △△△△△
        L_BOUNDARY = fsSettings["l_boundary"];      // △△△△△ zlc新添加的变量:  l_boundary: 40 △△△△△
        R_BOUNDARY = fsSettings["r_boundary"];      // △△△△△ zlc新添加的变量:  r_boundary: 40 △△△△△
        PCL_MIN_DIST = fsSettings["pcl_min_dist"];  // △△△△△ zlc新添加的变量:  pcl_min_dist: 0.3 △△△△△
        PCL_MAX_DIST = fsSettings["pcl_max_dist"];  // △△△△△ zlc新添加的变量:  pcl_max_dist: 6 △△△△△
        RESOLUTION   = fsSettings["resolution"];    // △△△△△ zlc新添加的变量:  resolution: 0.02 △△△△△


        // △△△△△ 下面是新添加的部分 △△△△△
#ifdef RGBD2
        // OctreePointCloudDensity has no ::Ptr
        posegraph.octree = new pcl::octree::OctreePointCloudDensity<pcl::PointXYZ>(RESOLUTION);
        posegraph.cloud  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        posegraph.save_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        posegraph.octree->setInputCloud(posegraph.cloud);
        posegraph.octree->addPointsFromInputCloud();
        // in pcl 1.8.0+, need to set bbox (isVoxelOccupiedAtPoint will check bbox)
        posegraph.octree->defineBoundingBox(-100, -100, -100, 100, 100, 100);
        // △△△△△ 上面是新添加的部分 △△△△△
#endif

        std::string pkg_path = ros::package::getPath("pose_graph");
        string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";    // 训练好的二进制词袋的路径
        cout << "vocabulary_file" << vocabulary_file << endl;
        posegraph.loadVocabulary(vocabulary_file);                          // 加载二进制词袋

        BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";      // 计算描述子pattern的文件
        cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;
        // 和前面一样，生成一个相机模型
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

        fsSettings["image_topic"] >> IMAGE_TOPIC;                       // 原图的topic
#ifdef RGBD2
        fsSettings["depth_topic"] >> DEPTH_TOPIC;                    // △△△△△ zlc新添加的部分 △△△△△
#endif
        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        fsSettings["output_path"] >> VINS_RESULT_PATH;                  // 这里是导入储存地址
        fsSettings["save_image"] >> DEBUG_IMAGE;         // 这里值是1

        // create folder if not exists
        // FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
        // FileSystemHelper::createDirectoryIfNotExists(VINS_RESULT_PATH.c_str());


        // △△△△△ zlc新添加的部分 △△△△△
        cv::Mat cv_qid, cv_tid;
        fsSettings["extrinsicRotation"] >> cv_qid;
        fsSettings["extrinsicTranslation"] >> cv_tid;
        cv::cv2eigen(cv_qid, qi_d);
        cv::cv2eigen(cv_tid, ti_d);
        // △△△△△ 上面是新添加的部分 △△△△△


        VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];            // 可视化是否使用imu进行前推
        LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];      // 是否加载已有地图, = 0
        FAST_RELOCALIZATION = fsSettings["fast_relocalization"];                // 是否快速重定位，这个和VIO结点有交互
        // VINS_RESULT_PATH = VINS_RESULT_PATH + "/vins_result_loop.csv";
        VINS_RESULT_PATH = VINS_RESULT_PATH + "/stamped_pose_graph_estimate.txt";
        std::ofstream fout(VINS_RESULT_PATH, std::ios::out);                 // zlc：PLVINS中注释，RGBD中保留
        fout.close();
        fsSettings.release();

        // 这里的轨迹，PL-VINS中添加了如下的语句进行评测数据保存
        // std::ofstream foutC("/home/zlc/cpl-tmp_ws/src/PL-VINS/Trajactory/new_tum_evo/tum_fast_plvins_loop1.txt", std::ios::out);
        // std::ofstream foutC("/home/zlc/cpl-tmp_ws/src/PL-VINS/Trajactory/new_tum_evo/stamped_pose_graph_estimate.txt", std::ios::out);
        // foutC.close();
        //std::ofstream foutC1("/home/zlc/cpl-tmp_ws/src/PL-VINS/Trajactory/new_tum_evo/evo_fast_plvins_loop1.txt", std::ios::out);
        //foutC1.close();


        // not used
        if (LOAD_PREVIOUS_POSE_GRAPH)
        {
            printf("load pose graph\n");
            m_process.lock();
            posegraph.loadPoseGraph();              // △ zlc添加：这个函数不会被运行
            m_process.unlock();
            printf("load pose graph finish\n");
            load_flag = 1;
        }
        else
        {
            printf("no previous pose graph\n");
            load_flag = 1;
        }
    }

    fsSettings.release();

    // publish camera pose by imu propagate and odometry (Ps and Rs of curr frame)
    // not important
    ros::Subscriber sub_imu_forward = n.subscribe("/plvins_estimator/imu_propagate", 2000, imu_forward_callback);
    // odometry_buf
    ros::Subscriber sub_vio = n.subscribe("/plvins_estimator/odometry", 2000, vio_callback);


    // △△△△△ 下面是新添加的部分,针对sub_image做了优化，添加了深度图及两部分的同步操作 △△△△△
    // get image msg, store in image_buf
    // ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 2000, image_callback);
    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, IMAGE_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(n, DEPTH_TOPIC, 1);
    // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub_image, sub_depth, 2000);
    // fit fisheye camera
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_image, sub_depth);
    sync.registerCallback(boost::bind(&image_callback, _1, _2));
    // △△△△△ 上面是新添加的部分 △△△△△


    ros::Subscriber sub_pose = n.subscribe("/plvins_estimator/keyframe_pose", 2000, pose_callback);
    // get extrinsic (ric qic  odometry.pose.pose.position and odometry.pose.pose.orientation)
    // update tic and qic real-time
    ros::Subscriber sub_extrinsic = n.subscribe("/plvins_estimator/extrinsic", 2000, extrinsic_callback);
    // get keyframe_point(pointcloud), store in point_buf (marginalization_flag == 0)
    ros::Subscriber sub_point = n.subscribe("/plvins_estimator/keyframe_point", 2000, point_callback);


    // do relocalization here.    回环部分，订阅匹配图像及重定位位姿
    // pose_graph publish match_points to vins_estimator, estimator then publish relo_relative_pose
    ros::Subscriber sub_relo_relative_pose = n.subscribe("/plvins_estimator/relo_relative_pose", 2000, relo_relative_pose_callback);

    pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_key_odometrys = n.advertise<visualization_msgs::Marker>("key_odometrys", 1000);
    // not used
    pub_vio_path = n.advertise<nav_msgs::Path>("no_loop_path", 1000);
    pub_match_points = n.advertise<sensor_msgs::PointCloud>("match_points", 100);

    std::thread measurement_process;
    std::thread keyboard_command_process;

    // main thread
    measurement_process = std::thread(process);
    // zlc添加：not used：这个多线程函数实际并没有使用
    keyboard_command_process = std::thread(command);


    ros::spin();

    return 0;
}