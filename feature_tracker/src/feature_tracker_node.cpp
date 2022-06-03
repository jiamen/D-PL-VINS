#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include <message_filters/time_synchronizer.h>      // △△△△△ 新添加的头文件 △△△△△
#include <message_filters/synchronizer.h>           // △△△△△ 新添加的头文件 △△△△△
#include <message_filters/sync_policies/approximate_time.h>     // △△△△△ 新添加的头文件 △△△△△


#include <thread>               // △△△△△ FAST新添加的头文件 △△△△△
#include <mutex>                // △△△△△ FAST新添加的头文件 △△△△△
#include <condition_variable>   // △△△△△ FAST新添加的头文件 △△△△△


#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img, pub_match;
ros::Publisher pub_restart;                 // △ zlc：PL-VINS中没有 △

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;


double last_image_time = 0;                 // △ zlc：PL-VINS中没有
bool init_pub = 0;                          // △ zlc：PL-VINS中没有

double frame_cnt = 0;                       // △ zlc：PL-VINS添加
double sum_time = 0.0;                      // △ zlc：PL-VINS添加
double mean_time = 0.0;                     // △ zlc：PL-VINS添加





/**
 * @brief 根据当前imu数据预测当前位姿
 *      使用mid-point方法对imu状态量进行预测
 * @param[in] imu_msg
 */
Matrix3d relative_R = Matrix3d::Identity();
double latest_time;
bool init_imu = 1;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;


/*
double last_imu_t = 0;          // △ zlc添加：上一帧IMU的时间
std::mutex m_buf;               // △ zlc添加：添加线程操作
std::mutex m_state;
queue<sensor_msgs::ImuConstPtr> imu_buf;
std::condition_variable con;

void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)    // △ zlc添加：时间戳错乱
    {
        ROS_WARN("imu message in disorder! %f", imu_msg->header.stamp.toSec());
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    // cl:pub "imu_propagate"
    {
        std::lock_guard<std::mutex> lg(m_state);
        //predict imu (no residual error)
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }

}
*/


/*
// △△△△△ zlc：原来的图像的回调函数 △△△△△
void img_callback(const sensor_msgs::ImageConstPtr& img_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
    }

    // frequency control, 如果图像频率低于一个值
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat show_img = ptr->image;

    TicToc t_r;
    frame_cnt ++;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)     // 单目 或者 非双目,   #### readImage： 这个函数里 做了很多很多事, 提取特征 ####
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)));   // rowRange(i,j) 取图像的i～j行
        else
        {
            if (EQUALIZE)      // 对图像进行直方图均衡化处理
            {
                std::cout <<" EQUALIZE " << std::endl;
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
//        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
        trackerData[i].showUndistortion();
#endif
    }

    // 双目
    if ( PUB_THIS_FRAME && STEREO_TRACK && trackerData[0].cur_pts.size() > 0)
    {
        pub_count++;
        r_status.clear();
        r_err.clear();
        TicToc t_o;
        cv::calcOpticalFlowPyrLK(trackerData[0].cur_img, trackerData[1].cur_img, trackerData[0].cur_pts, trackerData[1].cur_pts, r_status, r_err, cv::Size(21, 21), 3);
        ROS_DEBUG("spatial optical flow costs: %fms", t_o.toc());
        vector<cv::Point2f> ll, rr;
        vector<int> idx;
        for (unsigned int i = 0; i < r_status.size(); i++)
        {
            if (!inBorder(trackerData[1].cur_pts[i]))
                r_status[i] = 0;

            if (r_status[i])
            {
                idx.push_back(i);

                Eigen::Vector3d tmp_p;
                trackerData[0].m_camera->liftProjective(Eigen::Vector2d(trackerData[0].cur_pts[i].x, trackerData[0].cur_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                ll.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));

                trackerData[1].m_camera->liftProjective(Eigen::Vector2d(trackerData[1].cur_pts[i].x, trackerData[1].cur_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                rr.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));
            }
        }
        if (ll.size() >= 8)
        {
            vector<uchar> status;
            TicToc t_f;
            cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 1.0, 0.5, status);
            ROS_DEBUG("find f cost: %f", t_f.toc());
            int r_cnt = 0;
            for (unsigned int i = 0; i < status.size(); i++)
            {
                if (status[i] == 0)
                    r_status[idx[i]] = 0;
                r_cnt += r_status[idx[i]];
            }
        }
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)                   // 单目 或者 非双目
                completed |= trackerData[j].updateID(i);   // 更新检测到的特征的id, 如果一个特征是新的，那就赋予新的id，如果一个特征已经有了id，那就不处理
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;   //  feature id
        sensor_msgs::ChannelFloat32 u_of_point;    //  u
        sensor_msgs::ChannelFloat32 v_of_point;    //  v

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            if (i != 1 || !STEREO_TRACK)  // 单目
            {
                auto un_pts = trackerData[i].undistortedPoints();
                auto &cur_pts = trackerData[i].cur_pts;
                auto &ids = trackerData[i].ids;
                for (unsigned int j = 0; j < ids.size(); j++)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    ROS_ASSERT(inBorder(cur_pts[j]));
                }
            }
            else if (STEREO_TRACK)
            {
                auto r_un_pts = trackerData[1].undistortedPoints();
                auto &ids = trackerData[0].ids;
                for (unsigned int j = 0; j < ids.size(); j++)
                {
                    if (r_status[j])
                    {
                        int p_id = ids[j];
                        hash_ids[i].insert(p_id);
                        geometry_msgs::Point32 p;
                        p.x = r_un_pts[j].x;
                        p.y = r_un_pts[j].y;
                        p.z = 1;

                        feature_points->points.push_back(p);
                        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    }
                }
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        pub_img.publish(feature_points);

//        if (SHOW_TRACK)  // SHOW_TRACK
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);

            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);
//                tmp_img = trackerData[0].cur_img;
                cv::cvtColor(trackerData[0].cur_img, tmp_img, CV_GRAY2RGB);
                if (i != 1 || !STEREO_TRACK)
                {
                    for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                    {
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                        //char name[10];
                        //sprintf(name, "%d", trackerData[i].ids[j]);
                        //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                    }
                }
                else
                {
                    for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                    {
                        if (r_status[j])
                        {
                            cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(0, 255, 0), 2);
                            cv::line(stereo_img, trackerData[i - 1].cur_pts[j], trackerData[i].cur_pts[j] + cv::Point2f(0, ROW), cv::Scalar(0, 255, 0));
                        }
                    }
                }
            }

            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);

            pub_match.publish(ptr->toImageMsg());
        }
    }
    sum_time += t_r.toc();
    mean_time = sum_time/frame_cnt;
//    ROS_INFO("whole point feature tracker processing costs: %f", t_r.toc());
//    ROS_INFO("whole point feature tracker processing costs: %f", mean_time);  // zlc删除

}
*/


// △△△△△ 新添加的深度信息，图像的回调函数 △△△△△
void img_callback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
    if(first_image_flag)        // 对第一帧图像的基本操作
    {
        first_image_flag = false;
        first_image_time = color_msg->header.stamp.toSec();
        last_image_time  = color_msg->header.stamp.toSec();     // △ zlc：原本PL-VINS中删除
        return;                                                 // △ zlc：原本PL-VINS中删除
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

        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);              // 告诉其他模块要重启了
        return;
    }
    last_image_time = color_msg->header.stamp.toSec();  // 更新上一帧图像时间


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
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), depth_ptr->image.rowRange(ROW * i, ROW * (i + 1)), color_msg->header.stamp.toSec());
            // rowRange(i,j) 取图像的i～j行
        }
        else
        {
            if (EQUALIZE)   // 对图像进行直方图均衡化处理
            {
                std::cout <<" EQUALIZE " << std::endl;
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
            {
                trackerData[i].cur_img   = ptr->image.rowRange(ROW * i, ROW * (i + 1));
                trackerData[i].cur_depth = depth_ptr->image.rowRange(ROW * i, ROW * (i + 1));   // △△△△△ 先添加的深度数据 △△△△△
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
                completed |= trackerData[j].updateID(i);    // 单目的情况下可以直接用=号
        if (!completed)
            break;
    }

    // 给后端喂数据
    if (PUB_THIS_FRAME)
    {
        frame_cnt ++;

        //vector<int> test;
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
            auto &un_pts  = trackerData[i].cur_un_pts;          // 去畸变的归一化相机坐标系
            auto &cur_pts = trackerData[i].cur_pts;             // 像素坐标
            auto &ids = trackerData[i].ids;                             // id
            auto &pts_velocity = trackerData[i].pts_velocity;   // 归一化坐标下的速度
            for (unsigned int j = 0; j < ids.size(); j ++)
            {
                // 只发布追踪大于1的，因为等于1没法构成重投影约束，也没法三角化
                if (trackerData[i].track_cnt[j] > 1)
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
                            (int)show_depth.at<unsigned short>(round(cur_pts[j].y), round(cur_pts[j].x)));
                    // debug use: print depth pixels
                    // test.push_back((int)show_depth.at<unsigned short>(round(cur_pts[j].y),round(cur_pts[j].x)));
                }
            }
        }
        // debug use: print depth pixels
        // for (int iii = test.size() - 1; iii >= 0; iii--)
        // {
        //     std::cout << test[iii] << " ";
        // }
        // std::cout << std::endl;
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
            pub_img.publish(feature_points);   // "feature"  前端得到的特征点信息通过这个publisher发布出去
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

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j ++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
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
            }
            // cv::imshow("vis", stereo_img);
            // cv::waitKey(5);
            pub_match.publish(ptr->toImageMsg());       // △△△△△ 这里才正式的发布图片消息 △△△△△
        }
    }

    sum_time += t_r.toc();
    mean_time = sum_time/frame_cnt;

    std::cout << "!!!!!!!!!shi-thomas feature cost: " << mean_time << endl;

    // ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
    // ROS_INFO("whole point feature tracker processing costs: %f", mean_time);  // zlc删除

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "feature_tracker");       // ros节点初始化
    ros::NodeHandle n("~");                           // 声明一个句柄，～代表这个节点的命名空间
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);    // 设置ros log级别

    readParameters(n);      // 读取配置文件   paremeters.cpp

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

    // 这个向roscore注册订阅这个topic，收到一次message就执行一次回调函数
    // 源代码是：
    // ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    // △ zlc：下面是zlc添加
    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, IMAGE_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(n, DEPTH_TOPIC, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_image, sub_depth);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));           // 调用img回调函数

    // △ zlc：下面是zlc添加，思路来自于FAST部分
    // ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 1000, imu_callback, ros::TransportHints().tcpNoDelay());

    pub_img   = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart", 1000);     // △ zlc添加 △

    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */

    ros::spin();        // spin代表这个节点开始循环查询topic是否接收

    return 0;
}
