#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>


#include <message_filters/subscriber.h>


#include <message_filters/time_synchronizer.h>              // △△△△△ zlc新添加
#include <message_filters/synchronizer.h>                   // △△△△△ zlc新添加
#include <message_filters/sync_policies/approximate_time.h> // △△△△△ zlc新添加


#include "./linefeature_tracker.h"                    // △ zlc添加深度点

// #include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img_line, pub_match_line;        // zlc改名

LineFeatureTracker trackerData;
double first_image_time;
double last_image_time = 0;     // △ zlc添加

int pub_count = 1;
bool first_image_flag = true;

double frame_cnt = 0;
double sum_time  = 0.0;
double mean_time = 0.0;


// void img_callback(const sensor_msgs::ImageConstPtr& img_msg)
void img_callback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
    if(first_image_flag)        // 对第一帧图像的基本操作
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time  = img_msg->header.stamp.toSec();
        return ;
    }

    // frequency control, 如果图像频率低于一个值，就发送     控制一下发送给后端的频率
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        ROS_INFO("!!!!!!!!!!!!!!!!PUB_THIS_FRAME = %d\n", PUB_THIS_FRAME);
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    TicToc t_r;

    if (PUB_THIS_FRAME)
    {

        // cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        cv_bridge::CvImageConstPtr ptr;
        if (img_msg->encoding == "8UC1") // shan:why 8UC1 need this operation? Find answer:https://github.com/ros-perception/vision_opencv/issues/175
        {
            sensor_msgs::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width  = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";         // 更改color_msg彩色图像帧消息的编码方式
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else    // 如果是mono8编码方式直接复制就可以
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

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
        // cv::imshow("lineimg",show_img);
        // cv::waitKey(1);

        frame_cnt ++;
        trackerData.readImage(ptr->image.rowRange(0 , ROW));   // rowRange(i,j) 取图像的i～j行


        cv::Mat show_depth = depth_ptr->image;      // △△△△△ 新添加的深度值 △△△△△
        pub_count ++;
        sensor_msgs::PointCloudPtr feature_lines(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_line;         //  feature id
        sensor_msgs::ChannelFloat32 u_of_endpoint;      //  u
        sensor_msgs::ChannelFloat32 v_of_endpoint;      //  v

        sensor_msgs::ChannelFloat32 depth_of_startpt;   // △△△△△ zlc：新添加的变量 △△△△△
        sensor_msgs::ChannelFloat32 depth_of_endpt;     // △△△△△ zlc：新添加的变量 △△△△△


        feature_lines->header = img_msg->header;
        feature_lines->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i ++)
        {
            if (i != 1 || !STEREO_TRACK)  // 单目
            {
                auto un_lines = trackerData.undistortedLineEndPoints();

                // auto &cur_lines = trackerData.curframe_->vecLine;
                auto& ids = trackerData.curframe_->lineID;

                for (unsigned int j = 0; j < ids.size(); j ++)
                {

                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);       // 这个并没有用到
                    geometry_msgs::Point32 p;
                    p.x = un_lines[j].StartPt.x;
                    p.y = un_lines[j].StartPt.y;
                    p.z = 1;

                    feature_lines->points.push_back(p);                     // 线起始点的x，y，1坐标,归一化坐标
                    depth_of_startpt.values.push_back((int)show_depth.at<unsigned short>(round(un_lines[j].StartPt.y),
                                                                                         round(un_lines[j].StartPt.x)));
                    id_of_line.values.push_back(p_id * NUM_OF_CAM + i);
                    // std::cout<< "feature tracking id: " <<p_id * NUM_OF_CAM + i<<" "<<p_id<<"\n";
                    u_of_endpoint.values.push_back(un_lines[j].EndPt.x);
                    v_of_endpoint.values.push_back(un_lines[j].EndPt.y);
                    depth_of_endpt.values.push_back((int)show_depth.at<unsigned short>(round(un_lines[j].EndPt.y),
                                                                                       round(un_lines[j].EndPt.x)));
                    // ROS_ASSERT(inBorder(cur_pts[j]));
                }
            }

        }
        feature_lines->channels.push_back(id_of_line);          // 线的序号
        feature_lines->channels.push_back(u_of_endpoint);       // 线结束点的x坐标
        feature_lines->channels.push_back(v_of_endpoint);       // 线结束点的y坐标
        feature_lines->channels.push_back(depth_of_startpt);    // 线起始点的深度值
        feature_lines->channels.push_back(depth_of_endpt);      // 线结束点的深度值
        ROS_DEBUG("publish %f, at %f", feature_lines->header.stamp.toSec(), ros::Time::now().toSec());
        pub_img_line.publish(feature_lines);


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

                int lowest = 0, highest = 255;
                int range = (highest - lowest) + 1;
                for (unsigned int j = 0; j < trackerData.curframe_->vecLine.size(); j ++)
                {
                    // double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    // cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);

                    // KeyLine line1 = trackerData.curframe_->vecLine[j];

                    // draw line
                    unsigned int r = lowest + int(rand() % range);
                    unsigned int g = lowest + int(rand() % range);
                    unsigned int b = lowest + int(rand() % range);
                    cv::Point startPoint = cv::Point(int(trackerData.curframe_->vecLine[j].StartPt.x), int(trackerData.curframe_->vecLine[j].StartPt.y));
                    cv::Point endPoint   = cv::Point(int(trackerData.curframe_->vecLine[j].EndPt.x), int(trackerData.curframe_->vecLine[j].EndPt.y));
                    // cv::circle(img1, startPoint, 2, cv::Scalar(0, 255, 0), 2);
                    // cv::circle(img1, endPoint, 2, cv::Scalar(0, 255, 0), 2);
                    cv::line(tmp_img, startPoint, endPoint, cv::Scalar(r, g, b),2 ,8);
                }
            }
            // cv::imshow("vis", stereo_img);
            // cv::waitKey(5);
            pub_match_line.publish(ptr->toImageMsg());       // △△△△△ 这里才正式的发布图片消息 △△△△△
        }

    }
    sum_time += t_r.toc();
    mean_time = sum_time/frame_cnt;
    // ROS_INFO("whole Line feature tracker processing costs: %f", mean_time);
    std::cout << "Line feature tracker processing costs: " << mean_time << endl;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "linefeature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i ++)
        trackerData.readIntrinsicParameter(CAM_NAMES[i]);



    ROS_INFO("start line feature");


    // △ zlc添加：这里把订阅函数改为同步订阅深度图像和彩色图像
    // ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);
    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, IMAGE_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(n, DEPTH_TOPIC, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_image, sub_depth);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));           // 调用img回调函数


    pub_img_line   = n.advertise<sensor_msgs::PointCloud>("linefeature", 1000);
    pub_match_line = n.advertise<sensor_msgs::Image>("linefeature_img",1000);

    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */

    ros::spin();

    return 0;
}
