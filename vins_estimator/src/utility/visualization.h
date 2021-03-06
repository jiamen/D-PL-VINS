#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>       // △△△△△ zlc新增的头文件 △△△△△
#include <std_msgs/Bool.h>          // △△△△△ zlc新增的头文件 △△△△△

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator.h"
#include "../parameters.h"
#include <fstream>


#include <cv_bridge/cv_bridge.h>     // △△△△△  zlc添加：为了发布可视化点特征图像帧，特意添加的  △△△△△


extern ros::Publisher pub_odometry;
extern ros::Publisher pub_path, pub_pose;
extern ros::Publisher pub_cloud, pub_map;
extern ros::Publisher pub_key_poses;

extern ros::Publisher pub_ref_pose, pub_cur_pose;
extern ros::Publisher pub_key;

extern nav_msgs::Path path;

extern ros::Publisher pub_pose_graph;

extern int IMAGE_ROW, IMAGE_COL;

void registerPub(ros::NodeHandle& n);

void pubLatestOdometry(const Eigen::Vector3d& P, const Eigen::Quaterniond& Q, const Eigen::Vector3d& V, const std_msgs::Header& header);

void printStatistics(const Estimator& estimator, double t);

void pubOdometry(const Estimator& estimator, const std_msgs::Header& header, Eigen::Vector3d loop_correct_t,
                Eigen::Matrix3d loop_correct_r);

void pubInitialGuess(const Estimator& estimator, const std_msgs::Header& header);

void pubKeyPoses(const Estimator& estimator, const std_msgs::Header& header, Eigen::Vector3d loop_correct_t,
				Eigen::Matrix3d loop_correct_r);

void pubCameraPose(const Estimator& estimator, const std_msgs::Header& header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r);

//void pubPointCloud(const Estimator& estimator, const std_msgs::Header& header, Eigen::Vector3d loop_correct_t,
//                   Eigen::Matrix3d loop_correct_r);
void pubPointCloud(Estimator& estimator, const std_msgs::Header& header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r);

void pubLinesCloud(const Estimator& estimator, const std_msgs::Header& header, Eigen::Vector3d loop_correct_t,
				   Eigen::Matrix3d loop_correct_r);

void pubPoseGraph(CameraPoseVisualization* posegraph, const std_msgs::Header& header);

void updateLoopPath(nav_msgs::Path _loop_path);

void pubTF(const Estimator& estimator, const std_msgs::Header& header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r);



void pubKeyframe(const Estimator& estimator);
void pubRelocalization(const Estimator& estimator);


void pubTrackImg(const cv_bridge::CvImageConstPtr& img_ptr);
