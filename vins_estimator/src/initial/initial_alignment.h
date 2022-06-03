#pragma once

// std
#include <iostream>
#include <map>

// Eigen
#include <eigen3/Eigen/Dense>

// ros
#include <ros/ros.h>

#include "../factor/imu_factor.h"
#include "../feature_manager.h"

#include "../utility/utility.h"
#include "../utility/Twist.h"
#include "../utility/math_utils.h"
#include "../utility/math_utils.h"



using namespace Eigen;
using namespace std;

class ImageFrame
{
public:
    ImageFrame(){};
    // ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 5, 1>>>>& _points, double _t):t{_t},is_key_frame{false}
    ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>>& _points, double _t):t{_t},is_key_frame{false}
    {                                           // x, y, z, p_u, p_v, depth;
        /*
        for (auto& it : _points)
        {
            int feature_id = it.first;
            int camera_id = it.second[0].first;
            double x = it.second[0].second.x();
            double y = it.second[0].second.y();
            double z = it.second[0].second.z();
            points[feature_id].emplace_back(camera_id,  Vector3d(x,y,z));
        }
         */
        points = _points;
    };
    /* zlc注释掉的
    ImageFrame(const map<int, vector<pair<int, Vector3d>>>& _points, double _t):points{_points},t{_t},is_key_frame{false}
    {
    };*/

    /*
    ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>>& _points, double _t):t{_t},is_key_frame{false}
    {
        points = _points;
    };
    */

    // map<int, vector<pair<int, Vector3d> > > points;
    map<int, vector< pair<int, Eigen::Matrix<double, 6, 1>> > > points;  // △ zlc添加：xyz_uv_velocity_depth， 速度是两维
                                                             // points: xyz_uv_depth << x, y, z, p_u, p_v, depth;
    double t;
    Matrix3d R;
    Vector3d T;
    IntegrationBase* pre_integration;
    bool is_key_frame;                  // △△△△△ zlc添加：初始化过程中是否是关键帧 △△△△△
};

bool VisualIMUAlignment(map<double, ImageFrame>& all_image_frame, Vector3d* Bgs, Vector3d& g, VectorXd& x);
