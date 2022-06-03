#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"


// 原因：
// 姿态使用的是4元数，但只有3个自由度，且不支持加法
// 优化时，每更新一次都要对其进行归一化，否则就不再是单位四元数
// 这种方法显然比较复杂，因此要寻找3个参数的运算方法
// 解决方法：
// 定义对应的3维localparameter，并实现关于它的广义加法和雅可比。
// 本部分在《多传感器融合》中3节中讲到
class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int  GlobalSize() const { return 7; };      // 四元数
    virtual int  LocalSize()  const { return 6; };      //

public:
    static void Global2LocalJacobian(const double *x, Eigen::Matrix<double, 7, 6, Eigen::RowMajor> *jacobian);
    static void Local2GlobalJacobian(const double *x, Eigen::Matrix<double, 6, 7, Eigen::RowMajor> *jacobian);
};
