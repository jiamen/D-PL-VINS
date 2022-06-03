#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

// SizedCostFunction<误差项大小（重投影误差）、第一个优化变量（位姿量）大小（T0）、第二个优化变量（位姿量）大小（T1）、第三个优化变量（观测量）大小（4自由度的O=(ψ, φ)^T）>
class lineProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 4>
{
  public:
    lineProjectionFactor(const Eigen::Vector4d& _pts_i);

    // 解析求导                             输入参数块             输出残差数组         输出雅可比矩阵
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void check(double** parameters);

    Eigen::Vector4d obs_i;

    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};


///////////////////////////////line in camera frame///////////////////////////////////////////
class lineProjectionFactor_incamera : public ceres::SizedCostFunction<2, 7, 7, 7, 4>
{
public:
    lineProjectionFactor_incamera(const Eigen::Vector4d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

class lineProjectionFactor_instartframe : public ceres::SizedCostFunction<2, 4>
{
public:
    lineProjectionFactor_instartframe(const Eigen::Vector4d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};
