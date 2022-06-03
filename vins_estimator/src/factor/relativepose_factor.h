//
// Created by zlc on 2022/4/30.
//

#ifndef _PLVINS_ESTIMATOR_RELATIVATEPOSE_H_
#define _PLVINS_ESTIMATOR_RELATIVATEPOSE_H_

#pragma once


// Eigen
#include "../utility/EigenTypes.h"

// Ceres
#include <ceres/ceres.h>

// Ros
#include <ros/assert.h>

// Self
#include "../parameters.h"
#include "../utility/math_utils.h"
#include "../utility/tic_toc.h"


// 自动求导模板
/*
class RelativePoseFactorAutoDiff
{
public:
    Eigen::Vector3d t_meas_;
    Eigen::Quaterniond q_meas_;

    static Eigen::Mat66d sqrt_info_;
    static double sum_t;


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RelativePoseFactorAutoDiff(const Eigen::Vector3d& t_meas,
                               const Eigen::Quaterniond& q_meas)
    {
        t_meas_ = t_meas;
        q_meas_ = q_meas;
    }


    // 残差的计算模型  ceres 《SLAM 14讲》P117
    template <typename T>
    bool operator() (const T* const pose_i, T* residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> Pi(pose_i);
        Eigen::Map<const Eigen::Quaternion<T>> Qi(pose_i + 3);      // 类型不指定

        Eigen::Map<Eigen::Matrix<T, 6, 1>> residual(residuals_ptr);

        residual.template head<3>() = Pi - t_meas_.cast<T>();
        residual.template tail<3>() = T(2) * (Qi.conjugate() * q_meas_.cast<T>()).vec();

        residual.applyOnTheLeft(sqrt_info_.template cast<T>());

        return true;
    }

    // 自动推导损失函数   《SLAM 14讲》 P118
    static ceres::CostFunction* Create(const Eigen::Vector3d& t_meas,
                                       const Eigen::Quaterniond& q_meas)
    {
        //                                           误差类型         输出维度  输入维度
        return new ceres::AutoDiffCostFunction<RelativePoseFactorAutoDiff, 6, 7>(
                new RelativePoseFactorAutoDiff(t_meas, q_meas)
                );
    }
};
*/


class RelativePoseFactorAutoDiff
{
public:
    RelativePoseFactorAutoDiff(const Eigen::Vector3d& t_meas,
                               const Eigen::Quaterniond& q_meas)
    {
        t_meas_ = t_meas;
        q_meas_ = q_meas;
    };

    template <typename T>
    bool operator()(const T* const pose_i, T* residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> Pi(pose_i);
        Eigen::Map<const Eigen::Quaternion<T>> Qi(pose_i + 3);

        Eigen::Map<Eigen::Matrix<T, 6, 1>> residual(residuals_ptr);

        residual.template head<3>() = Pi - t_meas_.cast<T>();
        residual.template tail<3>() =
                T(2) * (Qi.conjugate() * q_meas_.cast<T>()).vec();
        residual.applyOnTheLeft(sqrt_info_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& t_meas,
                                       const Eigen::Quaterniond& q_meas)
    {
        return new ceres::AutoDiffCostFunction<RelativePoseFactorAutoDiff, 6, 7>(
                new RelativePoseFactorAutoDiff(t_meas, q_meas));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    Eigen::Vector3d t_meas_;
    Eigen::Quaterniond q_meas_;

    static Eigen::Mat66d sqrt_info_;
    static double sum_t;
};





#endif // _PLVINS_ESTIMATOR_RELATIVATEPOSE_H_
