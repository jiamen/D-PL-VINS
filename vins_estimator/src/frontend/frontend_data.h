//
// Created by zlc on 2022/4/24.
//

#ifndef _PLVINS_ESTIMATOR_FRONTEND_DATA_H_
#define _PLVINS_ESTIMATOR_FRONTEND_DATA_H_

#pragma one


#include <memory>
#include <opencv2/core/core.hpp>

#include "../utility/EigenTypes.h"


namespace vins
{

using FeatureID = int;
using TimeFrameId = double;
using FeatureTrackerResults = Eigen::aligned_map<
        int,
        Eigen::aligned_vector< std::pair<int, Eigen::Matrix<double, 7, 1>> > >;


struct FrontEndResult
{
    typedef std::shared_ptr<FrontEndResult> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // optical flow result
    double timestamp;
    FeatureTrackerResults feature;

    cv::Mat image;
};


}


#endif // _PLVINS_ESTIMATOR_FRONTEND_DATA_H_
