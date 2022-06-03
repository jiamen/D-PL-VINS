//
// Created by zlc on 2022/4/24.
//

#include "plane_detection.h"

// glog
#include <glog/logging.h>


double plane_detection::detect_plane(const sensor_msgs::PointCloud& pointCloud, const Eigen::Vector3d& camera_center)
{
    // 将点的距离序号的投票 全部清空
    std::fill(acculators_.begin(), acculators_.end(), 0);

    // 将 传感器数据 改造为 适合kdtree搜索的数据格式
    const PointCloudAdapter pc2kd(pointCloud.points);

    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<float, PointCloudAdapter>,
            PointCloudAdapter,
            3>;
    KDTree index(3, pc2kd,
                 nanoflann::KDTreeSingleIndexAdaptorParams(10));
    // arguments: dimensionality, point cloud adaptor, max leafs

    index.buildIndex();

    const float search_radius_sq = 2.5 * dmax_ * 2.5 * dmax_;       // zlc添加：搜索半径怎么确定的？？？
    std::vector<std::pair<size_t, float>> ret_matches;              // zlc添加：储存搜索结果, 　　序号 +　距离
    nanoflann::SearchParams params;
    params.sorted = false;

    // Vector3d -> Vector3f  相机位置
    Eigen::Vector3f camera_center_f32 = camera_center.cast<float>();

    // 邻域半径内有多少个近邻点
    size_t num_neighbors = index.radiusSearch(
            camera_center_f32.data(), search_radius_sq, ret_matches, params);


    Eigen::Vector3d z_axis{0, 0, -1};
    double camera_z = camera_center.z();        // 相机的位置

    // pair points
    for (size_t i = 0; i < num_neighbors; ++ i)
    {
        // ros数据转为Eigen数据                           根据匹配结果序号　得到　点数据
        Eigen::Vector3d pi = convertPoint32(pointCloud.points[ret_matches[i].first]);

        double candidate_d = pi.z() - camera_z;

        if (candidate_d > dmin_ && candidate_d < dmax_)
        {
            // 依据当前邻域点与相机点的距离，获取当前点的索引
            int index = dist2index(candidate_d);
            if (index >= acculators_.size())
                index = acculators_.size() - 1;

            if (index < 0)
                index = 0;

            ++ acculators_.at(index);       // actual voting， 统计 距离相机中心点距离相同的点 有多少
        }
    }

    // 找出 与相机中心距离一致的 最多的点， 这个距离就是地面距离
    auto d_best_candidate_ptr = std::max_element(acculators_.begin(), acculators_.end());
    // 通过点的序号，得到距离
    auto d_best_candidate = index2dist(d_best_candidate_ptr - acculators_.begin());

    LOG(INFO) << "\n candinate index: "
              << d_best_candidate_ptr - acculators_.begin()
              << "\n distance: " << d_best_candidate
              << "\n the number of point: " << *d_best_candidate_ptr;

    return d_best_candidate;
}


