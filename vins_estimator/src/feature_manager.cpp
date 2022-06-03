
// std
#include <iostream>

// glog
#include <gflags/gflags.h>
#include <glog/logging.h>

// Self
#include "feature_manager.h"

// △△△△△ zlc添加：添加地面点功能后添加的头文件 △△△△△
#include "utility/Twist.h"
#include "utility/logging.h"
#include "utility/math_utils.h"


Vector2d image_uv1;

// 特征点、线 的 最后观测帧
int lineFeaturePerId::endFrame()
{
    return start_frame + linefeature_per_frame.size() - 1;
}

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

// 设置相机与IMU的外参数
FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

/**
 * @brief 得到有效的地图点的数目
 *
 * @return int
 * */
int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    // TODO: 使用unorder_map 代替 list
    for (auto& it : feature)
    {

        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt ++;
        }
    }
    return cnt;
}

/*  zlc 注释
//                                                                      featureId      cameraId, point
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Vector3d>>> &image)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());  // 已有的特征数目
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image)        //遍历当前帧上的特征
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second);  //

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
                          });

        if (it == feature.end())  // 如果之前没存这个特征，说明是新的
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;     // 已经跟踪上了多少个点
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}
*/

// △ zlc添加：图像运行函数②
// bool FeatureManager::addFeatureCheckParallax(int frame_count,
//                                              const map<int, vector<pair<int, Matrix<double, 5, 1>>>>& image,
//                                              const map<int, vector<pair<int, Vector4d>>> &lines)
bool FeatureManager::addFeatureCheckParallax(int frame_count,
                                             const map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>>& image,
                                             const map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>>& lines)
{
    ROS_DEBUG("input point feature: %d", (int)image.size());
    ROS_DEBUG("input line feature: %d",  (int)lines.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());     // 已有的特征数目
    // std::cout << "~~~input point feature: " << (int)image.size() << endl;
    std::cout << "~~~input line feature: "  << (int)lines.size() << endl;
    // std::cout << "~~~num of feature: " << getFeatureCount() << endl;


    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto& id_pts : image)        // 遍历当前帧上的特征
    {
        // id_pts.second[0].second: Eigen::Matrix<double, 5, 1>
        FeaturePerFrame f_per_fra(id_pts.second[0].second);     // △ zlc：图像运行函数③，这里已改为6维 △

        int feature_id = id_pts.first;
        // std::cout<<"id: " << feature_id<<"\n";
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId& it)
        {
            return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
        });

        if (it == feature.end())    // 如果之前没存这个特征，说明是新的
        {
            feature.push_back(FeaturePerId(feature_id, frame_count,
                                           id_pts.second[0].second(5)));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num ++;     // 已经跟踪上了多少个点
        }
    }

    // zlc添加：下面是PL-VINS中开始处理线特征部分
    for (auto& id_line : lines)   //遍历当前帧上的特征
    {
        // map<int, vector<pair<int, Vector4d>>>& lines   ==> id_line.second[0].second : Vector4d
        lineFeaturePerFrame f_per_fra(
                Vector4d(id_line.second[0].second(0), id_line.second[0].second(1),
                                id_line.second[0].second(3), id_line.second[0].second(4)));  // 观测

        int feature_id = id_line.first;
        // cout << "line id: "<< feature_id << "\n";
        auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const lineFeaturePerId& it)
        {
            return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
        });

        if (it == linefeature.end())  // 如果之前没存这个特征，说明是新的
        {
            linefeature.push_back(lineFeaturePerId(feature_id, frame_count));   // △ zlc添加：建立新的线特征点，特征点+当前帧号
            // id_pts.second[0].second: Eigen::Matrix<double, 5, 1>
            // linefeature.push_back(lineFeaturePerId(feature_id, frame_count,
            //                        id_line.second[0].second(2), id_line.second[0].second(5)));
            linefeature.back().linefeature_per_frame.push_back(f_per_fra);                   // △ zlc添加：关联新的线特征点在当前帧上的观测
        }
        else if (it->feature_id == feature_id)
        {
            it->linefeature_per_frame.push_back(f_per_fra);
            it->all_obs_cnt ++;       // 观测到该线特征的帧数 +1
        }
    }

    // 前两帧都设置为KF，追踪过少也认为是KF
    if (frame_count < 2 || last_track_num < 20)
        return true;

    // 遍历所有点特征ID
    for (auto& it_per_id : feature)
    {
        // 计算的实际上是frame_count-1，也就是前一帧是否为关键帧
        // 因此起始帧至少得是frame_count-2，同时至少覆盖到frame_count-1帧
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num ++;
        }
    }

    // 这个和上一帧没有相同的特征点
    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;     // 看看平均视差是否超过一个阈值
    }
}


bool FeatureManager::addFeatureCheckParallax(int frame_count,
                                             const map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>>& image)
{
    ROS_DEBUG("input point feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());     // 已有的特征数目
    // std::cout << "~~~input point feature: " << (int)image.size() << endl;
    // std::cout << "~~~num of feature: " << getFeatureCount() << endl;


    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto& id_pts : image)        // 遍历当前帧上的特征
    {
        // id_pts.second[0].second: Eigen::Matrix<double, 5, 1>
        FeaturePerFrame f_per_fra(id_pts.second[0].second);     // △ zlc：图像运行函数③，这里已改为6维 △

        int feature_id = id_pts.first;
        // std::cout<<"id: " << feature_id<<"\n";
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId& it)
        {
            return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
        });

        if (it == feature.end())    // 如果之前没存这个特征，说明是新的
        {
            feature.push_back(FeaturePerId(feature_id, frame_count,
                                           id_pts.second[0].second(5)));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num ++;     // 已经跟踪上了多少个点
        }
    }


    // 前两帧都设置为KF，追踪过少也认为是KF
    if (frame_count < 2 || last_track_num < 20)
        return true;

    // 遍历所有点特征ID
    for (auto& it_per_id : feature)
    {
        // 计算的实际上是frame_count-1，也就是前一帧是否为关键帧
        // 因此起始帧至少得是frame_count-2，同时至少覆盖到frame_count-1帧
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num ++;
        }
    }

    // 这个和上一帧没有相同的特征点
    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;     // 看看平均视差是否超过一个阈值
    }
}




/*
bool FeatureManager::addFeatureCheckParallax(int frame_count,
                                             const map<int, vector<pair<int, Vector3d>>>& image,
                                             const map<int, vector<pair<int, Vector4d>>>& lines)
{
    ROS_DEBUG("input point feature: %d", (int)image.size());
    ROS_DEBUG("input line feature: %d", (int)lines.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());  // 已有的特征数目
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image)        //遍历当前帧上的特征
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second);  //

        int feature_id = id_pts.first;
        //std::cout<<"id: " << feature_id<<"\n";
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
        {
            return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
        });

        if (it == feature.end())  // 如果之前没存这个特征，说明是新的
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;     // 已经跟踪上了多少个点
        }
    }

    for (auto &id_line : lines)   //遍历当前帧上的特征
    {
        lineFeaturePerFrame f_per_fra(id_line.second[0].second);  // 观测

        int feature_id = id_line.first;
        //cout << "line id: "<< feature_id << "\n";
        auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const lineFeaturePerId &it)
        {
            return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
        });

        if (it == linefeature.end())  // 如果之前没存这个特征，说明是新的
        {
            linefeature.push_back(lineFeaturePerId(feature_id, frame_count));
            linefeature.back().linefeature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->linefeature_per_frame.push_back(f_per_fra);
            it->all_obs_cnt++;
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

bool FeatureManager::addFeatureCheckParallax(int frame_count,
                                             const map<int, vector<pair<int, Vector3d>>>& image,
                                             const map<int, vector<pair<int, Vector8d>>>& lines)
{
    ROS_DEBUG("input point feature: %d", (int)image.size());
    ROS_DEBUG("input line feature: %d", (int)lines.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());  // 已有的特征数目
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;

    for (auto& id_pts : image)        //遍历当前帧上的特征
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second);  //

        int feature_id = id_pts.first;
        //std::cout<<"id: " << feature_id<<"\n";
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
        {
            return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
        });

        if (it == feature.end())  // 如果之前没存这个特征，说明是新的
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;     // 已经跟踪上了多少个点
        }
    }

    for (auto &id_line : lines)   //遍历当前帧上的特征
    {
        lineFeaturePerFrame f_per_fra(id_line.second[0].second);  // 观测

        int feature_id = id_line.first;
        //cout << "line id: "<< feature_id << "\n";
        auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const lineFeaturePerId &it)
        {
            return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
        });

        if (it == linefeature.end())  // 如果之前没存这个特征，说明是新的
        {
            linefeature.push_back(lineFeaturePerId(feature_id, frame_count));
            linefeature.back().linefeature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->linefeature_per_frame.push_back(f_per_fra);
            it->all_obs_cnt++;
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}
*/

void FeatureManager::debugShow()
{
    ROS_DEBUG("debug show");
    for (auto &it : feature)
    {
        ROS_ASSERT(it.feature_per_frame.size() != 0);
        ROS_ASSERT(it.start_frame >= 0);
        ROS_ASSERT(it.used_num >= 0);

        ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame)
        {
            ROS_DEBUG("%d,", int(j.is_used));
            sum += j.is_used;
            printf("(%lf,%lf) ",j.point(0), j.point(1));
        }
        ROS_ASSERT(it.used_num == sum);
    }
}

/**
 * @brief 得到同时被frame_count_l frame_count_r帧看到的特征点在各自的坐标
 *
 * @param[in] frame_count_l
 * @param[in] frame_count_r
 * @return vector<pair<Vector3d, Vector3d>>
 */
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto& it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            // 获得在feature_per_frame中的索引
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));      // 返回相机坐标系下的坐标对
        }
    }
    return corres;
}


// △△△△△ zlc新添加的函数，得到两帧匹配点及对应深度值，    现在运行的这个函数 △△△△△
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorrespondingWithDepth(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto& it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;


            // △△△△△ 获取确定的深度 △△△△△
            double depth_a = it.feature_per_frame[idx_l].depth;
            if (depth_a < 0.1 || depth_a > 10)//max and min measurement
                continue;
            double depth_b = it.feature_per_frame[idx_r].depth;
            if (depth_b < 0.1 || depth_b > 10)//max and min measurement
                continue;


            a = it.feature_per_frame[idx_l].point;
            b = it.feature_per_frame[idx_r].point;

            a = a * depth_a;
            b = b * depth_b;

            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}


void FeatureManager::setDepth(const VectorXd& x)
{
    int feature_index = -1;
    for (auto& it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        // ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}


/**
 * @brief 移除一些不能被三角化的点
 *
 */
void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next ++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}


void FeatureManager::clearDepth(const VectorXd& x)
{
    int feature_index = -1;
    for (auto& it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++ feature_index);
    }
}


// △ zlc：现在运行这个函数 △
VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto& it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++ feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}


int FeatureManager::getLineFeatureCount()
{
    int cnt = 0;
    for (auto& it : linefeature)
    {

        it.used_num = it.linefeature_per_frame.size();

        if (it.used_num >= LINE_MIN_OBS && it.start_frame < WINDOW_SIZE - 2 && it.is_triangulation)
        {
            cnt ++;
        }
    }
    return cnt;
}



MatrixXd FeatureManager::getLineOrthVectorInCamera()
{
    MatrixXd lineorth_vec(getLineFeatureCount(),4);
    int feature_index = -1;
    for (auto& it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        lineorth_vec.row(++ feature_index) = plk_to_orth(it_per_id.line_plucker);

    }
    return lineorth_vec;
}



void FeatureManager::setLineOrthInCamera(MatrixXd x)
{
    int feature_index = -1;
    for (auto& it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        // std::cout<<"x:"<<x.rows() <<" "<<feature_index<<"\n";
        Vector4d line_orth = x.row(++feature_index);
        it_per_id.line_plucker = orth_to_plk(line_orth);// transfrom to camera frame

        // ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        /*
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
         */
    }
}


MatrixXd FeatureManager::getLineOrthVector(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    MatrixXd lineorth_vec(getLineFeatureCount(),4);
    int feature_index = -1;

    for (auto& it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        int imu_i = it_per_id.start_frame;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = Ps[imu_i] + Rs[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = Rs[imu_i] * ric[0];               // Rwc = Rwi * Ric

        Vector6d line_w = plk_to_pose(it_per_id.line_plucker, Rwc, twc);  // transfrom to world frame
        // line_w.normalize();
        lineorth_vec.row(++feature_index) = plk_to_orth(line_w);
        // lineorth_vec.row(++feature_index) = plk_to_orth(it_per_id.line_plucker);

    }
    return lineorth_vec;
}



void FeatureManager::setLineOrth(MatrixXd x,Vector3d P[], Matrix3d R[], Vector3d tic[], Matrix3d ric[])
{
    int feature_index = -1;
    for (auto& it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        Vector4d line_orth_w = x.row(++feature_index);
        Vector6d line_w = orth_to_plk(line_orth_w);

        int imu_i = it_per_id.start_frame;
        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = P[imu_i] + R[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = R[imu_i] * ric[0];               // Rwc = Rwi * Ric

        it_per_id.line_plucker = plk_from_pose(line_w, Rwc, twc); // transfrom to camera frame
        // it_per_id.line_plucker = line_w; // transfrom to camera frame

        // ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        /*
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
         */
    }
}


double FeatureManager::reprojection_error( Vector4d obs, Matrix3d Rwc, Vector3d twc, Vector6d line_w )
{

    double error = 0;

    Vector3d n_w, d_w;
    n_w = line_w.head(3);
    d_w = line_w.tail(3);

    Vector3d p1, p2;
    p1 << obs[0], obs[1], 1;
    p2 << obs[2], obs[3], 1;

    Vector6d line_c = plk_from_pose(line_w,Rwc,twc);
    Vector3d nc = line_c.head(3);
    double sql = nc.head(2).norm();
    nc /= sql;

    error += fabs( nc.dot(p1) );
    error += fabs( nc.dot(p2) );

    return error / 2.0;
}


// △△△△△ zlc添加：地面点区分（无返回值） △△△△△
void FeatureManager::planePointClassify(double* T_w_i_ptr, double* T_i_c_ptr, double plane_d_w)
{

}

// △△△△△ zlc添加：地面点区分（待返回值） △△△△△
int FeatureManager::planePointClassify(Eigen::Vec3d Ps[], Eigen::Vec3d tic[], Matrix3d ric[], double plane_d_w)
{
    // Eigen::aligned_map<vins::TimeFrameId, Eigen::aligned_map<vins::TimeFrameId, Transformd>> relativePoses;
    Eigen::aligned_map<int, Eigen::aligned_map<int, Transformd>> relativePoses;

    // Eigen::aligned_map<vins::TimeFrameId, Transformd> frame_poses;
    Eigen::aligned_map<int, Transformd> frame_poses;

    Transformd T_i_c{ric[0], tic[0]};


    // zlc：先把帧帧之间的相对位姿变换计算出来
    // for (size_t host_id = 0; host_id < time_frameid2_int_frameid_->size(); host_id ++)
    for (int host_id = 0; host_id<WINDOW_SIZE + 1; host_id ++)
    {
        // vins::TimeFrameId host_tid = int_frameid2_time_frameid_->at(host_id);

        Transformd T_w_i_host(Rs[host_id], Ps[host_id]);
        Transformd T_w_c_host = T_w_i_host * T_i_c;
        frame_poses[host_id] = T_w_c_host;

        //for (size_t target_id = host_id + 1; target_id < time_frameid2_int_frameid_->size(); target_id ++)
        for (int target_id = host_id + 1; target_id < WINDOW_SIZE + 1; target_id ++)
        {
            // vins::TimeFrameId target_tid = int_frameid2_time_frameid_->at(target_id);

            Transformd T_w_i_target(Rs[target_id], Ps[target_id]);
            Transformd T_w_c_target = T_w_i_target * T_i_c;

            // T_w_host * T_host_target = T_w_target
            // T_w_target * T_target_host = T_w_host
            Transformd T_c_target_c_host = T_w_c_target.inverse() * T_w_c_host;

            // 相对位姿存的是 主帧 到 目标帧 的位姿
            // relativePoses[host_tid][target_tid] = T_c_target_c_host;
            relativePoses[host_id][target_id] = T_c_target_c_host;
        }
    }

    auto projection_plane_func = [&](
                                    const Transformd& T_w_c_host,
                                    const Transformd& T_c_target_c_host,
                                    const double plane_d_w,
                                    const double* const pts_host_ptr,
                                    const double* const pts_target_ptr,
                                    double* residual_ptr)
    {
        using T = double;
        using Vector2T = Eigen::Matrix<T, 2, 1>;
        using Vector3T = Eigen::Matrix<T, 3, 1>;
        using Vector4T = Eigen::Matrix<T, 4, 1>;
        using QuaterionT = Eigen::Quaternion<T>;
        using TransformT = Twist<T>;

        Eigen::Map<const Vector3T> pts_i(pts_host_ptr);
        Eigen::Map<const Vector3T> pts_j(pts_target_ptr);

        // Step1: construct plane in world frame coordinate.
        Vector4T plane_w;
        plane_w << T(0), T(0), T(1), plane_d_w;

        // Step2: transform plane from world frame to camera frame.
        Vector4T plane_c_host = math_utils::TransformPlane(plane_w, T_w_c_host.inverse());

        // Step3: get the 3d point in host camera frame.
        Vector3T ray_c_host = pts_i.cast<T>();
        T z_lambda = -plane_c_host.w() / (plane_c_host.template head<3>().dot(ray_c_host));

        if (z_lambda < 0)
        {
            LOG(INFO) << RED << "negative depth: " << std::fixed << z_lambda << NO_COLOR;
        }
        Vector3T point_c_host = z_lambda * ray_c_host;

        // Step4: transform the 3d point from host camera to target camera.
        Vector3T point_c_target = T_c_target_c_host * point_c_host;

        Eigen::Map<Vector2T> residual(residual_ptr);

        T point_depth_target = point_c_target.z();
        residual = (point_c_target / point_depth_target).template head<2>() - pts_j.template head<2>();

        residual = T(640) * residual;

        if (z_lambda < 0)
        {
            residual[0] = 640;
            residual[1] = 640;
        }
        return z_lambda;
    };

    int num_plane_points = 0;

    // begin loop all landmark.
    for (auto& it_per_id : feature)
    {
        // Step 1 : 如果一个点是面点，则跳过
        if (it_per_id.is_plane_point)
        {
            num_plane_points ++;
            continue;
        }

        it_per_id.used_num = it_per_id.feature_per_frame.size();

        // Step 2 ：如果一个特征点被观测的次数少于2，不进行平面判定
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
        {
            continue;
        }

        auto host_tid = it_per_id.start_frame;

        Transformd T_w_c_host = frame_poses.at(host_tid);
        // const double* pts_host_ptr = it_per_id.feature_per_frame.at(host_tid).point.data();
        const double* pts_host_ptr = it_per_id.feature_per_frame[0].point.data();

        std::vector<double> reprojection_error;

        std::string fout;
        fout += "***residual: ";

        auto target_tid = host_tid - 1;
        for (const auto& it_per_frame : it_per_id.feature_per_frame)
        {
            target_tid ++;
            if (host_tid == target_tid)
                continue;

            Transformd T_c_target_c_host = relativePoses.at(host_tid).at(target_tid);

            const double* pts_target_ptr = it_per_frame.point.data();

            Eigen::Vec2d residual;
            projection_plane_func(T_w_c_host,
                                  T_c_target_c_host,
                                  plane_d_w,
                                  pts_host_ptr,
                                  pts_target_ptr,
                                  residual.data());
            reprojection_error.push_back(residual.norm());
            // fout += (std::to_string(residual.norm()) + " ");
        }

        auto max_element = *std::max_element(reprojection_error.begin(), reprojection_error.end());

        if (max_element < 10)
        {
            it_per_id.is_plane_point = true;
            num_plane_points ++;
        }

        //  LOG(INFO) << GREEN << fout << NO_COLOR;

    }   // end loop all landmark

    return num_plane_points;
}




// △ zlc添加：现在运行这个函数 △
void FeatureManager::triangulateLine(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    // std::cout<<"feature_manager.cpp/triangulateLine(): before linefeature size: " << linefeature.size() << std::endl;
    int i = 0;
    for (auto& it_per_id : linefeature)        // 遍历每个特征，对新特征进行三角化
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();    // 已经有多少帧看到了这个特征
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2))   // 看到的帧数少于2， 或者 这个特征最近倒数第二帧才看到， 那都不三角化
            continue;

        if (it_per_id.is_triangulation)       // 如果已经三角化了
        // if (it_per_id.estimated_startpt_depth > 0 && it_per_id.estimated_endpt_depth > 0)
            continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];               // Rwc = Rwi * Ric

        // vector<double> verified_startpt_depths;                 // △ zlc新添加的变量
        // vector<double> verified_endpt_depths;                   // △ zlc新添加的变量

        double d = 0, min_cos_theta = 1.0;
        Eigen::Vector3d tij;
        Eigen::Matrix3d Rij;
        Eigen::Vector4d obsi, obsj;      // obs from two frame are used to do triangulation
        // Eigen::Matrix<double, 6, 1> obsi, obsj;                 // △ zlc添加

        // plane pi from ith obs in ith camera frame
        Eigen::Vector4d pii;
        Eigen::Vector3d ni;             // normal vector of plane
        // 遍历所有看到这个线特征的关键帧KF
        for (auto& it_per_frame : it_per_id.linefeature_per_frame)   // 遍历所有的观测， 注意 start_frame 也会被遍历
        {
            imu_j ++;

            if(imu_j == imu_i)          // 第一个观测是start frame 上
            {
                obsi = it_per_frame.lineobs;
                Eigen::Vector3d p1( obsi(0), obsi(1), 1 );
                Eigen::Vector3d p2( obsi(2), obsi(3), 1 );
                // obsi = it_per_frame.lineobs_depth;
                // Eigen::Vector3d p1(obsi(0), obsi(1), obsi(2));
                // Eigen::Vector3d p2(obsi(3), obsi(4), obsi(5));
                pii = pi_from_ppp(p1, p2,Vector3d( 0, 0, 0 ));      // 三点确定一个平面,得到平面法向量
                ni = pii.head(3);
                ni.normalize();
                continue;
            }

            // 非start frame(其他帧)上的观测
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];           // Rwj

            Eigen::Vector3d t = R0.transpose() * (t1 - t0);   // tij  = Riw *
            Eigen::Matrix3d R = R0.transpose() * R1;          // Rij  = Riw * Rwj
            
            Eigen::Vector4d obsj_tmp = it_per_frame.lineobs;
            // Eigen::Matrix<double, 6, 1> obsj_tmp = it_per_frame.lineobs_depth;

            // plane pi from jth obs in ith camera frame
            Vector3d p3( obsj_tmp(0), obsj_tmp(1), 1 );     // △zlc：观测线的起点
            Vector3d p4( obsj_tmp(2), obsj_tmp(3), 1 );     // △zlc：观测线的终点
            // Vector3d p3( obsj_tmp(0), obsj_tmp(1), obsj_tmp(2) );
            // Vector3d p4( obsj_tmp(3), obsj_tmp(4), obsj_tmp(5) );
            p3 = R * p3 + t;                         // △△△△△ zlc：把j下的两个观测点移到i帧下了
            p4 = R * p4 + t;
            Vector4d pij = pi_from_ppp(p3, p4,t);       // △△△△△ zlc添加：根据投影线与原点计算的 法向量
            Eigen::Vector3d nj = pij.head(3);
            nj.normalize();

            double cos_theta = ni.dot(nj);      // △△△△△ zlc： 两个平面的法向量积 △△△△△
            if(cos_theta < min_cos_theta)
            {
                min_cos_theta = cos_theta;
                tij = t;
                Rij = R;
                obsj = obsj_tmp;
                d = t.norm();
            }
            // if( d < t.norm() )  // 选择最远的那俩帧进行三角化
            // {
            //     d = t.norm();
            //     tij = t;
            //     Rij = R;
            //     obsj = it_per_frame.lineobs;      // 特征的图像坐标
            // }

        }
        
        // if the distance between two frame is lower than 0.1m or the parallax angle is lower than 15deg , do not triangulate.
        // if(d < 0.1 || min_cos_theta > 0.998) 
        if(min_cos_theta > 0.998)     // △ zlc：想想这个值怎么来的，点积，也就是一个向量在两一个向量上的点积  1×cos15°，注意，两个值已经归一化
        // if( d < 0.2 ) 
            continue;

        // △△△△△△△△△△△△△△△△△△△△△△△△ zlc添加： 经过上面的操作，我们得到了 相差最大的两帧 对于 同一特征线的观测 △△△△△△△△△△△△△△△△△△△△△△△

        // plane pi from jth obs in ith camera frame
        Vector3d p3( obsj(0), obsj(1), 1 );
        Vector3d p4( obsj(2), obsj(3), 1 );
        // Vector3d p3( obsj(0), obsj(1), obsj(2) );
        // Vector3d p4( obsj(3), obsj(4), obsj(5) );
        p3 = Rij * p3 + tij;
        p4 = Rij * p4 + tij;
        Vector4d pij = pi_from_ppp(p3, p4,tij);

        Vector6d plk = pipi_plk( pii, pij );            // △△△△△ 两平面相交得到空间线的普吕克坐标在ci下的表示 △△△△△
        Vector3d n = plk.head(3);                   // △△△△△ zlc添加：论文中的n
        Vector3d v = plk.tail(3);                   // △△△△△ zlc添加：论文中的d


        // Vector3d cp = plucker_origin( n, v );
        // if ( cp(2) < 0 )
        {
          //  cp = - cp;
          //  continue;
        }

        // Vector6d line;
        // line.head(3) = cp;
        // line.tail(3) = v;
        // it_per_id.line_plucker = line;

        // plk.normalize();
        it_per_id.line_plucker = plk;           // plk in camera frame 空间线在起始观测帧下的普吕克表示
        it_per_id.is_triangulation = true;
        i ++;
        // std::cout<<"feature_manager.cpp/triangulateLine(): after linefeature size: " << i << std::endl;

        //  used to debug
        Vector3d pc, nc, vc;
        nc = it_per_id.line_plucker.head(3);
        vc = it_per_id.line_plucker.tail(3);


        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;           // ？？？？？？？这是不是写错了？？？？？？？

        Vector4d obs_startframe = it_per_id.linefeature_per_frame[0].lineobs;               // 第一次观测到这线特征
        // Vector4d obs_startframe = it_per_id.linefeature_per_frame[0].lineobs_depth;      // 第一次观测到这帧
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        // Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), obs_startframe(2));
        // Vector3d p21 = Vector3d(obs_startframe(3), obs_startframe(4), obs_startframe(5));
        Vector2d ln = ( p11.cross(p21) ).head(2);                       // ?????????直线的垂直方向?????????
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // ???????????直线垂直方向上移动一个单位?????????????
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        // Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), p11(2));
        // Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), p21(2));
        Vector3d cam = Vector3d( 0, 0, 0 );

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;         // △△△△ zlc添加：核心思想，把起点的普鲁克坐标表示反映射回相机下的观测坐标
        Vector4d e2 = Lc * pi2;         // △△△△ zlc添加：核心思想，把起点的普鲁克坐标表示反映射回相机下的观测坐标
        e1 = e1 / e1(3);          // △△△△ zlc： 回到了归一化平面
        e2 = e2 / e2(3);

        Vector3d pts_1(e1(0),e1(1),e1(2));      // △△△△ zlc： 相机坐标系下  起点 的归一化平面表
        Vector3d pts_2(e2(0),e2(1),e2(2));      // △△△△ zlc： 相机坐标系下  终点 的归一化平面表

        Vector3d w_pts_1 =  Rs[imu_i] * (ric[0] * pts_1 + tic[0]) + Ps[imu_i];
        Vector3d w_pts_2 =  Rs[imu_i] * (ric[0] * pts_2 + tic[0]) + Ps[imu_i];
        it_per_id.ptw1 = w_pts_1;       // △△△△ zlc： 世界坐标系下  起点 的归一化平面表示
        it_per_id.ptw2 = w_pts_2;       // △△△△ zlc： 世界坐标系下  终点 的归一化平面表示

        // if(isnan(cp(0)))
        {

            // it_per_id.is_triangulation = false;

            // std::cout <<"------------"<<std::endl;
            // std::cout << line << "\n\n";
            // std::cout << d <<"\n\n";
            // std::cout << Rij <<std::endl;
            // std::cout << tij <<"\n\n";
            // std::cout <<"obsj: "<< obsj <<"\n\n";
            // std::cout << "p3: " << p3 <<"\n\n";
            // std::cout << "p4: " << p4 <<"\n\n";
            // std::cout <<pi_from_ppp(p3, p4,tij)<<std::endl;
            // std::cout << pij <<"\n\n";

        }


    }

    // std::cout<<"feature_manager.cpp/triangulateLine(): after linefeature size: " << i << std::endl;

//    removeLineOutlier(Ps,tic,ric);
}


/**
 *  @brief  stereo line triangulate
 */
void FeatureManager::triangulateLine(double baseline)
{
    for (auto &it_per_id : linefeature)        // 遍历每个特征，对新特征进行三角化
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();    // 已经有多少帧看到了这个特征
        // TODO: 右目没看到
        if (it_per_id.is_triangulation || it_per_id.used_num < 2)  // 已经三角化了 或者 少于两帧看到 或者 右目没有看到
            continue;

        int imu_i = it_per_id.start_frame;

        Vector4d lineobs_l,lineobs_r;
        lineFeaturePerFrame it_per_frame = it_per_id.linefeature_per_frame.front();
        lineobs_l = it_per_frame.lineobs;
        lineobs_r = it_per_frame.lineobs_R;

        // plane pi from ith left obs in ith left camera frame
        Vector3d p1( lineobs_l(0), lineobs_l(1), 1 );
        Vector3d p2( lineobs_l(2), lineobs_l(3), 1 );
        Vector4d pii = pi_from_ppp(p1, p2,Vector3d( 0, 0, 0 ));

        // plane pi from ith right obs in ith left camera frame
        Vector3d p3( lineobs_r(0) + baseline, lineobs_r(1), 1 );
        Vector3d p4( lineobs_r(2) + baseline, lineobs_r(3), 1 );
        Vector4d pij = pi_from_ppp(p3, p4,Vector3d(baseline, 0, 0));

        Vector6d plk = pipi_plk( pii, pij );
        Vector3d n = plk.head(3);
        Vector3d v = plk.tail(3);

        //Vector3d cp = plucker_origin( n, v );
        //if ( cp(2) < 0 )
        {
            //  cp = - cp;
            //  continue;
        }

        //Vector6d line;
        //line.head(3) = cp;
        //line.tail(3) = v;
        //it_per_id.line_plucker = line;

        // plk.normalize();
        it_per_id.line_plucker = plk;  // plk in camera frame
        it_per_id.is_triangulation = true;

        //if(isnan(cp(0)))
        {

            //it_per_id.is_triangulation = false;

            //std::cout <<"------------"<<std::endl;
            //std::cout << line << "\n\n";
            //std::cout << d <<"\n\n";
            //std::cout << Rij <<std::endl;
            //std::cout << tij <<"\n\n";
            //std::cout <<"obsj: "<< obsj <<"\n\n";
            //std::cout << "p3: " << p3 <<"\n\n";
            //std::cout << "p4: " << p4 <<"\n\n";
            //std::cout <<pi_from_ppp(p3, p4,tij)<<std::endl;
            //std::cout << pij <<"\n\n";

        }


    }

    removeLineOutlier();

}
//*/



/*
 // 此段代码用于仿真验证
void FeatureManager::triangulateLine(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    //std::cout<<"linefeature size: "<<linefeature.size()<<std::endl;
    // create two noise generator
    std::default_random_engine generator;
    std::default_random_engine generator_;
    std::normal_distribution<double> pixel_n_;

    std::normal_distribution<double> pixel_n(0.0, 1./500);
    std::normal_distribution<double> nt(0., 0.1);         // 10cm
    std::normal_distribution<double> nq(0., 1*M_PI/180);  // 2deg

    generator_ = generator;
    pixel_n_ = pixel_n;

    // 产生虚假观测
    // transform the landmark to world frame
    Eigen::Matrix4d Twl = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d Rwl;
    Rwl = Eigen::AngleAxisd(M_PI/5,Eigen::Vector3d::UnitZ())
          *Eigen::AngleAxisd(M_PI/5,Eigen::Vector3d::UnitY())
          *Eigen::AngleAxisd(-M_PI/5,Eigen::Vector3d::UnitX());
    Eigen::Vector3d twl(0.1, -0.1, 10.);
    //Eigen::Vector3d twl(15.0, 1.0, -1.);
    Twl.block(0, 0, 3, 3) = Rwl;
    Twl.block(0, 3, 3, 1) = twl;

    double cube_size = 6.0;
    Eigen::Vector4d pt0( 0.0, 0.0, 0.0, 1 );
    Eigen::Vector4d pt1( cube_size, 0.0, 0.0, 1 );          // line 1  = pt0 -->pt1
    Eigen::Vector4d pt2( 0.0, -cube_size, 0.0, 1);          // line 2  = pt0 -->pt2
    Eigen::Vector4d pt3( 0.0 , 0.0, cube_size, 1);    // line 3  = pt0 -->pt3
    pt0 = Twl * pt0;
    pt1 = Twl * pt1;
    pt2 = Twl * pt2;
    pt3 = Twl * pt3;


    int line_type = 0;
    for (auto &it_per_id : linefeature)        // 遍历每个特征，对新特征进行三角化
    {

        it_per_id.used_num = it_per_id.linefeature_per_frame.size();    // 已经有多少帧看到了这个特征
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2))   // 看到的帧数少于2， 或者 这个特征最近倒数第二帧才看到， 那都不三角化
            continue;

        // 挑选一条直线
        Eigen::Vector4d pc0 = pt0;
        Eigen::Vector4d pc1;
        switch(line_type)
        {
            case 0: {
                pc1 = pt1;
                line_type = 1;
                break;
            }
            case 1: {
                pc1 = pt2;
                line_type = 2;
                break;
            }
            case 2: {
                pc1 = pt3;
                line_type = 0;
                break;
            }
        }

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];               // Rwc = Rwi * Ric

        // tranfrom line to camera
        Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();

        //Eigen::Vector3d tcw = - R0.transpose() * t0;
        //Eigen::Matrix3d Rcw = R0.transpose();
        //Tcw.block(0, 0, 3, 3) = Rcw;
        //Tcw.block(0, 3, 3, 1) = tcw;

        Eigen::Vector4d pc0i = Tcw * pc0;
        Eigen::Vector4d pc1i= Tcw * pc1;

        double d = 0;
        Eigen::Vector3d tij;
        Eigen::Matrix3d Rij;
        Eigen::Vector4d obsi, obsj, temp_obsj;
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)   // 遍历所有的观测， 注意 start_frame 也会被遍历
        {
            imu_j++;

            Vector4d noise;
            noise << pixel_n_(generator_),pixel_n_(generator_),pixel_n_(generator_),pixel_n_(generator_);
            //noise.setZero();

            if(imu_j == imu_i)   // 第一个观测是start frame 上
            {
                obsi << pc0i(0) / pc0i(2), pc0i(1) / pc0i(2),
                        pc1i(0) / pc1i(2), pc1i(1) / pc1i(2);
                obsi = obsi + noise;
                it_per_frame.lineobs = obsi;
                //obsi = it_per_frame.lineobs;
                continue;
            }

            // std::cout<< "line tri: "<<imu_j<<std::endl;
            // 非start frame(其他帧)上的观测
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];

            Eigen::Vector3d t = R0.transpose() * (t1 - t0);   // tij
            Eigen::Matrix3d R = R0.transpose() * R1;          // Rij

            Eigen::Matrix4d Tji = Eigen::Matrix4d::Identity();
            Eigen::Vector3d tji = - R.transpose() * t;
            Eigen::Matrix3d Rji = R.transpose();
            Tji.block(0, 0, 3, 3) = Rji;
            Tji.block(0, 3, 3, 1) = tji;

            Eigen::Vector4d pc0j = Tji * pc0i;
            Eigen::Vector4d pc1j= Tji * pc1i;

            temp_obsj << pc0j(0) / pc0j(2), pc0j(1) / pc0j(2),
                    pc1j(0) / pc1j(2), pc1j(1) / pc1j(2);
            temp_obsj = temp_obsj + noise;
            it_per_frame.lineobs = temp_obsj;      // 特征的图像坐标
            if( d < t.norm() )  // 选择最远的那俩帧进行三角化
            {
                d = t.norm();
                tij = t;
                Rij = R;
                obsj = it_per_frame.lineobs;
            }

        }
        if(d < 0.15) // 如果小于15cm， 不三角化
            continue;

        // plane pi from ith obs in ith camera frame
        Vector3d p1( obsi(0), obsi(1), 1 );
        Vector3d p2( obsi(2), obsi(3), 1 );
        Vector4d pii = pi_from_ppp(p1, p2,Vector3d( 0, 0, 0 ));

        // plane pi from jth obs in ith camera frame
        Vector3d p3( obsj(0), obsj(1), 1 );
        Vector3d p4( obsj(2), obsj(3), 1 );
        p3 = Rij * p3 + tij;
        p4 = Rij * p4 + tij;
        Vector4d pij = pi_from_ppp(p3, p4,tij);

        Vector6d plk = pipi_plk( pii, pij );
        Vector3d n = plk.head(3);
        Vector3d v = plk.tail(3);
        Vector3d v1 = (pc0 - pc1).head(3);

        Vector6d line;
        line.head(3) = n;
        line.tail(3) = v;

        it_per_id.line_plucker = line;
        it_per_id.is_triangulation = true;

        it_per_id.line_plk_init = line;
        it_per_id.obs_init = obsi;

//-----------------------------------------------
        //  used to debug
        //std::cout <<"tri: "<< it_per_id.feature_id <<" " << it_per_id.line_plucker <<"\n";
        Vector3d pc, nc, vc;
        nc = it_per_id.line_plucker.head(3);
        vc = it_per_id.line_plucker.tail(3);

        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs_startframe = it_per_id.linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d( 0, 0, 0 );

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);

        Vector3d pts_1(e1(0),e1(1),e1(2));
        Vector3d pts_2(e2(0),e2(1),e2(2));

        Vector3d w_pts_1 =  Rs[imu_i] * (ric[0] * pts_1 + tic[0]) + Ps[imu_i];
        Vector3d w_pts_2 =  Rs[imu_i] * (ric[0] * pts_2 + tic[0]) + Ps[imu_i];
        it_per_id.ptw1 = w_pts_1;
        it_per_id.ptw2 = w_pts_2;
        it_per_id.Ri_ = Rs[imu_i];
        it_per_id.ti_ = Ps[imu_i];

        //std::cout<<"---------------------------\n";
        //std::cout << w_pts_1 <<"\n" << w_pts_2 <<"\n\n";

        //   -----
        imu_j = imu_i + 1;
        Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
        Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
        it_per_id.Rj_ = Rs[imu_j];
        it_per_id.tj_ = Ps[imu_j];
        it_per_id.obs_j = it_per_id.linefeature_per_frame[imu_j - imu_i].lineobs;

        Eigen::Vector3d t = R1.transpose() * (t0 - t1);   // Rcjw * (twci - twcj)
        Eigen::Matrix3d R = R1.transpose() * R0;          // Rij

        Vector6d plk_j = plk_to_pose(it_per_id.line_plucker, R, t);

        nc = plk_j.head(3);
        vc = plk_j.tail(3);

        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        obs_startframe = it_per_id.linefeature_per_frame[imu_j - imu_i].lineobs;   // 第一次观测到这帧
        p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        cam = Vector3d( 0, 0, 0 );

        pi1 = pi_from_ppp(cam, p11, p12);
        pi2 = pi_from_ppp(cam, p21, p22);

        e1 = Lc * pi1;
        e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);

        pts_1 = Vector3d(e1(0),e1(1),e1(2));
        pts_2 = Vector3d(e2(0),e2(1),e2(2));

        w_pts_1 =  R1 * pts_1 + t1;
        w_pts_2 =  R1 * pts_2 + t1;

        //std::cout << w_pts_1 << "\n" <<w_pts_2 <<"\n";

    }
    removeLineOutlier(Ps,tic,ric);
}
*/


// △ zlc：现在开始 不运行 这个函数 △
void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    // std::cout << "feature_manager.cpp/triangulate(): linefeature size: " << feature.size() << std::endl;
    for (auto& it_per_id : feature)        // 遍历每个特征，对新特征进行三角化
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();    // 已经有多少帧看到了这个特征
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))   // 看到的帧数少于2， 或者 这个特征最近倒数第二帧才看到， 那都不三角化
            continue;

        if (it_per_id.estimated_depth > 0)              // 如果已经三角化了
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;



        ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];               // Rwc = Rwi * Ric
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto& it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j ++;

            // std::cout<< "point tri: "<<imu_j<<std::endl;
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);   // tij
            Eigen::Matrix3d R = R0.transpose() * R1;          // Rij

            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();      // 特征的图像坐标方向向量
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;
        it_per_id.estimate_flag = 2;
        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}


// △△△△△  zlc：现在运行这个函数VINS-RGBD中新添加的函数  △△△△△
void FeatureManager::triangulateWithDepth(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    // std::cout << "feature_manager.cpp/triangulateWithDepth(): linefeature size: " << feature.size() << std::endl;
    for (auto& it_per_id : feature)         // △△△△△ 遍历每个特征，对新特征进行三角化 △△△△△
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();    // △△△△△ 已经有了多少帧看到了这个特征 △△△△△
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))  // △△△△△ 看到的帧数小于2，或者 这个特征最近倒数第二帧才看到，那都不三角化 △△△△△
            continue;

        if (it_per_id.estimated_depth > 0)          // △△△△△ 如果已经三角化了 △△△△△
            continue;

        int start_frame = it_per_id.start_frame;

        vector<double> verified_depths;             // △△△△△ 新添加的 △△△△△

        Eigen::Vector3d tr = Ps[start_frame] + Rs[start_frame] * tic[0];
        Eigen::Matrix3d Rr = Rs[start_frame] * ric[0];

        for (int i = 0; i < (int)it_per_id.feature_per_frame.size(); i ++)
        {
            Eigen::Vector3d t0 = Ps[start_frame+i] + Rs[start_frame+i] * tic[0];
            Eigen::Matrix3d R0 = Rs[start_frame+i] * ric[0];
            double depth_threshold = 3;         // for handheld and wheeled application. Since d435i <3 is quiet acc
            // double depth_threshold = 10;     // for tracked application, since IMU quite noisy in this scene
            if (it_per_id.feature_per_frame[i].depth < 0.1 || it_per_id.feature_per_frame[i].depth > depth_threshold)
                continue;
            Eigen::Vector3d point0(it_per_id.feature_per_frame[i].point * it_per_id.feature_per_frame[i].depth);

            // transform to reference frame
            Eigen::Vector3d t2r = Rr.transpose() * (t0 - tr);
            Eigen::Matrix3d R2r = Rr.transpose() * R0;

            for (int j = 0; j<(int)it_per_id.feature_per_frame.size(); j ++)
            {
                if (i == j)
                    continue;
                Eigen::Vector3d t1 = Ps[start_frame+j] + Rs[start_frame+j] * tic[0];
                Eigen::Matrix3d R1 = Rs[start_frame+j] * ric[0];
                Eigen::Vector3d t20 = R0.transpose() * (t1 - t0);       // tij
                Eigen::Matrix3d R20 = R0.transpose() * R1;              // Rij

                // 第一次i=0, j=1, 重投影残差在第2帧上验证;  第二次i=1，j=0，重投影残差在第1帧上验证
                Eigen::Vector3d point1_projected = R20.transpose() * point0 - R20.transpose() * t20;
                Eigen::Vector2d point1_2d(it_per_id.feature_per_frame[j].point.x(), it_per_id.feature_per_frame[j].point.y());
                Eigen::Vector2d residual = point1_2d - Vector2d(point1_projected.x() / point1_projected.z(), point1_projected.y() / point1_projected.z());
                if (residual.norm() < 10.0 / 460)
                {   // this can also be adjust to improve performance
                    Eigen::Vector3d point_r = R2r * point0 + t2r;
                    verified_depths.push_back(point_r.z());
                }
            }
        }

        if (verified_depths.size() == 0)
            continue;

        // 对A的SVD分解得到其最小奇异值对应的单位奇异向量(x,y,z,w)，深度为z/w

        double depth_sum = std::accumulate(std::begin(verified_depths),std::end(verified_depths),0.0);
        double depth_ave = depth_sum / verified_depths.size();
        // for (int i = 0; i < (int)verified_depths.size(); i ++)
        // {
        //     cout << verified_depths[i]<<"|";
        // }
        // cout << endl;
        it_per_id.estimated_depth = depth_ave;
        it_per_id.estimate_flag = 1;

        if (it_per_id.estimated_depth < 0.1)        // 小于0.1m的深度值不确定
        {
            it_per_id.estimated_depth = INIT_DEPTH;
            it_per_id.estimate_flag = 0;                // △△△△△ 新添加的变量 △△△△△
        }

    }
}


void FeatureManager::removeOutlier()
{
    ROS_BREAK();
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}
void FeatureManager::removeLineOutlier()
{

    for (auto it_per_id = linefeature.begin(), it_next = linefeature.begin();
         it_per_id != linefeature.end(); it_per_id = it_next)
    {
        it_next++;
        it_per_id->used_num = it_per_id->linefeature_per_frame.size();
        // TODO: 右目没看到
        if (it_per_id->is_triangulation || it_per_id->used_num < 2)  // 已经三角化了 或者 少于两帧看到 或者 右目没有看到
            continue;

        int imu_i = it_per_id->start_frame, imu_j = imu_i -1;

        // 计算初始帧上线段对应的3d端点
        Vector3d pc, nc, vc;
        nc = it_per_id->line_plucker.head(3);
        vc = it_per_id->line_plucker.tail(3);

        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs_startframe = it_per_id->linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d( 0, 0, 0 );

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);

        if(e1(2) < 0 || e2(2) < 0)
        {
            linefeature.erase(it_per_id);
            continue;
        }

        if((e1-e2).norm() > 10)
        {
            linefeature.erase(it_per_id);
            continue;
        }
/*
        // 点到直线的距离不能太远啊
        Vector3d Q = plucker_origin(nc,vc);
        if(Q.norm() > 5.0)
        {
            linefeature.erase(it_per_id);
            continue;
        }
*/

    }
}

// △ zlc添加：现在运行这个函数 △
void FeatureManager::removeLineOutlier(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{

    for (auto it_per_id = linefeature.begin(), it_next = linefeature.begin();
         it_per_id != linefeature.end(); it_per_id = it_next)
    {
        it_next ++;
        it_per_id->used_num = it_per_id->linefeature_per_frame.size();
        if (!(it_per_id->used_num >= LINE_MIN_OBS && it_per_id->start_frame < WINDOW_SIZE - 2 && it_per_id->is_triangulation))
            continue;

        int imu_i = it_per_id->start_frame, imu_j = imu_i -1;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = Ps[imu_i] + Rs[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = Rs[imu_i] * ric[0];               // Rwc = Rwi * Ric

        // 计算初始帧上线段对应的3d端点
        Vector3d pc, nc, vc;
        nc = it_per_id->line_plucker.head(3);
        vc = it_per_id->line_plucker.tail(3);

        //       double  d = nc.norm()/vc.norm();
        //       if (d > 5.0)
        {
        //           std::cerr <<"remove a large distant line \n";
        //           linefeature.erase(it_per_id);
        //           continue;
        }

        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs_startframe = it_per_id->linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Vector2d ln = ( p11.cross(p21) ).head(2);                       // 直线的垂直方向
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d( 0, 0, 0 );

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1 / e1(3);
        e2 = e2 / e2(3);

        // std::cout << "line endpoint: "<<e1 << "\n "<< e2<<"\n";
        // std::cout << "feature_manager.cpp/removeLineOutlier(): line endpoint: " << e1 << "\n ";     // << e2 << "\n";
        if(e1(2) < 0 || e2(2) < 0)
        {
            linefeature.erase(it_per_id);
            continue;
        }
        if((e1-e2).norm() > 10)
        {
            linefeature.erase(it_per_id);
            continue;
        }

/*
        // 点到直线的距离不能太远啊
        Vector3d Q = plucker_origin(nc,vc);
        if(Q.norm() > 5.0)
        {
            linefeature.erase(it_per_id);
            continue;
        }
*/
        // 并且平均投影误差不能太大啊
        Vector6d line_w = plk_to_pose(it_per_id->line_plucker, Rwc, twc);  // transfrom to world frame

        int i = 0;
        double allerr = 0;
        Eigen::Vector3d tij;
        Eigen::Matrix3d Rij;
        Eigen::Vector4d obs;

        // std::cout << "feature_manager.cpp/removeLineOutlier(): reprojection_error! \n";
        for (auto& it_per_frame : it_per_id->linefeature_per_frame)   // 遍历所有的观测， 注意 start_frame 也会被遍历
        {
            imu_j ++;

            obs = it_per_frame.lineobs;
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];

            double err =  reprojection_error(obs, R1, t1, line_w);

//            if(err > 0.0000001)
//                i++;
//            allerr += err;    // 计算平均投影误差

            if(allerr < err)    // 记录最大投影误差，如果最大的投影误差比较大，那就说明有outlier
                allerr = err;
        }
//        allerr = allerr / i;
        if (allerr > 3.0 / 500.0)
        {
            // std::cout << "feature_manager.cpp/removeLineOutlier(): remove a large error\n";
            linefeature.erase(it_per_id);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    // std::cout << "feature_manager.cpp/removeBackShiftDepth()!!\n";

    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next ++;

        if (it->start_frame != 0)    // 如果特征不是在这帧上初始化的，那就不用管，只要管id--
            it->start_frame --;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;              // 取出归一化相机坐标系坐标
            it->feature_per_frame.erase(it->feature_per_frame.begin());  // 该点不再被原来的第一帧看到，因此从中移除
            if (it->feature_per_frame.size() < 2)                               // 如果观测到这个帧的图像少于两帧，那这个特征不要了
            {
                feature.erase(it);
                continue;
            }
            else  // 进行管辖权的转交，如果还有很多帧看到它，而我们又把这个特征的初始化帧给marg掉了，那就得把这个特征转挂到下一帧上去, 这里 marg_R, new_R 都是相应时刻的相机坐标系到世界坐标系的变换
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;             // 实际相机坐标系下的坐标
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;              // 转到世界坐标系下
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);  // 转到新的最老帧的相机坐标系下
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }


    for (auto it = linefeature.begin(), it_next = linefeature.begin();
         it != linefeature.end(); it = it_next)
    {
        it_next ++;

        if (it->start_frame != 0)    // 如果特征不是在这帧上初始化的，那就不用管，只要管id--
        {
            it->start_frame--;
        }
        else
        {
/*
            //  used to debug
            Vector3d pc, nc, vc;
            nc = it->line_plucker.head(3);
            vc = it->line_plucker.tail(3);

            Matrix4d Lc;
            Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

            Vector4d obs_startframe = it->linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
            Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
            Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
            Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
            ln = ln / ln.norm();

            Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
            Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
            Vector3d cam = Vector3d( 0, 0, 0 );

            Vector4d pi1 = pi_from_ppp(cam, p11, p12);
            Vector4d pi2 = pi_from_ppp(cam, p21, p22);

            Vector4d e1 = Lc * pi1;
            Vector4d e2 = Lc * pi2;
            e1 = e1/e1(3);
            e2 = e2/e2(3);

            Vector3d pts_1(e1(0),e1(1),e1(2));
            Vector3d pts_2(e2(0),e2(1),e2(2));

            Vector3d w_pts_1 =  marg_R * pts_1 + marg_P;
            Vector3d w_pts_2 =  marg_R * pts_2 + marg_P;

            std::cout<<"-------------------------------\n";
            std::cout << w_pts_1 << "\n" <<w_pts_2 <<"\n\n";
            Vector4d obs_startframe = it->linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
            */
//-----------------
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());  // 移除观测
            if (it->linefeature_per_frame.size() < 2)                     // 如果观测到这个帧的图像少于两帧，那这个特征不要了
            {
                linefeature.erase(it);
                continue;
            }
            else  // 如果还有很多帧看到它，而我们又把这个特征的初始化帧给marg掉了，那就得把这个特征转挂到下一帧上去, 这里 marg_R, new_R 都是相应时刻的相机坐标系到世界坐标系的变换
            {
                it->removed_cnt++;
                // transpose this line to the new pose
                Matrix3d Rji = new_R.transpose() * marg_R;     // Rcjw * Rwci
                Vector3d tji = new_R.transpose() * (marg_P - new_P);
                Vector6d plk_j = plk_to_pose(it->line_plucker, Rji, tji);
                it->line_plucker = plk_j;
            }
//-----------------------
/*
            //  used to debug
            nc = it->line_plucker.head(3);
            vc = it->line_plucker.tail(3);

            Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

            obs_startframe = it->linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
            p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
            p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
            ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
            ln = ln / ln.norm();

            p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
            p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
            cam = Vector3d( 0, 0, 0 );

            pi1 = pi_from_ppp(cam, p11, p12);
            pi2 = pi_from_ppp(cam, p21, p22);

            e1 = Lc * pi1;
            e2 = Lc * pi2;
            e1 = e1/e1(3);
            e2 = e2/e2(3);

            pts_1 = Vector3d(e1(0),e1(1),e1(2));
            pts_2 = Vector3d(e2(0),e2(1),e2(2));

            w_pts_1 =  new_R * pts_1 + new_P;
            w_pts_2 =  new_R * pts_2 + new_P;

            std::cout << w_pts_1 << "\n" <<w_pts_2 <<"\n";
*/
        }
    }


}


/**
 * @brief 这个还没初始化结束，因此相比刚才，不进行地图点新的深度的换算，因为此时还有进行视觉惯性对齐
 *
 */
// 移除窗口里最老关键帧对应的帧上的特征
void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next ++;

        // 如果这个特征不是在窗口里最老关键帧上观测到的，由于窗口里移除掉了一个帧，所有其他特征对应的初始化帧id都要减1左移
        // 例如： 窗口里有 0,1,2,3,4 一共5个关键帧，特征f2在第2帧上三角化的， 移除掉第0帧以后， 第2帧在窗口里的id就左移变成了第1帧，这是很f2的start_frame对应减1
        if (it->start_frame != 0)
            it->start_frame --;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());  // 删掉特征ft在这个图像帧上的观测量
            if (it->feature_per_frame.size() == 0)                       // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                feature.erase(it);
        }
    }

    // std::cout << "remove back" << std::endl;
    for (auto it = linefeature.begin(), it_next = linefeature.begin();
         it != linefeature.end(); it = it_next)
    {
        it_next ++;

        // 如果这个特征不是在窗口里最老关键帧上观测到的，由于窗口里移除掉了一个帧，所有其他特征对应的初始化帧id都要减1左移
        // 例如： 窗口里有 0,1,2,3,4 一共5个关键帧，特征f2在第2帧上三角化的， 移除掉第0帧以后， 第2帧在窗口里的id就左移变成了第1帧，这是很f2的start_frame对应减1
        if (it->start_frame != 0)
            it->start_frame --;
        else
        {
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());  // 删掉特征ft在这个图像帧上的观测量
            if (it->linefeature_per_frame.size() == 0)                       // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                linefeature.erase(it);
        }
    }
}


/**
 *  @brief 对margin倒数第二帧进行处理
 * */
void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next ++;

        // 如果地图点被最后一帧看到，由于滑窗，它的起始帧减1
        if (it->start_frame == frame_count)  // 由于要删去的是第frame_count-1帧，最新这一帧frame_count的id就变成了i-1
        {
            it->start_frame --;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;    // j指向第i-1帧
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);   // 删掉特征ft在这个图像帧上的观测量
            if (it->feature_per_frame.size() == 0)                            // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                feature.erase(it);
        }
    }

    // std::cout << "feature_manager.cpp/removeFront(): remove front! \n" << std::endl;
    for (auto it = linefeature.begin(), it_next = linefeature.begin(); it != linefeature.end(); it = it_next)
    {
        it_next ++;

        if (it->start_frame == frame_count)  // 由于要删去的是第frame_count-1帧，最新这一帧frame_count的id就变成了i-1
        {
            it->start_frame --;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;    // j指向第i-1帧
            if (it->endFrame() < frame_count - 1)
                continue;
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin() + j);   // 删掉特征ft在这个图像帧上的观测量
            if (it->linefeature_per_frame.size() == 0)                            // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                linefeature.erase(it);
        }
    }

}


// zlc添加：现在运行这个函数
double FeatureManager::compensatedParallax2(const FeaturePerId& it_per_id, int frame_count)
{
    // check the second last frame is keyframe or not
    // parallax betwwen seconde last frame and third last frame

    //  frame_count ：当前帧的id
    //  it_per_id.start_frame ： 特征第一次被测到的 帧id
    //  feature_per_frame 是个向量容器，存着这个特征在每一帧上的观测量。 
    //                    如：feature_per_frame[0]，存的是ft在start_frame上的观测值; feature_per_frame[1]存的是start_frame+1上的观测
    // 找到相邻两帧
    const FeaturePerFrame& frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame& frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    // int r_i = frame_count - 2;
    // int r_j = frame_count - 1;
    // p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;              // 归一化相机坐标系的坐标差

    // 当都是归一化坐标系时，他们两个都是一样的
    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}