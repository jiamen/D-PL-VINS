#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

// #define FAST


// 判断一个点是否在边界内
bool inBorder(const cv::Point2f& pt)
{
    // 边界大小
    const int BORDER_SIZE = 1;

    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);

    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

// 根据状态位，进行“瘦身” 788025
void reduceVector(vector<cv::Point2f>& v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i ++)
        if (status[i])
            v[j ++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int>& v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i ++)
        if (status[i])
            v[j ++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
    // △ zlc：FAST添加的语句, 创建fast特征点检测器
    p_fast_feature_detector = cv::FastFeatureDetector::create();
}

// 给现有的特征点设置mask，目的为了特征点的均匀化
void FeatureTracker::setMask()
{
    // 如果是鱼眼相机的话
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    // 把跟踪次数、跟踪点坐标、跟踪点序号进行绑定，输入cnt_pts_id中
    for (unsigned int i = 0; i < forw_pts.size(); i ++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    // 利用光流特征，追踪次数多的稳定性好，排前面
    sort(cnt_pts_id.begin(), cnt_pts_id.end(),
         [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    // 把上述三项信息全部清空
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    // 重新遍历特征点，在每个点周围画圈，防止特征点过于稠密
    for (auto& it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)     // 注意255是白，0是黑
        {
            // 把挑选剩下的特征点重新放进容器
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            // Opencv函数，把周围一个园内全部置0，这个区域不允许别的特征点存在，避免特征点过于集中，0是黑
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }

    // △△△△△ zlc添加：下面是FAST添加的部分 △△△△△
#ifdef FAST
    mask_exp = mask.clone();
    for (auto& pt : unstable_pts)
    {
        cv::circle(mask, pt, MIN_DIST, 0, -1);
    }
#endif
}

// 把新的点加入容器，id给-1作为区分
void FeatureTracker::addPoints()
{
    // △△△△△ zlc添加：下面是FAST添加的部分 △△△△△
    for (auto& p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);       // 特征点id, 一开始给这些新的特征点赋值-1， 会在updateID()函数里用个全局变量给他赋值
        track_cnt.push_back(1);  // 初始化特征点的观测次数：1次
    }
}

// △ zlc：下面的函数是FAST添加的
void FeatureTracker::addPoints(int n_max_cnt)
{
    if (Keypts.empty())
    {
        return;
    }

    sort(Keypts.begin(), Keypts.end(),
         [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
             return a.response > b.response;
         });

    int n_add = 0;
    for (auto& Keypt : Keypts)
    {
        if (mask.at<uchar>(Keypt.pt) == 255)
        {
            forw_pts.push_back(Keypt.pt);
            ids.push_back(-1);
            track_cnt.push_back(1);
            cv::circle(mask, Keypt.pt, MIN_DIST, 0, -1);// zlc: prevent close feature selected
            n_add ++;
            if (n_add == n_max_cnt)
            {
                break;
            }
        }
    }
}



/**
 * @brief
 *
 * @param[in] _img 输入图像
 * @param[in] _cur_time 图像的时间戳
 * 1、图像均衡化预处理
 * 2、光流追踪
 * 3、提取新的特征点（如果发布）
 * 4、所有特征点去畸变，计算速度
 */
// △△△△△ 读取图像 △△△△△
void FeatureTracker::readImage(const cv::Mat& _img, const cv::Mat& _depth, double _cur_time, Matrix3d relative_R)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    // too dark or too bright: histogram
    if (EQUALIZE)   // 直方图均衡化，图像太暗或者太亮，提特征点比较难，所以均衡化一下
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());      // 直方图均衡化花费多长时间
    }
    else
        img = _img;

    // 这里forw表示当前帧，cur表示上一帧
    if (forw_img.empty())           // 第一次输入图像，prev_img这个没用
    {
        // curr_img<--->forw_img
        prev_img = cur_img = forw_img = img;
        prev_depth = cur_depth = forw_depth = _depth;       // △△△△△ zlc：新添加的深度值 △△△△△
    }
    else
    {
        forw_img = img;
        forw_depth = _depth;        // △△△△△ zlc：新添加的深度值 △△△△△
    }

    forw_pts.clear();
    unstable_pts.clear();

    // 上一帧有特征点，就可以进行光流追踪了，根据cur_pts计算当前帧特征点forw_pts
    if (cur_pts.size() > 0)         // i时刻的 特征点
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        // 调用opencv函数进行光流追踪
#ifdef FAST
        // Step 1： 通过opencv光流追踪给的状态剔除outlier，追踪成功后，每个成功点的status会置1
        // 光流跟踪的结果放在 forw_pts
        predictPtsInNextFrame(relative_R);
        forw_pts = predict_pts;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts,
                                 status, err, cv::Size(21, 21), 1,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
#else
        // Step 1： 通过opencv光流追踪给的状态剔除outlier，追踪成功后，每个成功点的status会置1
        // 光流跟踪的结果放在 forw_pts
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts,
                                 status, err, cv::Size(21, 21), 3);
#endif

        for (int i = 0; i < int(forw_pts.size()); i ++)
        {
#ifdef FAST
            if (!status[i] && inBorder(forw_pts[i]))
            {
                unstable_pts.push_back(forw_pts[i]);
            }

            else if (status[i] && !inBorder(forw_pts[i]))    // 跟踪成功，但是在图像外的点，设置成跟踪失败
#else
            // step 2：通过图像边界剔除outlier
            if (status[i] && !inBorder(forw_pts[i]))    // 跟踪成功，但是在图像外的点，设置成跟踪失败
#endif
                status[i] = 0;
        }

        // prev_pts 是向量容器， status指示了每个点是否跟踪成功了， reduceVector 是将所有跟踪成功的点放到这个向量容器的前面去， 比如1,0,0,1,1,0变成 1,1,1      
        reduceVector(prev_pts, status);             // 没用到
        reduceVector(cur_pts, status);              // 根据状态位，对上一帧的特征点进行瘦身
        reduceVector(forw_pts, status);             // 根据状态位，对当前帧的特征点进行瘦身
        reduceVector(ids, status);                  // 特征点的id
        reduceVector(cur_un_pts, status);           // △ zlc添加：去畸变后的坐标，  △△△  PL-VINS中去除了这个变量  △△△
        reduceVector(track_cnt, status);            // 被追踪次数

        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }


    for (auto& n : track_cnt)   // 对tracking上的特征的跟踪帧数进行更新，从第i帧成功跟踪到了i+1帧，跟踪帧数+1
        n ++;

    if (PUB_THIS_FRAME)
    {
        // step 3：通过对极约束来剔除outlier，对prev_pts和forw_pts做ransac剔除outlier.  通过计算F矩阵排除outlier
        rejectWithF();


        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();                 // 设置模板，把那些已经检测出特征点的区域给掩盖掉, 其他区域用于检测新的特征点
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        // 比如每个待发布的帧都要保证150个特征点，经过跟踪之后的稳定点是100个，那么还要再提取50个点
        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());  // 如果 当前特征点数目< MAX_CNT,  那就检测一些新的特征点
        if (n_max_cnt > 0)    // 少于最大特征点数目，那就补充新特征的
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;

#ifdef FAST
            TicToc t_t_fast;
            p_fast_feature_detector->detect(forw_img, Keypts, mask);
#else
            // 只有发布才可以提取更多特征点，同时避免提的点进mask
            // 会不会这些点集中？会，不过没关系，他们下一次作为老将就得接受均匀化的洗礼
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);  // 补充一些新的特征点
#endif
        }
        else
        {
            n_pts.clear();
            Keypts.clear();
        }

        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
#ifdef FAST
        addPoints(n_max_cnt);   // △ zlc : modified addPoints
#else
        addPoints();            // 将这个新的特征点加入 到 forw_pts
#endif
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());


    }
    prev_img = forw_img;
    prev_depth = cur_depth;         // △△△△△ 新添加的深度值 △△△△△
    prev_pts = forw_pts;
    prev_un_pts = cur_un_pts;

    cur_img = forw_img;             // 实际上是上一帧的图像
    cur_depth = forw_depth;         // △△△△△ 新添加的深度值 △△△△△
    cur_pts = forw_pts;             // 上一帧的特征点

    // 去畸变，投影至归一化平面，计算点特征速度（pixel/s）
    undistortedPoints();
    prev_time = cur_time;
}


/**
 * @brief  利用F基础矩阵的RANSAC求解方法去除outlier匹配点
 *
 */
void FeatureTracker::rejectWithF()
{
    // 当前被追踪到的光流至少8个点
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_prev_pts(prev_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < prev_pts.size(); i ++)
        {
            Eigen::Vector3d tmp_p;
            // 得到相机归一化坐标系的值
            m_camera->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            // 这里用一个虚拟相机，原因同样参考https://github.com/HKUST-Aerial-Robotics/VINS-Mono/issues/48
            // 这里有个好处就是对F_THRESHOLD和相机无关
            // 投影到虚拟相机的像素坐标系
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        // opencv接口计算本质矩阵，某种意义也是一种对级约束的outlier剔除
        cv::findFundamentalMat(un_prev_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = prev_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);       // △ zlc添加 △
        reduceVector(ids, status);
        reduceVector(track_cnt, status);

        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}


/**
 * @brief
 *
 * @param[in] i
 * @return true
 * @return false
 *  给新的特征点赋上id,越界就返回false
 */
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id ++;   // n_id 是个全局变量，给每个特征点一个独特的id
        return true;
    }
    else
        return false;
}


void FeatureTracker::readIntrinsicParameter(const string& calib_file)
{   // calib_file = config/realsense/realsense_color_config.yaml;
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    // 读到的相机内参赋给m_camera
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}


void FeatureTracker::showUndistortion(const string& name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i ++)
        for (int j = 0; j < ROW; j ++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            // printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i ++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        // cout << trackerData[0].K << endl;
        // printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        // printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            // ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    //cv::imshow(name, undistortedImg);
    //cv::waitKey(0);             // △ zlc改为0，原本是1 △
}

// △△△△△ PL-VINS 中添加的 △△△△△
void FeatureTracker::showUndistortion()
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    cv::Mat undist_map1_, undist_map2_;

    m_camera->initUndistortRectifyMap(undist_map1_,undist_map2_);
    cv::remap(cur_img, undistortedImg, undist_map1_, undist_map2_, CV_INTER_LINEAR);

    //cv::imshow("undist", undistortedImg);
    //cv::waitKey(1);             // waitKey(1)将等待按键1毫秒，它将继续刷新并读取帧。
}


// 当前帧所有点统一去畸变，同时计算特征点速度，用来后续时间戳标定
void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    // cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i ++)
    {
        // 有的之前去过畸变了，这里连同新人重新做一次
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        //https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/0d280936e441ebb782bf8855d86e13999a22da63/camera_model/src/camera_models/PinholeCamera.cc
        //brief Lifts a point from the image plane to its projective ray
        m_camera->liftProjective(a, b);     // 根据相机模型把 平面像素点 投影到 三维空间中
        // 特征点在相机坐标系的归一化坐标
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));

        // id->坐标的map
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        // printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }

    // caculate points velocity，计算特征点速度
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i ++)
        {
            if (ids[i] != -1)       // 不是新添加的特征点
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                // 找到同一个特征点
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    // 得到在归一化平面的速度
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else    // 新添加的特征点
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        // 第一帧的情况
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}

/*
vector<cv::Point2f> FeatureTracker::undistortedPoints()
{
    vector<cv::Point2f> un_pts;
    // cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }

    return un_pts;
}
*/


// △ zlc添加：整个函数的思想是把平面上的点投影到归一化平面上，利用两帧之间的IMU预积分值获得相对位姿变换，并以此预测空间点在下一帧中的空间坐标表示
void FeatureTracker::predictPtsInNextFrame(Matrix3d relative_R)
{
    predict_pts.resize(cur_pts.size());

    for (int i=0; i<cur_pts.size(); i ++)
    {
        Eigen::Vector3d tmp_P;
        m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_P);
        Eigen::Vector3d predict_P = relative_R * tmp_P;
        Eigen::Vector2d tmp_p;
        m_camera->spaceToPlane(predict_P, tmp_p);
        predict_pts[i].x = tmp_p.x();
        predict_pts[i].y = tmp_p.y();
    }
}
