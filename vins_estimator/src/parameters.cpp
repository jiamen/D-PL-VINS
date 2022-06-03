#include "parameters.h"

double BASE_LINE;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};       // 重力向量

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;                // △△△△△ zlc: PL-VINS中原本是去除 △△△△△
int ROLLING_SHUTTER;            // △△△△△ zlc: PL-VINS中原本是去除 △△△△△


int LOOP_CLOSURE = 0;
int MIN_LOOP_NUM;
// std::string CAM_NAMES;
std::vector<std::string> CAM_NAMES; // △△△△△ zlc: 这里按照双目来写 △△△△△
std::string PATTERN_FILE;
std::string VOC_FILE;
std::string IMAGE_TOPIC;
std::string DEPTH_TOPIC;        // △△△△△ zlc: FAST中添加的变量 △△△△△


std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string VINS_FOLDER_PATH;
int MAX_KEYFRAME_NUM;
std::string IMU_TOPIC;

int IMAGE_ROW, IMAGE_COL;       // △△△△△ zlc: 这里改了ROW,COL,而且RGBD中是double类型 △△△△△
double TD, TR;                  // △△△△△ zlc: PL-VINS中原本是去除 △△△△△




// △△△△△ zlc添加： 下面是Feature中的参数 △△△△△
int MAX_CNT;
int MAX_CNT_SET;
int MIN_DIST;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int EQUALIZE;
int FISHEYE;
std::string FISHEYE_MASK;

int STEREO_TRACK;
bool PUB_THIS_FRAME;

double ROW, COL;
int IMAGE_SIZE;

double DEPTH_MIN_DIST;
double DEPTH_MAX_DIST;

std::vector<std::string> SEMANTIC_LABEL;    // △ zlc添加：FAST新添加的视觉标签
std::vector<std::string> STATIC_LABEL;      // △ zlc添加：FAST新添加的视觉标签
std::vector<std::string> DYNAMIC_LABEL;     // △ zlc添加：FAST新添加的视觉标签
// △△△△△ zlc添加： 上面是Feature中的参数 △△△△△




template <typename T>
T readParam(ros::NodeHandle& n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}


void readParameters(ros::NodeHandle& n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    // config_file = "/home/zlc/cpl-tmp_ws/src/PL-VINS/config/euroc/realsense_color_config.yaml";
    // config_file = "/home/zlc/cpl-tmp_ws/src/PL-VINS/config/euroc/lifelong_config.yaml";
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }


    // VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");    // △△△△△ 在launch文件中给定，PL-VINS中添加的 △△△△△
    VINS_FOLDER_PATH = "/home/zlc/cpl-tmp_ws/src/PL-VINS";
    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["depth_topic"] >> DEPTH_TOPIC;                        // △△△△△ zlc: FAST中添加的变量 △△△△△
    fsSettings["imu_topic"] >> IMU_TOPIC;


    MAX_CNT = fsSettings["max_cnt"];
    MAX_CNT_SET = MAX_CNT;
    MIN_DIST = fsSettings["min_dist"];

    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    // CAM_NAMES = config_file;
    CAM_NAMES.push_back(config_file);       // △ zlc添加：这里是按照双目相机写的

    // △△△△△ zlc添加： FAST中新添加的，设置深度相机的有效范围
    DEPTH_MIN_DIST = fsSettings["depth_min_dist"];
    DEPTH_MAX_DIST = fsSettings["depth_max_dist"];


    // WINDOW_SIZE = 20;            // △：zlc添加：窗口大小前端中原本为20帧，现为10帧
    STEREO_TRACK = false;           // 双目跟踪
    // FOCAL_LENGTH = 460;          // 焦距，shan:What's this?---seems a virtual focal used in rejectWithF.
    PUB_THIS_FRAME = false;         // 是否发布此帧，初始设置为false

    if (FREQ == 0)
        FREQ = 100;




    IMAGE_COL = fsSettings["image_width"];
    IMAGE_ROW = fsSettings["image_height"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    IMAGE_SIZE = ROW * COL;
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);


    SOLVER_TIME = fsSettings["max_solver_time"];                    // 单次优化最大求解时间
    NUM_ITERATIONS = fsSettings["max_num_iterations"];              // 单次优化最大迭代次数
    MIN_PARALLAX = fsSettings["keyframe_parallax"];                 // 根据视差确定关键帧
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;                       // 这里在yaml中指定
    // VINS_RESULT_PATH = VINS_FOLDER_PATH + VINS_RESULT_PATH;
    // VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop11.csv";    // △ zlc添加
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop11.txt";    // △ zlc添加
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);    // △ zlc添加
    fout.close();                                                   // △ zlc添加
    // std::ofstream foutC("/home/zlc/cpl-tmp_ws/src/PL-VINS/Trajactory/new_tum_evo/tum_plvins.txt", std::ios::out);
    // foutC.close();
    // std::ofstream foutC1("/home/zlc/cpl-tmp_ws/src/PL-VINS/Trajactory/new_tum_evo/evo_plvins.txt", std::ios::out);
    // foutC1.close();

    // IMU、图像相关参数
    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    // ROW = fsSettings["image_height"];                       // △ zlc添加的，yaml文件中指定
    // COL = fsSettings["image_width"];                        // △ zlc添加的，yaml文件中指定
    // ROS_INFO("ROW: %f COL: %f ", ROW, COL);                 // △ zlc添加的，yaml文件中指定


    // △△△△△ zlc添加：语义标签读取 △△△△△
    for (auto iter : fsSettings["semantic_label"])
    {
        SEMANTIC_LABEL.emplace_back(iter.string());
    }
    for (auto iter : fsSettings["static_label"])
    {
        STATIC_LABEL.emplace_back(iter.string());
    }
    for (auto iter : fsSettings["dynamic_label"])
    {
        DYNAMIC_LABEL.emplace_back(iter.string());
    }


    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        fsSettings["ex_calib_result_path"] >> EX_CALIB_RESULT_PATH;
        EX_CALIB_RESULT_PATH = VINS_FOLDER_PATH + EX_CALIB_RESULT_PATH;     // △△△△△ 这里注意存储位置 △△△△△

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            fsSettings["ex_calib_result_path"] >> EX_CALIB_RESULT_PATH;
            EX_CALIB_RESULT_PATH = VINS_FOLDER_PATH + EX_CALIB_RESULT_PATH; // △△△△△ 这里注意存储位置 △△△△△
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    LOOP_CLOSURE = fsSettings["loop_closure"];
    if (LOOP_CLOSURE == 1)
    {
        fsSettings["voc_file"] >> VOC_FILE;;
        fsSettings["pattern_file"] >> PATTERN_FILE;
        VOC_FILE = VINS_FOLDER_PATH + VOC_FILE;
        PATTERN_FILE = VINS_FOLDER_PATH + PATTERN_FILE;
        MIN_LOOP_NUM = fsSettings["min_loop_num"];
        // CAM_NAMES = config_file;
        CAM_NAMES.push_back(config_file);
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;
    MAX_KEYFRAME_NUM = 1000;                // △△△△△ zlc添加： 关键帧最大数量，用于控制闭环优化位姿图管理？？？？控制规模？？？？


    // △ zlc添加：时延相关参数
    TD = fsSettings["td"];                      // tf: 0.0
    ESTIMATE_TD = fsSettings["estimate_td"];    // estimate_td: 1
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);


    // △ zlc添加：相机是全局快门还是滚动快门：数据集中是滚动快门，我自己的相机是全局快门  或者可以自己设置相机？？？
    ROLLING_SHUTTER = fsSettings["rolling_shutter"];    // rolling_shutter: 1
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];  // rolling_shutter_tr: 0.033
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }


    // 双目的参数，这里的参数在config/stereo_simdata_fix_extrinsic.yaml文件中定义，是否是用在双目中？？？？？
    if (!fsSettings["model_type"].isNone())
        BASE_LINE = static_cast<double>(fsSettings["base_line"]);
    else
        BASE_LINE = - 1.0;


    fsSettings.release();
}
