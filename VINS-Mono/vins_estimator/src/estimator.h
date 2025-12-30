#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>


class Estimator
{
  public:
    Estimator();

    void setParameter();

    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);                        // IMU数据处理函数入口
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header);// 后端处理图像函数入口
    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r);

    // internal
    void clearState();
    bool initialStructure();                                                // 初始化：视觉 SfM + 视觉-IMU 对齐
    bool visualInitialAlign();                                              // 视觉-IMU 对齐函数，恢复尺度和重力方向
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void solveOdometry();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();


    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;                     // 当前的求解状态标志：初始化 or 非线性优化
    MarginalizationFlag  marginalization_flag;  // 当前的边缘化策略标志
    Vector3d g;                                 // 重力向量
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

    Matrix3d ric[NUM_OF_CAM];                  // 相机到IMU的旋转矩阵    
    Vector3d tic[NUM_OF_CAM];                  // 相机到IMU的平移向量

    Vector3d Ps[(WINDOW_SIZE + 1)];            // 窗口中各帧的位置
    Vector3d Vs[(WINDOW_SIZE + 1)];            // 窗口中各帧的速度
    Matrix3d Rs[(WINDOW_SIZE + 1)];            // 窗口中各帧的旋转矩阵
    Vector3d Bas[(WINDOW_SIZE + 1)];           // 窗口中各帧的加速度偏置
    Vector3d Bgs[(WINDOW_SIZE + 1)];           // 窗口中各帧的陀螺仪偏置
    double td;                                 // 相机与IMU的时间偏移

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];    // 滑动窗口内各帧的 header 信息

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;                      // 上一时刻的加速度和角速度

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;        // 当前滑动窗口中图像的帧数
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    FeatureManager f_manager;                   // 特征点管理器
    MotionEstimator m_estimator;                // 五点法求解器
    InitialEXRotation initial_ex_rotation;      // 外参在线标定器

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;         // 上一次边缘化信息
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;        // 存储滑动窗口内所有图像帧及其对应的特征点和预积分量，初始化阶段会存储所有图像帧，后端优化阶段只存储滑动窗口内的图像帧
    IntegrationBase *tmp_pre_integration;

    //relocalization variable
    bool relocalization_info;
    double relo_frame_stamp;
    double relo_frame_index;
    int relo_frame_local_index;
    vector<Vector3d> match_points;
    double relo_Pose[SIZE_POSE];
    Matrix3d drift_correct_r;
    Vector3d drift_correct_t;
    Vector3d prev_relo_t;
    Matrix3d prev_relo_r;
    Vector3d relo_relative_t;
    Quaterniond relo_relative_q;
    double relo_relative_yaw;
};
