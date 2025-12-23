#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

const double FOCAL_LENGTH = 460.0;              // 相机焦距
const int WINDOW_SIZE = 10;                     // 滑动窗口大小
const int NUM_OF_CAM = 1;                       // 相机数量
const int NUM_OF_F = 1000;                      
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;                       // 初始化深度
extern double MIN_PARALLAX;                     // 关键帧选择的视差阈值
extern int ESTIMATE_EXTRINSIC;                  // 是否估计外参

extern double ACC_N, ACC_W;                     // 加速度计噪声和随机游走
extern double GYR_N, GYR_W;                     // 陀螺仪噪声和随机游走

extern std::vector<Eigen::Matrix3d> RIC;        // 相机到IMU的外参旋转矩阵
extern std::vector<Eigen::Vector3d> TIC;        // 相机到IMU的外参平移向量
extern Eigen::Vector3d G;                       // 重力加速度向量

extern double BIAS_ACC_THRESHOLD;               // 加速度计bias阈值
extern double BIAS_GYR_THRESHOLD;               // 陀螺仪bias阈值
extern double SOLVER_TIME;                      // 每次优化的最大时间
extern int NUM_ITERATIONS;                      // 每次优化的最大迭代次数
extern std::string EX_CALIB_RESULT_PATH;        // 外参估计结果保存路径 
extern std::string VINS_RESULT_PATH;            // VINS估计结果保存路径
extern std::string IMU_TOPIC;                   // IMU话题名称
extern double TD;                               // IMU与相机时间偏移
extern double TR;                               // 滚动快门读出时间
extern int ESTIMATE_TD;                         // 是否估计时间偏移
extern int ROLLING_SHUTTER;                     // 是否为滚动快门相机
extern double ROW, COL;                         // 图像高度和宽度


void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
