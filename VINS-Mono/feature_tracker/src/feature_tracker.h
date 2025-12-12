#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
  public:
    FeatureTracker();

    void readImage(const cv::Mat &_img,double _cur_time);   // 读取图像并处理

    void setMask();                                         // 设置掩码图像                         

    void addPoints();                                       // 添加特征点                

    bool updateID(unsigned int i);                          // 更新特征点ID

    void readIntrinsicParameter(const string &calib_file);  // 从相机标定文件中读取相机内参

    void showUndistortion(const string &name);              // 显示去畸变效果

    void rejectWithF();                                     // 使用基础矩阵F进行RANSAC剔除错误匹配的特征点              

    void undistortedPoints();                               // 计算去畸变后的特征点位置

    cv::Mat mask;                                       // 限制特征点检测的区域
    cv::Mat fisheye_mask;                               // 鱼眼相机掩码，仅中间区域有效
    cv::Mat prev_img, cur_img, forw_img;                // forw_img:最新的帧T时刻，cur_img:上一帧T-1时刻，prev_img:上上帧T-2时刻 初始化为空容器
    vector<cv::Point2f> n_pts;                          // 新检测到的特征点
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;    // 去畸变前的特征点像素位置
    vector<cv::Point2f> prev_un_pts, cur_un_pts;        // 去畸变后的特征点空间位置
    vector<cv::Point2f> pts_velocity;                   // 特征点速度 
    vector<int> ids;                                    // 特征点索引ID
    vector<int> track_cnt;                              // 特征点被成功追踪的计数
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    camodocal::CameraPtr m_camera;                      // 相机模型指针
    double cur_time;
    double prev_time;

    static int n_id;
};
