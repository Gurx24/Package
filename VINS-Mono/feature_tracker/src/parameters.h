#pragma once        // 这是一个预处理指令，用于确保该头文件在单个编译单元中只会被包含一次，避免重复定义错误
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>


// extern关键字用于声明一个全局变量，但不进行定义（即不分配内存）

/**
 * 特征跟踪器参数声明，参数设置在config文件中，通过ROS参数服务器读取    
 */

extern int ROW;                                 // 图像的行数（高度）
extern int COL;                                 // 图像的列数（宽度）
extern int FOCAL_LENGTH;                        // 相机的焦距
const int NUM_OF_CAM = 1;                       // 相机的数量，单目相机默认为1


extern std::string IMAGE_TOPIC;                 // 图像话题名称
extern std::string IMU_TOPIC;                   // IMU话题名称
extern std::string FISHEYE_MASK;                // 鱼眼相机掩码
extern std::vector<std::string> CAM_NAMES;      // 相机名称列表
extern int MAX_CNT;                             // 最大特征点数量
extern int MIN_DIST;                            // 两特征点之间最小距离
extern int WINDOW_SIZE;                         // 滑动窗窗口大小
extern int FREQ;                                // 向后端发布图像的频率，最小为10Hz,如果设置为0则与原始图像频率相同
extern double F_THRESHOLD;                      // RANSAC阈值
extern int SHOW_TRACK;                          // 是否将跟踪图像作为ROS话题发布
extern int STEREO_TRACK;                        // 是否立体跟踪
extern int EQUALIZE;                            // 如果图像太黑或太亮，打开均衡化获取充足的特征点
extern int FISHEYE;                             // 是否为鱼眼相机
extern bool PUB_THIS_FRAME;                     // 是否发布当前帧

void readParameters(ros::NodeHandle &n);        // 函数声明：读取参数的函数，接受一个ROS节点句柄作为参数
