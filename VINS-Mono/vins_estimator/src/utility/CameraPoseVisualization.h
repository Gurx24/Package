#pragma once

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>


/**
 * 这个类用于在RViz中可视化相机位姿
 * RViz 里看到的那些沿着轨迹移动的绿色“相机小框框”（视锥体），以及连接回环的红线段，都是通过这个类生成并发布的
 */
class CameraPoseVisualization {
public:
	std::string m_marker_ns;

	CameraPoseVisualization(float r, float g, float b, float a);
	
	void setImageBoundaryColor(float r, float g, float b, float a=1.0);
	void setOpticalCenterConnectorColor(float r, float g, float b, float a=1.0);
	void setScale(double s);
	void setLineWidth(double width);

	// 添加一个相机位姿到可视化序列中，该位姿由世界坐标系下的位置 p 和朝向 q 表示
	void add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
	void reset();

	void publish_by(ros::Publisher& pub, const std_msgs::Header& header);	// 将内存中积攒的所有画图指令(m_markers)打包成一个visualization_msgs::MarkerArray，并通过传入的ROS Publisher发送出去
	void add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);	// 用于表示相邻两帧之间的里程计路径
	void add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);// 当VINS检测到回环(Loop Closure)时，会用这个函数连接当前帧和历史帧
private:
	std::vector<visualization_msgs::Marker> m_markers;
	std_msgs::ColorRGBA m_image_boundary_color;
	std_msgs::ColorRGBA m_optical_center_connector_color;
	double m_scale;
	double m_line_width;

	/**
	 * 成像平面四个顶点在相机坐标系下的坐标
	 * 这些点在 3D 空间中构成了一个金字塔形状（相机的视锥体模型）
	 *   lt0          rt0
	 * 	+------------+
	 *  |            |
	 * 	|            |
	 *  |            |
	 * 	+------------+
	 *   lb0          rb0	
	 */ 
	static const Eigen::Vector3d imlt;
	static const Eigen::Vector3d imlb;
	static const Eigen::Vector3d imrt;
	static const Eigen::Vector3d imrb;

	static const Eigen::Vector3d oc  ;			// 光心(optical center)

	// 用于在图像左上角绘制一个小的“L”形指示器，帮助辨别相机的朝向
	static const Eigen::Vector3d lt0 ;			
	static const Eigen::Vector3d lt1 ;
	static const Eigen::Vector3d lt2 ;
};
