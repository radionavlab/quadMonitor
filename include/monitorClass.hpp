#pragma once

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <quadMonitor/quadError.h>
#include <quadMonitor/quadErrorList.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <string>
#include <iostream>

namespace monitor
{

class monitorNode
{
 public:
 	monitorNode();
    monitorNode(ros::NodeHandle &nh, std::string &quadname);

    void initialize(ros::NodeHandle &nh, std::string &quadname);
    void singleBaselineRTKCallback(const gbx_ros_bridge_msgs::SingleBaselineRTK::ConstPtr &msg);
    void attitude2DCallback(const gbx_ros_bridge_msgs::Attitude2D::ConstPtr &msg);
//    void twCallback(const gps_kf::twUpdate::ConstPtr &msg);
    void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void mavposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void timerCallback(const ros::TimerEvent &event);
    void updateErrorString(const std::string &str, const double &val);

    std::vector<std::string> gbxErrorsStr;
    std::vector<std::double> gbxErrorNum;
    double lastSBRTK_, lastA2D_;

 private:

    Eigen::Vector3d xPrev_;
    ros::Publisher monitorPub_;
    ros::Subscriber rtkSub_, a2dSub_, mavCapSub_, mavPoseSub_, twSub_;
    std::string quadname_;
    ros::Timer timerPub_;
    double lastMavpose_, lastMocap_;
};

}
