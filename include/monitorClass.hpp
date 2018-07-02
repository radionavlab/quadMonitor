#pragma once

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <gbx_ros_bridge_msgs/SingleBaselineRTK.h>
#include <gbx_ros_bridge_msgs/Attitude2D.h>
#include <nav_msgs/Odometry.h>
#include <gps_kf/twUpdate.h>
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
    void twCallback(const gps_kf::twUpdate::ConstPtr &msg);
    void mocapCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void mavposeCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void timerCallback(const ros::TimerEvent &event);

 private:

    ros::Publisher monitorPub_;
    ros::Subscriber rtkSub_, a2dSub_, mavCapSub_, mavPoseSub_, twSub_;
    std::string quadname_;
    ros::Timer timerPub_;
    double lastSBRTK_, lastA2D_, lastMavpose_, lastMocap_;

};

}