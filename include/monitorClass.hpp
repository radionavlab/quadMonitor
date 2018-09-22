#pragma once

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include "quad_monitor/quadErrorList.h"
#include "quad_monitor/quadError.h"
#include "std_msgs/String.h"
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
    void timerCallback(const ros::TimerEvent &event);
    void listCallback(const quad_monitor::quadErrorList::ConstPtr &msg);

 private:
    ros::Subscriber statusSub_;
    std::string quadname_;
    ros::Timer timerPub_;
    int lastNErr_;
    bool isHealthy_;
};

}
