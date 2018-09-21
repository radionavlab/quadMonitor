#pragma once

#include "gbxstreamendpointin.h"
#include "gbxstream.h"
#include "typedefs.h"
#include "report.h"
#include <boost/program_options.hpp>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <csignal>
#include <iostream>
#include <thread>
#include "classes.hpp"
namespace po = boost::program_options;

//Forward declaration must be namespace'd properly
namespace monitor
{
  class monitorNode;
}


class GbxStreamEndpointGPSKF : public GbxStreamEndpoint
{
public:
    GbxStreamEndpointGPSKF();

    // More useful functions
    virtual ~GbxStreamEndpointGPSKF();
    void configure(ros::NodeHandle &nh);
    void donothing(); //test
    void setRosPointer(std::shared_ptr<monitor::monitorNode> rosHandle);
    void updateErrorString(const std::string &str, const double &val);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    virtual bool openSinkStream_() override;
    virtual void closeSinkStream_() override;
    virtual bool isValidSinkStream_() const;
    virtual bool isSinkEndpoint_() const { return true; }
    virtual bool isProcessEndpoint_() const { return true; }
    virtual bool writeBytes_(const u8* buffer, size_t size);

    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
                std::shared_ptr<const ReportSingleBaselineRtk>&& pReport, const u8 streamId);
    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
                std::shared_ptr<const ReportAttitude2D>&& pReport, const u8 streamId);
private:
    std::shared_ptr<monitor::monitorNode> rosHandle_;


};
