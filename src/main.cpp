#include "monitorClass.hpp"
#include "gbxStreamEndpoint.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "monitor");
    ros::NodeHandle nh;

    try
    {
        std::string nodeName = ros::this_node::getName();
        auto monitor = std::make_shared<monitor::monitorNode>(nh);

        //Read name and initialize
        std::string quadname;
        ros::param::get(nodeName+"/quad"+to_string(i+1),quadname);
        monitor->initialize(nh, quadname);

        //GBX stuff
        auto gbxStream = std::make_shared<GbxStream>();
        gbxStream->pauseStream();

        int port = gbxport;
        auto epOutput = std::make_shared<GbxStreamEndpointGPSKF>();
        //epOutput->donothing();
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::SINGLE_BASELINE_RTK);
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::ATTITUDE_2D);
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).enableWhitelist();
        //make endpoint
        auto epInput = std::make_shared<GbxStreamEndpointIN>("192.168.2.2",port);
        gbxStream->resumeStream();
        //configuration
        epOutput->configure(nh);
        epOutput->setRosPointer(monitor);

        //Attachments
        if (!gbxStream->attachSinkEndpoint(epOutput))
        {
            std::cerr << "Attachment failed! (output)" << std::endl;
            return -1;
        }
        if (!gbxStream->attachSourceEndpoint(epInput))
        {
            std::cerr << "Attachment failed! (input)" << std::endl;
            return -1;
        }


        //Enable callbacks and cancel when called
        gbxStream->waitOnSourceDetach();
        gbxStream->detachSinkEndpoint(epOutput.get());
        ros::spin();
    }
    catch(const std::exception &e)
    {
        ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
        return 1;
    }
    return 0;
}

