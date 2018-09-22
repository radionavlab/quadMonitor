#include "monitorClass.hpp"

namespace monitor
{
monitorNode::monitorNode(){}


monitorNode::monitorNode(ros::NodeHandle &nh, std::string &quadname)
{
    initialize(nh,quadname);
}


void monitorNode::initialize(ros::NodeHandle &nh, std::string &quadname)
{
    quadname_ = quadname;
    statusSub_=nh.subscribe(quadname+"/monitor",1,&monitorNode::listCallback, this, ros::TransportHints().tcpNoDelay());
    isHealthy_ = true;

    bool oneshot=false;
    timerPub_ = nh.createTimer(ros::Duration(10.0), &monitorNode::timerCallback, this, oneshot);
}

void monitorNode::timerCallback(const ros::TimerEvent &event)
{
    static int counter(0);
    if(isHealthy_)
    {ROS_INFO("%s is healthy.",quadname_.c_str());}
    else if(!isHealthy_ && lastNErr_==0)
    {isHealthy_ = true;}
}


void monitorNode::listCallback(const quad_monitor::quadErrorList::ConstPtr &msg)
{
    int numerrors = msg->numErrors;
    lastNErr_ = 0;
    if(numerrors>=1)
    {
        double errV;
        for(int ij=0;ij++;ij<=numerrors)
        {
            errV = msg->errorMessages[ij].data;
            std_msgs::String errM = msg->errorMessages[ij].errType;
            if(errV<=0.0001 && errV>=-0.0001)
            {ROS_INFO("%s",(errM.data).c_str() );}
            else
            {ROS_INFO("%s %f",(errM.data).c_str(), errV);}
        }
        isHealthy_ = false;
    }
}

} //end namespace


//Snippet just in case whatever version of std is on the computer does not have it
std::string to_string( int x ) {
  int length = snprintf( NULL, 0, "%d", x );
  assert( length >= 0 );
  char* buf = new char[length + 1];
  snprintf( buf, length + 1, "%d", x );
  std::string str( buf );
  delete[] buf;
  return str;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "monitor");
    ros::NodeHandle nh;

    try
    {
        std::string nodeName = ros::this_node::getName();
        int numquads;
        ros::param::get(nodeName+"/numquads", numquads);
        monitor::monitorNode monitor[numquads];
        ROS_INFO("Launching monitor for %d quads",numquads);

        //Read name and initialize
        std::string quadname;
        for(int i=0; i<numquads; i++)
        {
            ros::param::get(nodeName+"/quad"+to_string(i+1),quadname);
            monitor[i].initialize(nh, quadname);
            ROS_INFO("%s initialized",quadname.c_str());
        }
        ros::spin();
    }
    catch(const std::exception &e)
    {
        ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
        return 1;
    }
    return 0;
}

