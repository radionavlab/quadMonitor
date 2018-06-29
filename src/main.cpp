#include "monitorClass.hpp"

namespace monitor
{
monitorNode::monitorNode()
{

}


monitorNode::monitorNode(ros::NodeHandle &nh, std::string &quadname)
{
    initialize(nh,quadname);
}


void monitorNode::initialize(ros::NodeHandle &nh, std::string &quadname)
{
    quadname_ = quadname;
    rtkSub_ = nh.subscribe("SingleBaselineRTK",1,&monitorNode::singleBaselineRTKCallback,
                                                        this, ros::TransportHints().unreliable());
    a2dSub_ = nh.subscribe("Attitude2D",1,&monitorNode::attitude2DCallback,
                                                        this, ros::TransportHints().unreliable());
    twSub_ = nh.subscribe("twUpdate",1,&monitorNode::twCallback,
                                                        this, ros::TransportHints().unreliable());
    mavCapSub_ = nh.subscribe("mavros/mocap/pose",1,&monitorNode::mocapCallback,
                                                        this, ros::TransportHints().unreliable());
    mavPoseSub_ = nh.subscribe("mavros/local_position/pose",1,&monitorNode::attitude2DCallback,
                                                        this, ros::TransportHints().unreliable());    
    lastSBRTK_=-1.0; lastA2D_=-1.0; lastMavpose_=-1.0; lastMocap_=-1.0;

    bool oneshot=false;
    timerPub_ = nh.createTimer(ros::Duration(1.0), &monitorNode::timerCallback, this, oneshot);
}

void monitorNode::timerCallback(const ros::TimerEvent &event)
{
    double thisTime = (ros::Time::now()).toSec();
    double dtsbrtk = thisTime - lastSBRTK_;
    double dta2d = thisTime - lastA2D_;
    double dtmocap = thisTime - lastMocap_;
    double dtmavpose = thisTime - lastMavpose_;

    if(dtmavpose > 1.0 && dtmocap > 1.0 && dta2d > 1.0 && dtsbrtk > 1.0)
    {
        ROS_INFO("ERROR: %s LOST", quadname_.c_str());
    }else if(dtsbrtk > 1.0 && dta2d > 1.0)
    {
        ROS_INFO("ERROR: GPS missing for %s",quadname_.c_str());
    }else if(dtsbrtk > 1.0)
    {
        ROS_INFO("ERROR: SBRTK missing for %s",quadname_.c_str());
    }else if(dta2d > 1.0)
    {
        ROS_INFO("ERROR: A2D missing for %s",quadname_.c_str());
    }else if(dtmocap > 1.0)
    {
        ROS_INFO("ERROR: GPSKF not publishing for %s",quadname_.c_str());
    }else if(dtmavpose > 1.0)
    {
        ROS_INFO("ERROR: Frame alignment topic missing for %s",quadname_.c_str());
    }
}


void monitorNode::singleBaselineRTKCallback(const gbx_ros_bridge_msgs::SingleBaselineRTK::ConstPtr &msg)
{
    static int zeroTestStatCounterS(0);
    static int lowTestStatCounterS(0);
    static int refnetTimeoutCounter(0);
    static int nddcounterS(0);
    double teststat(msg->testStat);
    int nDD(msg->numDD);
    double ageOfData(msg->ageOfReferenceData);
    
    //Eliminate junk messages
    if(teststat<0.01)
    {
        if(zeroTestStatCounterS%10==0)
        {
            ROS_INFO("ERROR: SBRTK publishing empty messages on %s", quadname_.c_str());
        }
        zeroTestStatCounterS++;
    }else if(zeroTestStatCounterS!=0)
    {
        zeroTestStatCounterS=0;
        //Do not publish a clear message because teststats might be zero
    }

    //Check signal reliability
    if(teststat<20.0 & teststat>0.01)
    {
        if(lowTestStatCounterS%10==0)
        {
            ROS_INFO("WARN: SBRTK teststat=%f for %s", teststat,quadname_.c_str());
        }
        lowTestStatCounterS++;
    }

    //Refnet timeout
    if(ageOfData > 0.50)
    {
        if(refnetTimeoutCounter%10==0)
        {
            ROS_INFO("ERROR: Refnet timeout for %s", quadname_.c_str());
        }
        refnetTimeoutCounter++;
    }else if(refnetTimeoutCounter!=0)
    {
        refnetTimeoutCounter=0;
        ROS_INFO("Refnet reconnected for %s",quadname_.c_str());
    }

    //Publish message when restored
    if((zeroTestStatCounterS!=0 || lowTestStatCounterS!=0) & teststat>20.0)
    {
        ROS_INFO("SBRTK has recovered for %s", quadname_.c_str());
        lowTestStatCounterS=0;
        zeroTestStatCounterS=0;
    }

    //Check number of satellites
    if(nDD<=5)
    {
        if(nddcounterS%10==0)
        {
            ROS_INFO("WARN: SBRTK numDD = %d for %s",nDD,quadname_.c_str());
        }
        nddcounterS++;
    }else if(nddcounterS>0)
    {
        nddcounterS=0;
    }

    //Get time
    lastSBRTK_=(ros::Time::now()).toSec();
}


void monitorNode::attitude2DCallback(const gbx_ros_bridge_msgs::Attitude2D::ConstPtr &msg)
{
    static int zeroTestStatCounterA(0);
    static int lowTestStatCounterA(0);
    static int nddcounterA(0);
    double teststat(msg->testStat);
    int nDD(msg->numDD);
    
    //Eliminate junk messages
    if(teststat<0.01)
    {
        if(zeroTestStatCounterA%10==0)
        {
            ROS_INFO("ERROR: A2D publishing empty messages on %s", quadname_.c_str());
        }
        zeroTestStatCounterA++;
    }else if(zeroTestStatCounterA!=0)
    {
        zeroTestStatCounterA=0;
        //Do not publish a clear message because teststats might be zero
    }

    //Check signal reliability
    if(teststat<20.0 & teststat>0.01)
    {
        if(lowTestStatCounterA%10==0)
        {
            ROS_INFO("WARN: A2D teststat=%f for %s", teststat,quadname_.c_str());
        }
        lowTestStatCounterA++;
    }

    //Publish message when restored
    if((zeroTestStatCounterA!=0 || lowTestStatCounterA!=0) & teststat>20.0)
    {
        ROS_INFO("A2D has recovered for %s", quadname_.c_str());
        lowTestStatCounterA=0;
        zeroTestStatCounterA=0;
    }

    //Check number of satellites
    if(nDD<=5)
    {
        if(nddcounterA%10==0)
        {
            ROS_INFO("WARN: A2D numDD = %d for %s",nDD,quadname_.c_str());
        }
        nddcounterA++;
    }else if(nddcounterA>0)
    {
        nddcounterA=0;
    }

    //Get time
    lastA2D_=(ros::Time::now()).toSec();
}


void monitorNode::twCallback(const gps_kf::twUpdate::ConstPtr &msg)
{
    double tw = msg->estimatedTW;
    if(tw <= 1.4)
    {
        ROS_INFO("WARN: T/W = %f for %s",tw,quadname_.c_str());
    }
}


void monitorNode::mocapCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    lastMocap_ = (ros::Time::now()).toSec();
}


void monitorNode::mavposeCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    lastMavpose_ = (ros::Time::now()).toSec();
    static int upsidedownCounter(0);
    
    //Check 3,3 element of R to determine if quad is upside-down
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double R33 = 1-2*qx*qx - 2*qy*qy;
    if(R33 < -0.7)
    {
        if(upsidedownCounter%10==0)
        {
            ROS_INFO("ERROR: %s is upside-down",quadname_.c_str());
        }
        upsidedownCounter++;
    }else if(upsidedownCounter>0)
    {
        ROS_INFO("Orientation for %s recovered.",quadname_.c_str());
        upsidedownCounter=0;
    }
}


} //end namespace


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

