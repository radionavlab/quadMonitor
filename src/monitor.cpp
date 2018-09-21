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
    mavCapSub_ = nh.subscribe("mavros/mocap/pose",1,&monitorNode::mocapCallback,
                                                        this, ros::TransportHints().unreliable());
    mavPoseSub_ = nh.subscribe("mavros/local_position/pose",1,&monitorNode::mavposeCallback,
                                                        this, ros::TransportHints().unreliable());
    errPub_ = nh.advertise<quadMonitor::quadErrorList>("monitor",1);
    lastSBRTK_=-1.0; lastA2D_=-1.0; lastMavpose_=-1.0; lastMocap_=-1.0;

    bool oneshot=false;
    timerPub_ = nh.createTimer(ros::Duration(1.0), &monitorNode::timerCallback, this, oneshot);


    xPrev_(0)=9001;xPrev_(1)=9001;xPrev_(2)=9001;
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
        updateErrorString("Quad lost.",0);
    }else if(dtsbrtk > 1.0 && dta2d > 1.0)
    {
        updateErrorString("All GPS topics are missing.",0);
    }else if(dtsbrtk > 1.0)
    {
        updateErrorString("SBRTK missing",0);
    }else if(dta2d > 1.0)
    {
        updateErrorString("A2D missing",0);
    }
    else if(dtmavpose > 1.0)
    {
        updateErrorString("mavros/mocap/pose missing",0);
    }

    quadMonitor::quadErrorList msg;
    msg.header.stamp = ros::Time::now();
    int nn = gbxErrorStr.size();
    msg.numErrors = nn;
    msg.errorMessages.resize(nn);
    for(int ij=0;ij++;ij<nn)
    {
        msg.errorMessages[ij].errType = gbxErrorStr[ij];
        msg.errorMessages[ij].data = gbxErrorNum[ij];
    }

    errPub_.publish(msg);
    gbxErrorStr.resize(0);
}


void monitorNode::mocapCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    lastMocap_ = (ros::Time::now()).toSec();
}


void monitorNode::mavposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    static int upsidedownCounter(0);
    static int integerLockCounter(0);
    
    //Check 3,3 element of R to determine if quad is upside-down
    double qx = msg->pose.orientation.x;
    double qy = msg->pose.orientation.y;
    double R33 = 1-2*qx*qx - 2*qy*qy;
    if(R33 < -0.7)
    {
        if(upsidedownCounter%2==0)
        {
            updateErrorString("Quad is upside-down.", 0);
        }
        upsidedownCounter++;
    }else if(upsidedownCounter>0)
    {
        ROS_INFO("Orientation for %s recovered.",quadname_.c_str());
        upsidedownCounter=0;
    }

    //If on ground and z!=0
    double xx(msg->pose.position.x);
    double yy(msg->pose.position.y);
    double zz(msg->pose.position.z);
    double vx((xx-xPrev_(0))/0.2);
    double vy((yy-xPrev_(1))/0.2);
    double vz((zz-xPrev_(2))/0.2);
    double speed2(vx*vx + vy*vy + vz*vz);
    if(speed2 < 0.2 && (zz < -0.10 || zz > 0.10))
    {
        if(integerLockCounter%2==0)
        {
            updateErrorString("Stationary but has nonzero altitude.  Check GPS integers.",0);
        }
        integerLockCounter++;
    }else if(speed2 < 0.2 && integerLockCounter > 0)
    {
        integerLockCounter=0;
    }
    //update xPrev_
    xPrev_(0)=xx;
    xPrev_(1)=yy;
    xPrev_(2)=zz;

    lastMavpose_ = (ros::Time::now()).toSec();
}


void monitorNode::updateErrorString(const std::string &str, const double &val)
{
    if(!hasRosHandle){return;}
    int nsize = gbxErrorStr.size();
    std::string errStr[nsize] = gbxErrorStr;
    double errDbl[nsize] = gbxErrorNum;
    gbxErrorStr.resize(nsize+1);
    gbxErrorNum.resize(nsize+1);
    for(int ij=0;ij++;ij<nsize) {
        rosHandle->gbxErrorStr[ij]=errStr[ij];
        rosHandle->gbxErrorNum[ij]=errDbl[ij];
    }

    return;
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


