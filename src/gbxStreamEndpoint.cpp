#include "gbxStreamEndpoint.hpp"
#include "navtoolbox.h"
#include <sys/time.h>
#include "mathHelperFunctions.hpp"
#include "estimationNode.hpp"


GbxStreamEndpointGPSKF::GbxStreamEndpointGPSKF(){}

GbxStreamEndpointGPSKF::~GbxStreamEndpointGPSKF() {
  closeSinkStream_();
}

void GbxStreamEndpointGPSKF::configure(ros::NodeHandle &nh)
{}


void GbxStreamEndpointGPSKF::setRosPointer(std::shared_ptr<monitor::monitorNode> rosHandle)
{
    rosHandle_=rosHandle;
    hasRosHandle=true;
    return;
}


bool GbxStreamEndpointGPSKF::openSinkStream_() {
  return true;
}

void GbxStreamEndpointGPSKF::closeSinkStream_() {
  return;
}

bool GbxStreamEndpointGPSKF::isValidSinkStream_() const {
  return true;
}

bool GbxStreamEndpointGPSKF::writeBytes_(const u8* buffer, size_t size) {
  return true;
}


//A2D callback.  Takes in message from A2D, synchronizes with message from SBRTK, then calls UKF update
GbxStreamEndpoint::ProcessReportReturn GbxStreamEndpointGPSKF::processReport_(
    std::shared_ptr<const ReportAttitude2D>&& pReport, const u8 streamId)
{
    ProcessReportReturn retval = ProcessReportReturn::ACCEPTED;
    if(hasRosHandle)
    {rosHandle_->lastA2D_=(ros::Time::now()).toSec();}


    static int zeroTestStatCounterA(0);
    static int lowTestStatCounterA(0);
    static int nddcounterA(0);
    double teststat(pReport->testStat);
    int nDD(pReport->numDD);
    
    //Eliminate junk messages
    if(teststat<0.01)
    {
        if(zeroTestStatCounterA%10==0)
        {
            updateErrorString("A2D publishing empty messages.",0);
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
            updateErrorString("Persistent low A2D teststat: ",teststat);
        }
        lowTestStatCounterA++;
    }

    //Publish message when restored
    if((zeroTestStatCounterA!=0 || lowTestStatCounterA!=0) & teststat>20.0)
    {
        lowTestStatCounterA=0;
        zeroTestStatCounterA=0;
    }

    //Check number of satellites
    if(nDD<=5)
    {
        if(nddcounterA%10==0)
        {
            updateErrorString("A2D has low satellites: ",nDD);
        }
        nddcounterA++;
    }else if(nddcounterA>0)
    {
        nddcounterA=0;
    }

    //Get time
    rosHandle_->lastA2D_=(ros::Time::now()).toSec();

//    int nsize = rosHandle_->gbxErrorStr.size();
//    std::string errStr[nsize] = rosHandle_->gbxErrorStr;
//    rosHandle_->gbxErrorStr.resize(nsize+1);
//    for(int ij=0;ij++;ij<nsize) {rosHandle->gbxErrorStr[ij]=errStr[ij];}
    return retval;    
}


//SBRTK callback.  Takes in message from SBRTK, synchronizes with message from A2D, then calls UKF update
GbxStreamEndpoint::ProcessReportReturn GbxStreamEndpointGPSKF::processReport_(
    std::shared_ptr<const ReportSingleBaselineRtk>&& pReport, const u8 streamId)
{
    ProcessReportReturn retval = ProcessReportReturn::ACCEPTED;
    if(hasRosHandle)
    {rosHandle_->lastSBRTK_=(ros::Time::now()).toSec();}


    static int zeroTestStatCounterS(0);
    static int lowTestStatCounterS(0);
    static int refnetTimeoutCounter(0);
    static int nddcounterS(0);
    double teststat(pReport->testStat());
    int nDD(pReport->numDD());
    double ageOfData(pReport->ageOfReferenceData());
    
    //Eliminate junk messages
    if(teststat<0.01)
    {
        if(zeroTestStatCounterS%10==0)
        {
            updateErrorString("SBRTK publishing empty messages.",0);
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
            updateErrorString("Persistent low SBRTK teststat: ",teststat);
        }
        lowTestStatCounterS++;
    }

    //Refnet timeout
    if(ageOfData > 5.0)
    {
        if(refnetTimeoutCounter%10==0)
        {
            updateErrorString("REFNET TIMEOUT AT: ",ageOfData);
        }
        refnetTimeoutCounter++;
    }else if(refnetTimeoutCounter!=0)
    {
        refnetTimeoutCounter=0;
    }

    if((zeroTestStatCounterS!=0 || lowTestStatCounterS!=0) & teststat>20.0)
    {
        lowTestStatCounterS=0;
        zeroTestStatCounterS=0;
    }

    //Check number of satellites
    if(nDD<=5)
    {
        if(nddcounterS%10==0)
        {
            updateErrorString("SBRTK has low satellites: ",nDD);
        }
        nddcounterS++;
    }else if(nddcounterS>0)
    {
        nddcounterS=0;
    }

    return retval;
}


void GbxStreamEndpoint::updateErrorString(const std::string &str, const double &val)
{
    if(!hasRosHandle){return;}
    int nsize = rosHandle_->gbxErrorStr.size();
    std::string errStr[nsize] = rosHandle_->gbxErrorStr;
    double errDbl[nsize] = rosHandle_->gbxErrorNum;
    rosHandle_->gbxErrorStr.resize(nsize+1);
    rosHandle_->gbxErrorNum.resize(nsize+1);
    for(int ij=0;ij++;ij<nsize) {
        rosHandle->gbxErrorStr[ij]=errStr[ij];
        rosHandle->gbxErrorNum[ij]=errDbl[ij];
    }

    return;
}
