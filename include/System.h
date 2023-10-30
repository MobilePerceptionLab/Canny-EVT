#pragma once

#ifndef EVIT_SYSTEM_H
#define EVIT_SYSTEM_H

#include <Eigen/Eigen>
#include <memory>
#include <chrono>
#include <cmath>
#include <deque>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <fstream>

#include "Frame.h"
#include "TimeSurface.h"
#include "EventCamera.h"
#include "Optimizer.h"
#include "type.h"

namespace CannyEVT{


class System
{
public:
    System(const std::string& config_path);

    ~System();

    void GrabEventData( const size_t &x, const size_t &y, const double &t, const bool &p );

    void LoadPointCloud();

    void ProcessBackEnd();

    std::vector<FrameData> getMeasurements();

    Frame::Ptr MakeFrame(const FrameData& frameData);

    void Draw();

    void updateHistoryEvent(EventMsg::ConstPtr event);
    void updateHistoryEvent(std::vector<EventMsg::ConstPtr> events);


    Frame::Ptr LastFrame;

protected:
    /* load configuration */
    void ReadYaml(const std::string& config_path);

protected:
    Optimizer::Ptr mOptimizer;
    EventCamera::Ptr mEventCam;

    double mFirstFrameStamp;
    double mCurFrameStamp;

    double mLastTsStamp;
    double mFrameFrequency;

    double mLastEventStamp;
    Frame::Ptr mLastFrame;
    Frame::Ptr mFrameForOpticalFlow;

    std::mutex mDataMutex;
    std::condition_variable con;
    double start_event_t = 0;

    double mTsDecayFactor;

    std::deque<EventMsg::ConstPtr> mEventBuf;

    //global point cloud
    std::vector<pcl::PointNormal *> vPointXYZPtr;
    pCloud mCloud;

    bool bStart_backend = true;

    //result saving
    std::ofstream mOfsPose;

    //visulization
    std::vector<Eigen::Vector3d> vPath_to_draw;
    double mzMax;
    double mzMin;

    /* configuration */
    std::string mResultPath;
    std::string mPcdPath;
    double mStartTime;
    std::queue<Frame::Ptr> mFrameQueue;
    int mQueueSize;


public:
    cv::Mat mPositiveHistoryEvent;
    cv::Mat mNegativeHistoryEvent;
    cv::Mat mHistoryEvent;
};//end of the System class

}


#endif //EVIT_SYSTEM_H
