#include "System.h"
#include "LogMacro.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/eigen.hpp>
#include <pangolin/pangolin.h>
#include "utility.h"

#include <pcl/common/transforms.h>

using namespace CannyEVT;


System::System(const std::string& config_path):
mEventCam(new EventCamera(config_path)), mLastEventStamp(0),mLastFrame(nullptr),
mCloud(new std::vector<Point>()), mOptimizer(nullptr), mFirstFrameStamp(0),
mHistoryEvent(mEventCam->getHeight(), mEventCam->getWidth(), CV_64F, 0.0),
mPositiveHistoryEvent(mEventCam->getHeight(), mEventCam->getWidth(), CV_64F, 0.0),
mNegativeHistoryEvent(mEventCam->getHeight(), mEventCam->getWidth(), CV_64F, 0.0)
{
    //Load data from yaml
    ReadYaml(config_path);

    //Load global point cloud
    LoadPointCloud();
    mOptimizer = std::make_shared<Optimizer>(config_path, mEventCam);

    mOfsPose.open(mResultPath, std::ofstream::out);
    if(!mOfsPose.is_open())
        WARNING("[System]", "mOfsPose is not open");
    SUCCESS("[System]:", "init done");
}

System::~System()
{
    bStart_backend = false;
    mOfsPose.close();
}

void System::ReadYaml(const std::string &config_path )
{
    cv::FileStorage fs(config_path, cv::FileStorage::READ );
    if (!fs.isOpened())
        ALERT("[System]:", "config not open");
    else
    {
        SUCCESS("[System]", "open file %s ...", config_path.c_str());
        mResultPath = fs["resultPath"].string();
        if (fs["resultPath"].isNone())
            WARNING("[System]", "config[resultPath] not set!");

        mPcdPath = fs["pcd_path"].string();
        if (fs["pcd_path"].isNone())
            WARNING("[System]", "config[pcd_path] not set!");

        mStartTime = fs["start_time"];
        if (fs["start_time"].isNone())
            WARNING("[System]", "config[start_time] not set!");

        mTsDecayFactor = fs["ts_decay_factor"];
        if (fs["ts_decay_factor"].isNone())
            WARNING("[System]", "config[ts_decay_factor] not set!");

        mFirstFrameStamp = fs["start_time"];
        mCurFrameStamp = mFirstFrameStamp;
        mLastTsStamp = fs["start_time"];
        if (fs["start_time"].isNone())
            WARNING("[System]", "config[start_time] not set!");

        mFrameFrequency = fs["frame_frequency"];
        if (fs["frame_frequency"].isNone())
            WARNING("[System]", "config[frame_frequency] not set!");
            
        mzMin = fs["zMin"];
        if (fs["zMin"].isNone())
            WARNING("[zMin]", "config[zMin] not set!");

        mzMax = fs["zMax"];
        if (fs["zMax"].isNone())
            WARNING("[zMax]", "config[zMax] not set!");

        mQueueSize = fs["queueSize"];
        if (fs["queueSize"].isNone())
            WARNING("[queueSize]", "config[queueSize] not set!");

    }
    fs.release();
}

void System::LoadPointCloud(){
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in(new pcl::PointCloud<pcl::PointNormal>);
    if (pcl::io::loadPCDFile(mPcdPath, *cloud_in) < 0)
    {
        PCL_ERROR("\a->point cloud does not exist!\n");
        system("pause");
    }
    SUCCESS("[System]", "load %d points", cloud_in->points.size() );

    for(auto p : *cloud_in)
    {

        Eigen::Vector3d p_pos(p.x, p.y, p.z);
        Eigen::Vector3d p_normal(p.normal_x, p.normal_y, p.normal_z);

        mCloud->emplace_back(p_pos.x(), p_pos.y(), p_pos.z(),
                             p_normal.x(), p_normal.y(), p_normal.z());
    }
}


void System::GrabEventData(const size_t &x, const size_t &y, const double &t, const bool &p)
{

    EventMsg::ConstPtr eventMsg(new EventMsg(t, x, y, p));

    if (t < mLastEventStamp)
    {
//        ALERT("[System]:", "Event message in disorder");
        return;
    }
    mLastEventStamp = t;

    if(t < mFirstFrameStamp)
    {
        updateHistoryEvent(eventMsg);
        return;
    }

    mDataMutex.lock();
    mEventBuf.push_back(eventMsg);
    mDataMutex.unlock();

    //notify the backend thread
    con.notify_one();
}

void System::ProcessBackEnd(){
    // bool bfirst = true;

    //test
//     std::cout << "111111111111111111"<< std::endl;
//     Eigen::Matrix4d T_first;
//     T_first<< 0.0365235643, -0.3137547797,  0.9488013369,  0.1284005649,
// -0.9969353010, -0.0771664592,  0.0128586365,  0.0810022731,
//  0.0691811827, -0.9463631862, -0.3156116047, -0.0241857160,
//  0.0000000000,  0.0000000000,  0.0000000000,  1.0000000000;

    while(bStart_backend){
        //get the mutex before reading the data
        std::unique_lock<std::mutex> lk(mDataMutex);

        std::vector<FrameData> measurements;
    
        con.wait(lk, [&measurements, this] { return (measurements = getMeasurements()).size() != 0;});

        lk.unlock();
        for(auto &measurement : measurements){
            //make frame by the measurement
            Frame::Ptr CurrentFrame = MakeFrame(measurement);
            if(CurrentFrame->mStamp < mStartTime) continue;

            if (mLastFrame != nullptr && mFrameForOpticalFlow != nullptr) mOptimizer->OptimizeEventProblem(CurrentFrame->mTs, mCloud, mLastFrame->T, mFrameForOpticalFlow->T, CurrentFrame->T);        
            else if(mLastFrame != nullptr) mOptimizer->OptimizeEventProblem(CurrentFrame->mTs, mCloud, mLastFrame->T, mLastFrame->T, CurrentFrame->T);
            else mOptimizer->OptimizeEventProblem(CurrentFrame->mTs, mCloud, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity(), CurrentFrame->T);
            if(mFrameQueue.size() >= mQueueSize){
                mFrameQueue.push(CurrentFrame);
                mFrameQueue.pop();
                mFrameForOpticalFlow = mFrameQueue.front();
            }
            else mFrameQueue.push(CurrentFrame);
            
            mLastFrame = CurrentFrame;
            // Utility::DrawTs(CurrentFrame->mTs, mCloud, CurrentFrame->T, mEventCam, "new");
            Eigen::Quaterniond q(mLastFrame->T.block<3,3>(0,0));
            mOfsPose << mLastFrame->mStamp << " " << mLastFrame->T.block<3,1>(0,3)(0) 
                                           <<  " " << mLastFrame->T.block<3,1>(0,3)(1)
                                           << " " << mLastFrame->T.block<3,1>(0,3)(2) 
                                           << " " << q.coeffs() << std::endl;
            vPath_to_draw.push_back({mLastFrame->T.block<3,1>(0,3)(0), mLastFrame->T.block<3,1>(0,3)(1), mLastFrame->T.block<3,1>(0,3)(2)});
            
        }

    }
}

std::vector<FrameData> System::getMeasurements()
{
    std::vector<FrameData> measurements;
    while(1)
    {
        if (mEventBuf.empty())
            return measurements;

        if (mEventBuf.back()->ts < mCurFrameStamp)
            return measurements;

        std::vector<EventMsg::ConstPtr> Events;
        while(mEventBuf.front()->ts < mCurFrameStamp)
        {
            Events.emplace_back(mEventBuf.front());
            mEventBuf.pop_front();
        }

        measurements.emplace_back(Events, mCurFrameStamp);

        mCurFrameStamp = mCurFrameStamp + 1 / mFrameFrequency;
        if (measurements.size() >= 2)
            return measurements;
    }
    return measurements;
}

Frame::Ptr System::MakeFrame(const FrameData& frameData)
{
    updateHistoryEvent(frameData.eventData);
    TimeSurface::Ptr ts(new TimeSurface(mHistoryEvent, mPositiveHistoryEvent, mNegativeHistoryEvent,
                                        frameData.stamp, mTsDecayFactor, mEventCam));

    Frame::Ptr F(new Frame(ts, frameData.stamp, mEventCam));
    return F;
}

void System::updateHistoryEvent(EventMsg::ConstPtr event)
{
    if (event->p)
        mPositiveHistoryEvent.at<double>(event->y, event->x) = event->ts;
    else
        mNegativeHistoryEvent.at<double>(event->y, event->x) = event->ts;

    mHistoryEvent.at<double>(event->y, event->x) = event->ts;
}

void System::updateHistoryEvent(std::vector<EventMsg::ConstPtr> events)
{
    for(EventMsg::ConstPtr& msg: events)
        updateHistoryEvent(msg);
}


void System::Draw(){
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);

    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
              pangolin::ProjectionMatrix(1024,768,500, 500, 512, 384, 0.1, 1000),
              pangolin::ModelViewLookAt(-5, 0, 15, 7, 0, 0, 1.0, 0.0, 0.0)
              );

    pangolin::View& d_cam = pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
          .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        glClearColor(0.75f, 0.75f, 0.75f, 0.75f);
        glColor3f(0, 0, 1);
        pangolin::glDrawAxis(3);

        // draw trajactory with black
        glColor3f(0, 0, 0);
        glLineWidth(2);
        glBegin(GL_LINES);
        int nPath_size = vPath_to_draw.size();

        for(int i = 0; i < nPath_size-1; ++i)
        {

            double zc = 5*std::sqrt(vPath_to_draw[i].x()*vPath_to_draw[i].x() + vPath_to_draw[i].y()*vPath_to_draw[i].y() + vPath_to_draw[i].z()*vPath_to_draw[i].z());


            int index = floor((1/zc - 1/mzMin) / (1/mzMax - 1/mzMin) * 255.0f);
            if (index > 255)
                index = 255;
            if (index < 0)
                index = 0;

            glColor3f(1.0f * Utility::b[index], 1.0f * Utility::g[index], 1.0f * Utility::r[index]);

            glVertex3f(vPath_to_draw[i].x(), vPath_to_draw[i].y(), vPath_to_draw[i].z());
            glVertex3f(vPath_to_draw[i+1].x(), vPath_to_draw[i+1].y(), vPath_to_draw[i+1].z());
        }
        glEnd();

        // draw point cloud
        glPointSize(1);
        glBegin(GL_POINTS);
        glColor3f(0.0,0.0,0.0);
        for(auto p : *mCloud){
            double x = p.x;
            double y = p.y;
            double z = p.z;
            
            // get the depth in camera coordinate
            Eigen::Vector4d t;
            t << x, y, z, 1;
            if(mLastFrame) t = mLastFrame->T.inverse() * t;
            double zc = t(2);


            int index = floor((1/zc - 1/mzMin) / (1/mzMax - 1/mzMin) * 255.0f);
            if (index > 255)
                index = 255;
            if (index < 0)
                index = 0;

            glColor3f(1.0f * Utility::b[index], 1.0f * Utility::g[index], 1.0f * Utility::r[index]);

            glVertex3d(x, y, z);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
