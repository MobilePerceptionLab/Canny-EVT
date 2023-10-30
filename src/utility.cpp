//
// Created by mpl on 23-4-5.
//

#include "utility.h"

using namespace CannyEVT;


void Utility::DrawTs(TimeSurface::Ptr TsPtr, CannyEVT::pCloud cloud, Eigen::Matrix4d Twc, EventCamera::Ptr eventCam,
                     std::string name)
{
    cv::Mat ts = TsPtr->getTs().clone();
    ts.convertTo(ts, CV_8UC1);
    cv::cvtColor(ts, ts, cv::COLOR_GRAY2BGR);
    Eigen::Matrix3d Rwc = Twc.block<3,3>(0, 0);
    Eigen::Vector3d twc = Twc.block<3,1>(0, 3);
    for (size_t i = 0; i < cloud->size(); i++)
    {
        Point pt = cloud->at(i);
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::Vector3d pc = Rwc.transpose() * (p - twc);
        Eigen::Vector2d p2d = eventCam->World2Cam(pc);
        cv::Point cvpt(p2d.x(), p2d.y());
        cv::circle(ts, cvpt, 0, CV_RGB(255, 0, 0), cv::FILLED);
    }
    cv::imshow(name, ts);
    cv::waitKey(10);
}

Eigen::Matrix3d Utility::cayley2rot(const Eigen::Vector3d &cayley)
{
    Eigen::Matrix3d R;
    double scale = 1 + pow(cayley[0], 2) + pow(cayley[1], 2) + pow(cayley[2], 2);

    R(0, 0) = 1 + pow(cayley[0], 2) - pow(cayley[1], 2) - pow(cayley[2], 2);
    R(0, 1) = 2 * (cayley[0] * cayley[1] - cayley[2]);
    R(0, 2) = 2 * (cayley[0] * cayley[2] + cayley[1]);
    R(1, 0) = 2 * (cayley[0] * cayley[1] + cayley[2]);
    R(1, 1) = 1 - pow(cayley[0], 2) + pow(cayley[1], 2) - pow(cayley[2], 2);
    R(1, 2) = 2 * (cayley[1] * cayley[2] - cayley[0]);
    R(2, 0) = 2 * (cayley[0] * cayley[2] - cayley[1]);
    R(2, 1) = 2 * (cayley[1] * cayley[2] + cayley[0]);
    R(2, 2) = 1 - pow(cayley[0], 2) - pow(cayley[1], 2) + pow(cayley[2], 2);

    R = (1 / scale) * R;
    return R;
}

Eigen::Vector3d Utility::rot2cayley(const Eigen::Matrix3d &R)
{
    Eigen::Matrix3d C1;
    Eigen::Matrix3d C2;
    Eigen::Matrix3d C;
    C1 = R - Eigen::Matrix3d::Identity();
    C2 = R + Eigen::Matrix3d::Identity();
    C = C1 * C2.inverse();

    Eigen::Vector3d cayley;
    cayley[0] = -C(1, 2);
    cayley[1] = C(0, 2);
    cayley[2] = -C(0, 1);
    return cayley;
}

// void Utility::visulization(TimeSurface::Ptr TsPtr, EventProblemLM::Ptr EventProblemLMPtr, EventCamera::Ptr eventCam){
//        size_t width, height; //肯定已知，根据具体变量名赋值
//        cv::Mat reprojMap = cv::Mat(cv::Size(width, height), CV_8UC1, cv::Scalar(0));
//        cv::Mat TS = TsPtr->getTs();//取出当前TS
//        TS.convertTo(reprojMap, CV_8UC1);//类型转化
//        cv::cvtColor(reprojMap, reprojMap, CV_GRAY2BGR);

//        Eigen::Matrix3d Rcw = EventProblemLMPtr -> mRwc.transpose();
//        Eigen::Vector3d tcw = -Rcw * EventProblemLMPtr -> mtwc;

//        size_t numVisualization = EventProblemLMPtr -> mResItems.size();//根据选出的点云进行.size()
//        for( size_t i = 0; i<numVisualization; i++){
//            ResidualItem &ri = EventProblemLMPtr ->mResItems[i];
//            Eigen::Vector3d p3d = Rcw * ri.p_ + tcw;//this is a 3d point in camera coordinat
//            Eigen::Vector2d p2d = eventCam -> World2Cam(p3d);//this is a 2D point in camera coordinate. result of world2cam(p3d)
           
//            double z = p3d(2);

//            double zMin = 0;
//            double zMax = 5;//最大最小深度值

//            DrawPoint(1.0 / z, 1.0 / zMin, 1.0 / zMax, Eigen::Vector2d(p2d(0), p2d(1)), reprojMap);
//        }
//        cv::imshow("reprojMap", reprojMap);
//        cv::waitKey(0);
//    }

   void Utility::DrawPoint(double val, double maxRange, double minRange, const Eigen::Vector2d &location, cv::Mat &img) {
       // define the color based on the inverse depth
       int index = floor((val - minRange) / (maxRange - minRange) * 255.0f);
       if (index > 255)
           index = 255;
       if (index < 0)
           index = 0;

       cv::Scalar color = CV_RGB(255.0f * Utility::r[index], 255.0f * Utility::g[index], 255.0f * Utility::b[index]);
       // draw the point
       cv::Point point;
       point.x = location[0];
       point.y = location[1];
       cv::circle(img, point, 1, color, cv::FILLED);
   }

