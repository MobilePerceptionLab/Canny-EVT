//
// Created by mpl on 23-3-9.
//

#include "EventCamera.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include "LogMacro.h"

using namespace CannyEVT;

EventCamera::EventCamera(std::string configPath):
mWidth(0), mHeight(0),mK(Eigen::Matrix3d::Zero()), mKinv(Eigen::Matrix3d::Zero()),
mP(Eigen::Matrix<double, 3, 4>::Zero()), mDistort(cv::Mat()),mUndistortM1(cv::Mat()), mUnditortM2(cv::Mat()),
mRbc(Eigen::Matrix3d::Zero()), mtbc(Eigen::Vector3d::Zero())
{
    cv::FileStorage fs( configPath, cv::FileStorage::READ);
    if ( !fs.isOpened() )
        ALERT("[EventCam]", "config not open");
    SUCCESS("[EventCamera]", "open file %s ...", configPath.c_str());

    cv::Mat Rbc = fs["Rbc"].mat();
    cv::cv2eigen(Rbc, mRbc);
    cv::Mat tbc = fs["tbc"].mat();
    cv::cv2eigen(tbc, mtbc);

    mHeight = fs["height"];
    mWidth = fs["width"];
    mDistort = fs["distortion"].mat();
    cv::Mat K = fs["K"].mat();
    std::string distortion_type = fs["model"].string();

    if (distortion_type == "plumb_bob")
    {
        mDType = PLUMB_BOB;
        cv::Mat P;
        if (fs["P"].isNone())
        {
            P = cv::getOptimalNewCameraMatrix(K, mDistort, cv::Size(mWidth, mHeight), 0);
            Eigen::Matrix3d Keigen;
            cv::cv2eigen(P, Keigen);
            mP.block<3, 3>(0, 0) = Keigen;

        }
        else
        {
            P = fs["P"].mat();
            cv::cv2eigen(P, mP);
        }

        cv::initUndistortRectifyMap(K, mDistort, cv::Mat::eye(3, 3, CV_32F),
                                    P, cv::Size(mWidth, mHeight), CV_32F,
                                    mUndistortM1, mUnditortM2);
    }
    else if (distortion_type == "equidistant")
    {
        mDType = EQUIDISTANT;
        cv::Mat P;
        if (fs["P"].isNone())
        {
            cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
                    K, mDistort, cv::Size(mWidth, mHeight), cv::Mat::eye(3, 3, CV_32F), P, 1);
            Eigen::Matrix3d Keigen;
            cv::cv2eigen(P, Keigen);
            mP.block<3, 3>(0, 0) = Keigen;
        }
        else
        {
            P = fs["P"].mat();
            cv::cv2eigen(P, mP);
        }

        cv::fisheye::initUndistortRectifyMap(K, mDistort, cv::Mat::eye(3, 3, CV_32F),
                                             P, cv::Size(mWidth, mHeight), CV_32F,
                                             mUndistortM1, mUnditortM2);
    }
    else
    {
        throw std::invalid_argument("[EventCam]:Unspport type:" + distortion_type);
    }

    mK = mP.block<3, 3>(0, 0);
    mKinv = mK.inverse();

    // mask
    cv::Mat undistMask = cv::Mat::ones(mHeight, mWidth, CV_32F);
    undistortImage(undistMask, undistMask);
    cv::threshold(undistMask, undistMask, 0.999, 255, cv::THRESH_BINARY);
    undistMask.convertTo(undistMask, CV_8U);
    cv::cv2eigen(undistMask, mUndistortRectifyMask);

    std::cout << "*************Event Camera*************\n";
    std::cout << "size(width, height): " << mWidth <<  "," << mHeight << std::endl;
    std::cout << "K(Undistort):\n" << mK << std::endl;
    std::cout << "P:\n" << mP << std::endl;
    std::cout << "Distort:" << mDistort.t() << std::endl;
    std::cout << "Distort Model:" << fs["model"].string() << std::endl;
    fs.release();
}


void EventCamera::undistortImage(const cv::Mat& src, cv::Mat& dest)
{
#ifdef OPENCV3_FOUND
    cv::remap(src, dest, mUndistortM1, mUnditortM2, CV_INTER_LINEAR);
#else
    cv::remap(src, dest, mUndistortM1, mUnditortM2, cv::INTER_LINEAR);
#endif

}

void EventCamera::preComputeRectifiedCoordinate()
{
    cv::Mat cvSrcMask = cvSrcMask.ones(mHeight, mWidth, CV_32F);
    cv::Mat cvDstMask = cvSrcMask.zeros(mHeight, mWidth, CV_32F);
    cv::remap(cvSrcMask, cvDstMask, mUndistortM1, mUnditortM2, cv::INTER_LINEAR);
    cv::threshold(cvDstMask, cvDstMask, 0.999, 255, cv::THRESH_BINARY);
    cvDstMask.convertTo(cvDstMask, CV_8U);
    cv::cv2eigen(cvDstMask, mUndistortRectifyMask);
}

int EventCamera::getHeight()
{
    return mHeight;
}

int EventCamera::getWidth()
{
    return mWidth;
}

Eigen::Matrix<double, 3, 4> EventCamera::getProjectionMatrix(){
    return mP;
}

Eigen::MatrixXi& EventCamera::getUndistortRectifyMask(){
    return mUndistortRectifyMask;
}

Eigen::Matrix3d EventCamera::Rbc()
{
    return mRbc;
}

Eigen::Vector3d EventCamera::tbc()
{
    return mtbc;
}

Eigen::Matrix4d EventCamera::Tbc()
{
    Eigen::Matrix4d Tbc = Eigen::Matrix4d::Identity();
    Tbc.block<3,3>(0,0) = mRbc;
    Tbc.block<3,1>(0,3) = mtbc;
    return Tbc;
}
