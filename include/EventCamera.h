//
// Created by mpl on 23-3-9.
//

#ifndef CANNY_EVIT_EVENTCAMERA_H
#define CANNY_EVIT_EVENTCAMERA_H

#include <cstddef>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <memory>

namespace CannyEVT
{

enum DistortionModel
{
    PLUMB_BOB,
    EQUIDISTANT
};

class EventCamera
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<EventCamera> Ptr;
    typedef std::shared_ptr<EventCamera const> ConstPtr;

    EventCamera(std::string configPath);

    ~EventCamera() = default;

    void undistortImage(const cv::Mat& src, cv::Mat& dest);

    int getWidth();

    int getHeight();

    Eigen::Matrix<double, 3, 4> getProjectionMatrix();

    void preComputeRectifiedCoordinate();

    Eigen::MatrixXi& getUndistortRectifyMask();

    Eigen::Matrix3d Rbc();
    Eigen::Vector3d tbc();
    Eigen::Matrix4d Tbc();
public:

    inline Eigen::Vector2d World2Cam(const Eigen::Vector3d& p)
    {
        Eigen::Vector3d x_homo = mP.block<3, 3>(0, 0) * p + mP.block<3, 1>(0, 3);
        return x_homo.head(2) / x_homo(2);
    }

protected:
    int mWidth, mHeight;
    Eigen::Matrix3d mK;
    Eigen::Matrix3d mKinv;
    Eigen::Matrix<double, 3, 4> mP;
    cv::Mat mDistort;
    DistortionModel mDType;

    cv::Mat mUndistortM1, mUnditortM2; // undistortMap
    Eigen::Matrix3d mRbc;
    Eigen::Vector3d mtbc;

    Eigen::MatrixXi mUndistortRectifyMask;


};




}



#endif //CANNY_EVIT_EVENTCAMERA_H
