//
// Created by mpl on 23-3-10.
//

#ifndef EVIT_TIMESURFACE_H
#define EVIT_TIMESURFACE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

#include "type.h"
#include "EventCamera.h"

namespace CannyEVT
{


class TimeSurface
{
public:
    inline static Eigen::MatrixXd fixed_Ts{Eigen::MatrixXd::Zero(480, 640)};

    typedef std::shared_ptr<TimeSurface> Ptr;
    typedef std::shared_ptr<TimeSurface const> ConstPtr;

    TimeSurface(const cv::Mat& eventsHistory, const cv::Mat& PosEventsHistory, const cv::Mat& NegEventsHistory,
                double stamp, double decayFactor, EventCamera::Ptr eventCam);

    TimeSurface(const std::vector<EventMsg::ConstPtr>& eventMsgs, double stamp, double decayFactor,
                EventCamera::Ptr eventCam);
//    TimeSurface(const cv::Mat& Ts, double stamp, double decayFactor);
    TimeSurface(std::vector<EventMsg::ConstPtr>::const_iterator begin, std::vector<EventMsg::ConstPtr>::const_iterator end,
                double stamp, double decayFactor, EventCamera::Ptr eventCam);

    void createMatrix();
    void GenerateAnnf(const cv::Mat& Ts, cv::Mat &annfRow, cv::Mat &annfCol, int radius = 5);

    void GenerateTsGrad(const cv::Mat& Ts, cv::Mat& gradX, cv::Mat& gradY);

    void GenerateInverseTS(const cv::Mat& Ts, cv::Mat& invTs, size_t kernelSize = 5);

    cv::Mat& getTs();
    cv::Mat& getTsPositiveMat();
    cv::Mat& getTsNegativeMat();

    cv::Mat& getTsAnnfCol();
    cv::Mat& getTsAnnfPositiveCol();
    cv::Mat& getTsAnnfNegativeCol();

    cv::Mat& getTsAnnfRow();
    cv::Mat& getTsAnnfPositiveRow();
    cv::Mat& getTsAnnfNegativeRow();

    cv::Mat& getInverseTs();
    cv::Mat& getInverseTsPositive();
    cv::Mat& getInverseTsNegative();

    cv::Mat& getInverseTsGradX();
    cv::Mat& getInverseTsPositiveGradX();
    cv::Mat& getInverseTsNegativeGradX();

    cv::Mat& getInverseTsGradY();
    cv::Mat& getInverseTsPositiveGradY();
    cv::Mat& getInverseTsNegativeGradY();

protected:
    cv::Mat mTsMat;
    cv::Mat mTsPositiveMat;
    cv::Mat mTsNegativeMat;//Ts made by negative events

    cv::Mat mTsAnnfColMat;
    cv::Mat mTsAnnfPositiveColMat;
    cv::Mat mTsAnnfNegativeColMat;

    cv::Mat mTsAnnfRowMat;
    cv::Mat mTsAnnfPositiveRowMat;
    cv::Mat mTsAnnfNegativeRowMat;

    cv::Mat mInverseTsMat;//255 - Ts
    cv::Mat mInverseTsPositiveMat;
    cv::Mat mInverseTsNegativeMat;

    cv::Mat mInverseTsGradMatX;//grad of the 255-Ts
    cv::Mat mInverseTsPositiveGradMatX;
    cv::Mat mInverseTsNegativeGradMatX;

    cv::Mat mInverseTsGradMatY;//grad of the 255-Ts
    cv::Mat mInverseTsPositiveGradMatY;
    cv::Mat mInverseTsNegativeGradMatY;




    double mDecayFactor;
    double mStamp;

public:
    Eigen::MatrixXd mInverseTsMatE;//255 - Ts 
    Eigen::MatrixXd mInverseTsPositiveMatE;
    Eigen::MatrixXd mInverseTsNegativeMatE;

    Eigen::MatrixXd mInverseTsGradMatXE;//grad of the 255-Ts
    Eigen::MatrixXd mInverseTsPositiveGradMatXE;
    Eigen::MatrixXd mInverseTsNegativeGradMatXE;

    Eigen::MatrixXd mInverseTsGradMatYE;//grad of the 255-Ts
    Eigen::MatrixXd mInverseTsPositiveGradMatYE;
    Eigen::MatrixXd mInverseTsNegativeGradMatYE;
};

}


#endif //EVIT_TIMESURFACE_H
