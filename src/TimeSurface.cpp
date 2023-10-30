//
// Created by mpl on 23-3-10.
//

#include "TimeSurface.h"
#include "opencv2/core/eigen.hpp"
using namespace CannyEVT;



TimeSurface::TimeSurface(const std::vector<EventMsg::ConstPtr> &eventMsgs, double stamp,
                         double decayFactor, EventCamera::Ptr eventCam)
:mStamp(stamp), mDecayFactor(decayFactor), mTsMat(eventCam->getHeight(), eventCam->getWidth(), CV_32F, 0.0),
mTsPositiveMat(eventCam->getHeight(), eventCam->getWidth(), CV_32F, 0.0),
mTsNegativeMat(eventCam->getHeight(), eventCam->getWidth(), CV_32F, 0.0)
{
    eventCam->undistortImage(mTsMat, mTsMat);
    eventCam->undistortImage(mTsPositiveMat, mTsPositiveMat);
    eventCam->undistortImage(mTsNegativeMat, mTsNegativeMat);
    createMatrix();

}
TimeSurface::TimeSurface(std::vector<EventMsg::ConstPtr>::const_iterator begin,
                         std::vector<EventMsg::ConstPtr>::const_iterator end,
                         double stamp, double decayFactor, EventCamera::Ptr eventCam):
mStamp(stamp), mDecayFactor(decayFactor), mTsMat(eventCam->getHeight(), eventCam->getWidth(), CV_32F, 0.0),
mTsPositiveMat(eventCam->getHeight(), eventCam->getWidth(), CV_32F, 0.0),
mTsNegativeMat(eventCam->getHeight(), eventCam->getWidth(), CV_32F, 0.0)
{
    for (auto iter = begin; iter != end; iter++)
    {
        double dt = stamp - (*iter)->ts;
        float expVal = std::exp(-dt / decayFactor);
        expVal = expVal * 255.0;
        TimeSurface::fixed_Ts((*iter)->y, (*iter)->x) = (*iter)->ts;

        if((*iter)->p){
            mTsPositiveMat.at<float>((*iter)->y, (*iter)->x) = expVal;
        }
        else{
            mTsNegativeMat.at<float>((*iter)->y, (*iter)->x) = expVal;
        }
    }
    for(int i = 0; i < 480; i++){
        for(int j = 0; j < 640; j++) {
            double dt = stamp - fixed_Ts(i,j);
            if (fixed_Ts(i,j) != 0)
                std::cout << dt << std::endl;
            float expVal = std::exp(-dt / decayFactor);
            expVal = expVal * 255.0;
            if(expVal > 255)
                expVal = 255;
            mTsMat.at<float>(i, j) = expVal;
        }
    }

    cv::Mat ts = mTsMat.clone();
    ts.convertTo(ts, CV_8UC1);
    cv::cvtColor(ts, ts, cv::COLOR_GRAY2BGR);
    cv::imshow("TSmat", ts);
    eventCam->undistortImage(mTsMat, mTsMat);
    eventCam->undistortImage(mTsPositiveMat, mTsPositiveMat);
    eventCam->undistortImage(mTsNegativeMat, mTsNegativeMat);
    createMatrix();
}


TimeSurface::TimeSurface(const cv::Mat& eventsHistory, const cv::Mat& PosEventsHistory, const cv::Mat& NegEventsHistory,
                         double stamp, double decayFactor, EventCamera::Ptr eventCam):
mStamp(stamp), mDecayFactor(decayFactor), mTsMat(eventCam->getHeight(), eventCam->getWidth(), CV_64F, 0.0),
mTsPositiveMat(eventCam->getHeight(), eventCam->getWidth(), CV_64F, 0.0),
mTsNegativeMat(eventCam->getHeight(), eventCam->getWidth(), CV_64F, 0.0)
{
    mTsMat = (eventsHistory - stamp) / decayFactor;
    mTsPositiveMat = (PosEventsHistory - stamp) / decayFactor;
    mTsNegativeMat = (NegEventsHistory - stamp) / decayFactor;

    cv::exp(mTsMat, mTsMat);
    cv::exp(mTsPositiveMat, mTsPositiveMat);
    cv::exp(mTsNegativeMat, mTsNegativeMat);

    mTsMat = mTsMat * 255.0;
    mTsPositiveMat = mTsPositiveMat * 255.0;
    mTsNegativeMat = mTsNegativeMat * 255.0;

    eventCam->undistortImage(mTsMat, mTsMat);
    eventCam->undistortImage(mTsPositiveMat, mTsPositiveMat);
    eventCam->undistortImage(mTsNegativeMat, mTsNegativeMat);
//    std::cout << "1111\n";
    createMatrix();

}

void TimeSurface::createMatrix()
{
    GenerateAnnf(mTsMat, mTsAnnfRowMat, mTsAnnfColMat);
    GenerateAnnf(mTsPositiveMat, mTsAnnfPositiveRowMat, mTsAnnfPositiveColMat);
    GenerateAnnf(mTsNegativeMat, mTsAnnfNegativeRowMat, mTsAnnfNegativeColMat);

    mTsMat.convertTo(mTsMat, CV_8U);
    mTsPositiveMat.convertTo(mTsPositiveMat, CV_8U);
    mTsNegativeMat.convertTo(mTsNegativeMat, CV_8U);
    GenerateInverseTS(mTsMat, mInverseTsMat);
    GenerateInverseTS(mTsPositiveMat, mInverseTsPositiveMat);
    GenerateInverseTS(mTsNegativeMat, mInverseTsNegativeMat);

    GenerateTsGrad(mInverseTsMat, mInverseTsGradMatX, mInverseTsGradMatY);
    GenerateTsGrad(mInverseTsPositiveMat, mInverseTsPositiveGradMatX, mInverseTsPositiveGradMatY);
    GenerateTsGrad(mInverseTsNegativeMat, mInverseTsNegativeGradMatX, mInverseTsNegativeGradMatY);

    cv::cv2eigen(mInverseTsMat, mInverseTsMatE);
    cv::cv2eigen(mInverseTsPositiveMat, mInverseTsPositiveMatE);
    cv::cv2eigen(mInverseTsNegativeMat, mInverseTsNegativeMatE);

    cv::cv2eigen(mInverseTsGradMatX, mInverseTsGradMatXE);
    cv::cv2eigen(mInverseTsPositiveGradMatX, mInverseTsPositiveGradMatXE);
    cv::cv2eigen(mInverseTsNegativeGradMatX, mInverseTsNegativeGradMatXE);

    cv::cv2eigen(mInverseTsGradMatY, mInverseTsGradMatYE);
    cv::cv2eigen(mInverseTsPositiveGradMatY, mInverseTsPositiveGradMatYE);
    cv::cv2eigen(mInverseTsNegativeGradMatY, mInverseTsNegativeGradMatYE);
}

void TimeSurface::GenerateAnnf(const cv::Mat &Ts, cv::Mat &annfRow, cv::Mat &annfCol, int radius)
{
    int row = Ts.rows;
    int col = Ts.cols;

    annfRow = cv::Mat(row, col, CV_64F, -1.0);
    annfCol = cv::Mat(row, col, CV_64F, -1.0);
    cv::Mat DistanceField(row, col, CV_64F, -1.0);

    for(int i = 0; i < row; i++){
        for(int j = 0; j < col; j++){
            if(Ts.at<double>(i, j) > 15.0){
                int startRow = i - radius;
                int startCol = j - radius;

                int endRow = i + radius;
                int endCol = j + radius;

                for(int r = startRow; r <= endRow; r++){
                    for(int c = startCol; c <= endCol; c++){
                        if(r < 0 || c < 0 || r >= row || c >= col){
                            continue;
                        }
                        int d = sqrt((r - i) * (r - i) + (c - j) * (c - j));
                        if(DistanceField.at<double>(r, c) == -1 || d < DistanceField.at<double>(r, c)){
                            DistanceField.at<double>(r, c) = d;
                            annfRow.at<double>(r, c) = i;
                            annfCol.at<double>(r, c)= j;
                        }
                    }
                }
            }
        }
    }

}

void TimeSurface::GenerateTsGrad(const cv::Mat &Ts, cv::Mat &gradX, cv::Mat& gradY)
{
    cv::Sobel(Ts, gradX, CV_64F, 1, 0);
    cv::Sobel(Ts, gradY, CV_64F, 0, 1);
}

void TimeSurface::GenerateInverseTS(const cv::Mat& Ts, cv::Mat& invTs, size_t kernelSize)
{
    if(kernelSize > 0)
    {
        cv::Mat TsBlur;
        cv::GaussianBlur(Ts, TsBlur, cv::Size(kernelSize, kernelSize), 0.0);
        invTs = 255.0 - TsBlur;
    }
    else
    {
        invTs = 255.0 - Ts;
    }
}


//------------- get protected variables---------------

cv::Mat& TimeSurface::getTs()
{
    return mTsMat;
}

cv::Mat& TimeSurface::getTsPositiveMat()
{
    return mTsPositiveMat;
}

cv::Mat& TimeSurface::getTsNegativeMat()
{
    return mTsNegativeMat;
}

cv::Mat& TimeSurface::getTsAnnfCol()
{
    return mTsAnnfColMat;
}

cv::Mat& TimeSurface::getTsAnnfPositiveCol()
{
    return mTsAnnfPositiveColMat;
}

cv::Mat& TimeSurface::getTsAnnfNegativeCol()
{
    return mTsAnnfNegativeColMat;
}

cv::Mat& TimeSurface::getTsAnnfRow()
{
    return mTsAnnfRowMat;
}

cv::Mat& TimeSurface::getTsAnnfPositiveRow()
{
    return mTsAnnfPositiveRowMat;
}

cv::Mat& TimeSurface::getTsAnnfNegativeRow()
{
    return mTsAnnfNegativeRowMat;
}

cv::Mat& TimeSurface::getInverseTs()
{
    return mInverseTsMat;
}

cv::Mat& TimeSurface::getInverseTsPositive()
{
    return mInverseTsPositiveMat;
}

cv::Mat& TimeSurface::getInverseTsNegative()
{
    return mInverseTsNegativeMat;
}

cv::Mat& TimeSurface::getInverseTsGradX()
{
    return mInverseTsGradMatX;
}

cv::Mat& TimeSurface::getInverseTsPositiveGradX()
{
    return mInverseTsPositiveGradMatX;
}

cv::Mat& TimeSurface::getInverseTsNegativeGradX()
{
    return mInverseTsNegativeGradMatX;
}

cv::Mat& TimeSurface::getInverseTsGradY()
{
    return mInverseTsGradMatY;
}

cv::Mat& TimeSurface::getInverseTsPositiveGradY()
{
    return mInverseTsPositiveGradMatY;
}

cv::Mat& TimeSurface::getInverseTsNegativeGradY()
{
    return mInverseTsNegativeGradMatY;
}
