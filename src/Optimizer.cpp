//
// Created by mpl on 23-3-15.
//

#include "Optimizer.h"
#include "LogMacro.h"
#include <opencv2/core/eigen.hpp>
#include <random>
#include <algorithm>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace CannyEVT;

Optimizer::Optimizer(std::string configPath, EventCamera::Ptr eventCam):
mEventCam(eventCam), mEventProblemConfig(new EventProblemConfig(1, 1, 5, "Huber", 50.0))

{
    mEventProblem = std::make_shared<EventProblemLM>(mEventProblemConfig, mEventCam);

    cv::FileStorage fs(configPath, cv::FileStorage::READ );
    if (!fs.isOpened())
        ALERT("[Optimizer]:", "config not open");

    mPatchSizeX = fs["patchSizeX"];
    if (fs["patchSizeX"].isNone())
        WARNING("[Optimizer]", "config[patchSizeX] not set!");

    mPatchSizeY = fs["patchSizeY"];
    if (fs["patchSizeY"].isNone())
        WARNING("[Optimizer]", "config[patchSizeY] not set!");
    mPointCulling = fs["PointCulling"];
    if (fs["PointCulling"].isNone())
        WARNING("[PointCulling]", "config[PointCulling] not set!");
    if(mPointCulling == 1) mEventProblemConfig->usingPointCulling = true;
    else mEventProblemConfig->usingPointCulling = false;

    mNormalFlow = fs["NormalFlow"];
    if (fs["NormalFlow"].isNone())
        WARNING("[NormalFlow]", "config[NormalFlow] not set!");
    if(mNormalFlow == 1) mEventProblemConfig->usingNormalFlow = true;
    else mEventProblemConfig->usingNormalFlow = false;

    mVisualize = fs["visulization"];
    if (fs["visulization"].isNone())
        WARNING("[visulization]", "config[visulization] not set!");

    mzMin = fs["zMin"];
    if (fs["zMin"].isNone())
        WARNING("[zMin]", "config[zMin] not set!");

    mzMax = fs["zMax"];
    if (fs["zMax"].isNone())
        WARNING("[zMax]", "config[zMax] not set!");

}


bool Optimizer::OptimizeEventProblem(TimeSurface::Ptr ts, pCloud cloud, const Eigen::Matrix4d& Tinit,
                                     const Eigen::Matrix4d& Twl, Eigen::Matrix4d& result)
{
    Eigen::Matrix4d Twc_init = Tinit;
    Eigen::Matrix4d Twc_last = Twl;
    mEventProblem->setProblem(ts, cloud, Twc_init, Twc_last);

    Eigen::LevenbergMarquardt<EventProblemLM, double> lm(*mEventProblem);
    lm.resetParameters();
    lm.parameters.ftol = 1e-3;
    lm.parameters.xtol = 1e-3;
    lm.parameters.maxfev = mEventProblemConfig->MAX_ITERATION_ * 8; 

    size_t iteration = 0;
    size_t nfev = 0;
    while(true)
    {
        if (iteration >= mEventProblemConfig->MAX_ITERATION_)
        {
            break;
        }

        mEventProblem->setStochasticSampling((iteration % mEventProblem->mNumBatches) * mEventProblemConfig->BATCH_SIZE_,
                                             mEventProblemConfig->BATCH_SIZE_);

        Eigen::VectorXd x(6);
        x.fill(0.0);
        if (lm.minimizeInit(x) == Eigen::LevenbergMarquardtSpace::ImproperInputParameters) {
            std::cout << "ImproperInputParameters for LM (Tracking)." << std::endl;
            return false;
        }
        Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeOneStep(x);
        mEventProblem->addMotionUpdate(x);

        iteration++;
        nfev += lm.nfev;

        if(mVisualize == 1){

            size_t width, height; 
            cv::Mat reprojMap = cv::Mat(cv::Size(width, height), CV_8UC1, cv::Scalar(0));
            cv::Mat TS = ts->getTs();
            TS.convertTo(reprojMap, CV_8UC1);
            cv::cvtColor(reprojMap, reprojMap, CV_GRAY2BGR);//COLOR_GARY2BGR for opencv 3 and 4

            Eigen::Matrix3d Rcw = mEventProblem -> mRwc.transpose();
            Eigen::Vector3d tcw = -Rcw * mEventProblem -> mtwc;

            size_t numVisualization = std::min(mEventProblem -> mResItems.size(), (size_t)2000);
            for( size_t i = 0; i<numVisualization; i++){
                ResidualItem &ri = mEventProblem ->mResItems[i];
                Eigen::Vector3d p3d = Rcw * ri.p_ + tcw;//this is a 3d point in camera coordinat
                Eigen::Vector2d p2d = mEventCam -> World2Cam(p3d);//this is a 2D point in camera coordinate. result of world2cam(p3d)
           
                double z = p3d(2);

                Utility::DrawPoint(1.0 / z, 1.0 / mzMin, 1.0 / mzMax, Eigen::Vector2d(p2d(0), p2d(1)), reprojMap);
            }
            cv::imshow("reprojMap", reprojMap);
            cv::waitKey(10);
   
        }

        if (status == Eigen::LevenbergMarquardtSpace::Status::RelativeErrorTooSmall ||
            status == Eigen::LevenbergMarquardtSpace::Status::RelativeErrorAndReductionTooSmall)
            break;
    }

    result = mEventProblem->getPose();

    return true;
}



void Optimizer::SamplePointCloud(const std::vector<Point::Ptr> &src, std::vector<Point::Ptr> &dest, int sampleN)
{
    if (src.size() <= sampleN)
    {
        dest.resize(src.size());
        std::copy(src.begin(), src.end(), dest.begin());
    }
    else
    {
        size_t N = src.size();
        std::vector<size_t> indices;
        indices.resize(N);
        for (size_t i = 0; i < N; i++)
            indices[i] = i;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(indices.begin(), indices.end(), gen);


        for (size_t i = 0; i < sampleN; i++)
        {
            int sample_i = indices[i];

            dest.push_back(src[sample_i]);
        }
    }
}



