//
// Created by mpl on 23-4-3.
//

#ifndef CannyEVT_CamBasedProblemLM_H
#define CannyEVT_CamBasedProblemLM_H

#include "GenericFunctor.h"
#include "EventCamera.h"
#include "TimeSurface.h"
#include "type.h"

namespace CannyEVT
{

struct CamBasedProblemConfig {
    typedef std::shared_ptr<CamBasedProblemConfig> Ptr;

    CamBasedProblemConfig(size_t patchSize_X, size_t patchSize_Y, size_t kernelSize, const std::string &LSnorm,
                       double huber_threshold, double invDepth_min_range = 0.2, double invDepth_max_range = 2.0,
                       const size_t MIN_NUM_EVENTS = 1000, const size_t MAX_REGISTRATION_POINTS = 3000,
                       const size_t BATCH_SIZE = 200, const size_t MAX_ITERATION = 10, const size_t NUM_THREAD = 4)
            :patchSize_X_(patchSize_X), patchSize_Y_(patchSize_Y), kernelSize_(kernelSize), LSnorm_(LSnorm),
            huber_threshold_(huber_threshold),invDepth_min_range_(invDepth_min_range), invDepth_max_range_(invDepth_max_range),
            MIN_NUM_EVENTS_(MIN_NUM_EVENTS), MAX_REGISTRATION_POINTS_(MAX_REGISTRATION_POINTS), BATCH_SIZE_(BATCH_SIZE),
            MAX_ITERATION_(MAX_ITERATION), StochasticSampling(true), NUM_THREAD(NUM_THREAD), debug(false)
              {}

    size_t patchSize_X_, patchSize_Y_;
    size_t kernelSize_;
    std::string LSnorm_;
    double huber_threshold_;
    double invDepth_min_range_;
    double invDepth_max_range_;
    size_t MIN_NUM_EVENTS_;
    size_t MAX_REGISTRATION_POINTS_;
    size_t BATCH_SIZE_;
    size_t MAX_ITERATION_;
    bool StochasticSampling;
    size_t NUM_THREAD;
    bool debug;
    bool usingPointCulling;
    bool usingNormalFlow;

};


class CamBasedProblemLM: public GenericFunctor<double>
{
public:
    typedef std::shared_ptr<CamBasedProblemLM> Ptr;

    struct Job {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ResidualItems *pvRI;
        TimeSurface::ConstPtr pTs;
        const Eigen::Matrix4d *Tcw;
        size_t threadID;
    };

public:
    CamBasedProblemLM(CamBasedProblemConfig::Ptr config, EventCamera::Ptr EventCam);
    ~CamBasedProblemLM() = default;

    void setConfig(CamBasedProblemConfig::Ptr config);
    void setProblem(TimeSurface::Ptr Ts, pCloud cloud, Eigen::Matrix4d Twc, Eigen::Matrix4d Twl);
    Eigen::Matrix4d getPose();
    void setStochasticSampling(size_t offset, size_t N);
    int operator()(const Eigen::Matrix<double, 6, 1> &x, Eigen::VectorXd &fvec) const;
    int df(const Eigen::Matrix<double, 6, 1> &x, Eigen::MatrixXd &fjac) const;
    void addMotionUpdate(const Eigen::Matrix<double, 6, 1> &dx);

    bool patchInterpolation(const Eigen::MatrixXd &img, const Eigen::Vector2d &location,
                            Eigen::MatrixXd &patch, bool debug) const;

    void thread(Job &job) const;
    void computeJ_G(const Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 12, 6> &J_G);
    bool isValidPatch(Eigen::Vector2d &patchCentreCoord, Eigen::MatrixXi &mask, size_t wx, size_t wy) const;
    bool isValidFlow(const Eigen::Vector2d &flow);
    int predictPolarity(const Eigen::Vector2d& gradFlow, const Eigen::Vector2d& opticalFlow);
    void getWarpingTransformation(Eigen::Matrix4d &warpingTransf, const Eigen::Matrix<double, 6, 1> &x) const;
    void pointCulling(int col, int row, Eigen::MatrixXd& depthMatrix, Eigen::MatrixXd& indexMatrix, double depth, int index);

public:
    EventCamera::Ptr mEventCamera;
    CamBasedProblemConfig::Ptr mConfig;
    TimeSurface::Ptr mTs;
    pCloud mCloud;

    ResidualItems mResItems, mResItemsStochSampled;

    Eigen::Matrix<double, 12, 6> mJ_G_0;

    // event came pose
    Eigen::Matrix3d mRwc;
    Eigen::Vector3d mtwc;

    int mCounter; // times of optimization(not iterations)
    int mPatchSize;
    size_t mNumBatches;
    size_t mNumPoints;
};
}

#endif //CannyEVT_CamBasedProblemLM_H
