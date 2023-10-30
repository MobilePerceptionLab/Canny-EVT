//
// Created by mpl on 23-4-5.
//

#ifndef CannyEVT_TYPE_H
#define CannyEVT_TYPE_H

#include <memory>
#include <vector>
#include <Eigen/Core>

namespace CannyEVT
{

struct ImuMsg
{
    typedef std::shared_ptr<ImuMsg> Ptr;
    typedef std::shared_ptr<ImuMsg const> ConstPtr;

    ImuMsg(const double ts, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr):ts(ts), acc(acc), gyr(gyr)
    {}

    double ts;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
};

struct EventMsg
{
    typedef std::shared_ptr<EventMsg> Ptr;
    typedef std::shared_ptr<EventMsg const> ConstPtr;

    EventMsg(double ts, size_t x, size_t y, bool p):ts(ts), x(x), y(y), p(p)
    {}

    double ts;
    size_t x, y;
    bool p;
};


struct FrameData
{
    FrameData(std::vector<EventMsg::ConstPtr> event,  double ts):
            eventData(std::move(event)), stamp(std::move(ts))
    {}

    std::vector<EventMsg::ConstPtr> eventData;
    double stamp;
};


struct Point {

    typedef std::shared_ptr<Point> Ptr;
    typedef std::shared_ptr<Point const> ConstPtr;

    Point(double x, double y, double z, double xg, double yg, double zg):
            x(x), y(y), z(z), xg(xg), yg(yg), zg(zg)
    {};

    double x;
    double y;
    double z;

    double xg;//3D gradient
    double yg;
    double zg;
};

typedef std::shared_ptr<std::vector<Point>> pCloud; 

}

#endif //CannyEVT_TYPE_H
