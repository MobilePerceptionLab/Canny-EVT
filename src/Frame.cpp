//
// Created by mpl on 23-3-9.
//

#include "Frame.h"
#include "utility.h"

using namespace CannyEVT;

Frame::Frame(TimeSurface::Ptr Ts,  double stamp, EventCamera::Ptr eventCam):
mTs(Ts), mStamp(stamp), mEventCam(eventCam)
{}
