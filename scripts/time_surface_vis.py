import cv2
import cv_bridge
import minkindr
import numpy as np
import rosbag
import rospy
import scipy
from dvs_msgs import msg
from dvs_msgs.msg import Event, EventArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import *


def rosTime2Float(t):
    return t.nsecs/1e9+t.secs


class TimeSurfaceComp:
    def __init__(self):
        self.K_ = None
        self.dist_ = None
        self.stamped_poses_ = None
        self.events_ = None
        self.Tss_ = None
        self.h_ = None
        self.w_ = None
        # self.

    def loadData(self, bag_path):
        # load datasets, including depth frames, ground truth poses and all the events
        bag = rosbag.Bag(bag_path, "r")
        # pose_msgs = [m for m in bag.read_messages("/davis/left/pose")]
        cam_info_msgs = [m for m in bag.read_messages(
            "/davis/left/camera_info")]
        cam_info = cam_info_msgs[0].message

        self.K_ = np.array(cam_info.K).reshape([3, 3])
        self.h_ = cam_info.height
        self.w_ = cam_info.width
        self.dist_ = np.array(cam_info.D)
        events_temp = [m.message for m in bag.read_messages(
            "/davis/left/events")]
        # self.stamped_poses_ = [m.message for m in pose_msgs]
        # pack event arrays
        self.events_ = []
        for ea in events_temp:
            for e in ea.events:
                self.events_.append(e)
        bag.close()

    def interpolatePoseStamped(p):
        pass

    def printConfig(self):
        print(self.K_)
        print(self.dist_)
        print(len(self.stamped_poses_))
        print(len(self.events_))
        print(self.events_[0].ts)
        print(self.stamped_poses_[0].header.stamp)

    def createTimeSurfaceStamped(self, interval_s=0.04):
        self.Tss_ = []
        t_init = rosTime2Float(self.events_[0].ts)
        t_start = t_init
        decay_factor = 3
        Ts_temp = np.zeros([self.h_, self.w_], dtype=np.float32)
        for e in self.events_:
            t_cur = rosTime2Float(e.ts)
            if t_cur > t_start+interval_s:
                # update new frame
                t_start += interval_s
                Ts_vis = (Ts_temp*255).astype(np.uint8)
                Ts_vis_median = cv2.bilateralFilter(Ts_vis, 2, 20, 10)

                cv2.imshow("Ts", np.hstack(
                    (Ts_vis, Ts_vis_median)))
                cv2.waitKey(0)
                Ts_temp = np.zeros([self.h_, self.w_], dtype=np.float32)
            elif t_cur > t_start:
                # update current frame
                x, y = e.x, e.y
                t_decay = interval_s-(t_cur-t_start)
                v_decay = np.exp(-t_decay/decay_factor)
                Ts_temp[y, x] = v_decay
        cv2.destroyAllWindows()


if __name__ == "__main__":
    tsc = TimeSurfaceComp()
    tsc.loadData("/home/mpl/Downloads/indoor_flying2_data.bag")
    tsc.createTimeSurfaceStamped()
    tsc.printConfig()
