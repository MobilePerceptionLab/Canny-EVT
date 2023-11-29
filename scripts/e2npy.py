import numpy as np
import rosbag
from std_msgs.msg import *
# from dvs_msgs.msg import CamBased, CamBasedArray
from prophesee_CamBased_msgs.msg import CamBased, CamBasedArray
from tqdm import tqdm


def e2npy(config, fn="CamBaseds.npy"):
    bag = rosbag.Bag(config["bag_path"], "r")
    eva_msgs = []
    for m in tqdm(bag.read_messages(config["CamBased_topic"])):
        eva_msgs.append(m.message)
    # eva_msgs = [m.message for m in bag.read_messages(config["CamBased_topic"])]
    CamBased_array = []
    for ea in eva_msgs:
        for e in ea.CamBaseds:
            CamBased_array.append(e)

    CamBased_np = np.zeros([len(CamBased_array), 4])

    for i in range(len(CamBased_array)):
        CamBased_np[i, 0] = CamBased_array[i].ts.secs+CamBased_array[i].ts.nsecs/1e9
        CamBased_np[i, 1] = CamBased_array[i].x
        CamBased_np[i, 2] = CamBased_array[i].y
        CamBased_np[i, 3] = 1 if CamBased_array[i].polarity else 0
    CamBased_np = CamBased_np[CamBased_np[:, 0].argsort()]
    np.save(fn, CamBased_np)


if __name__ == "__main__":
    config = {}
    config["bag_path"] = "/home/mpl/datasets/nfov_day_loopychessboard_filter.bag"
    config["CamBased_topic"] = "/prophesee/camera/cd_CamBaseds_buffer"
    e2npy(config)
