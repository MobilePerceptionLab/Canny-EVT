import numpy as np
import rosbag
from std_msgs.msg import *
# from dvs_msgs.msg import Event, EventArray
from prophesee_event_msgs.msg import Event, EventArray
from tqdm import tqdm


def e2npy(config, fn="events.npy"):
    bag = rosbag.Bag(config["bag_path"], "r")
    eva_msgs = []
    for m in tqdm(bag.read_messages(config["event_topic"])):
        eva_msgs.append(m.message)
    # eva_msgs = [m.message for m in bag.read_messages(config["event_topic"])]
    event_array = []
    for ea in eva_msgs:
        for e in ea.events:
            event_array.append(e)

    event_np = np.zeros([len(event_array), 4])

    for i in range(len(event_array)):
        event_np[i, 0] = event_array[i].ts.secs+event_array[i].ts.nsecs/1e9
        event_np[i, 1] = event_array[i].x
        event_np[i, 2] = event_array[i].y
        event_np[i, 3] = 1 if event_array[i].polarity else 0
    event_np = event_np[event_np[:, 0].argsort()]
    np.save(fn, event_np)


if __name__ == "__main__":
    config = {}
    config["bag_path"] = "/home/mpl/datasets/nfov_day_loopychessboard_filter.bag"
    config["event_topic"] = "/prophesee/camera/cd_events_buffer"
    e2npy(config)
