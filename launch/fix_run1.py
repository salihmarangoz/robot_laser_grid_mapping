
# written for "run1.bag"
# this script replaces nan values with inf to comply with REP specifications: https://www.ros.org/reps/rep-0117.html
# run "rosbag compress run1_fixed.bag" after processing

import rosbag
import numpy as np

with rosbag.Bag('run1_fixed.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('run1.bag').read_messages():
        if topic == "/scan":
            new_ranges = []
            for i in range(len(msg.ranges)):
                if np.isnan(msg.ranges[i]):
                    new_ranges.append(np.inf)
                else:
                    new_ranges.append(msg.ranges[i])
            msg.ranges = new_ranges
            outbag.write(topic, msg, t)
        else:
            outbag.write(topic, msg, t)