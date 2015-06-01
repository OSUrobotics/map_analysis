#!/usr/bin/env python
from map_analysis.math import ssd_mat as ssd
from map_analysis.bag_utils import extract_clusters
import rosbag
import rospy
import numpy as np
import os
import yaml


N_BINS = 100
LOWER_CUTOFF_BIN = 6


# note: in indigo, this can be replaced with bag.get_message_count
def get_n_messages(topic, info):
    return next(t['messages'] for t in info['topics'] if t['topic'] == topic)


def find_clusters(l, dist=0.5):
    sorted_l = np.sort(l)
    clusters = [[sorted_l[0], sorted_l[0]]]
    last = sorted_l[0]
    for t in sorted_l[1:]:
        # if t is within dist of the current cluster,
        # extend the cluster's upper bound
        if t - last < dist:
            clusters[-1][-1] = t
        # otherwise, create a new zero-length cluster
        else:
            clusters.append([t, t])
        last = t
    return clusters


if __name__ == '__main__':
    argv = rospy.myargv()[1:]
    topics = [
        '/wheelchair_lasers/left',
        '/wheelchair_lasers/right',
    ]
    bags = {}
    for bagfile in argv:
        print bagfile
        bag = rosbag.Bag(bagfile, 'r')
        bag_info = yaml.load(bag._get_yaml_info())

        n_messages_left = get_n_messages('/wheelchair_lasers/left', bag_info)
        n_messages_right = get_n_messages('/wheelchair_lasers/right', bag_info)

        # preallocate the numpy arrays
        left = np.zeros((n_messages_left, 512))
        right = np.zeros((n_messages_right, 512))

        l_stamps = []
        r_stamps = []
        ind_l = 0
        ind_r = 0
        for topic, msg, stamp in bag.read_messages(topics=topics):
            if topic.endswith('left'):
                left[ind_l, :] = msg.ranges
                l_stamps.append(stamp.to_sec())
                ind_l += 1
            elif topic.endswith('right'):
                right[ind_r, :] = msg.ranges
                r_stamps.append(stamp.to_sec())
                ind_r += 1

        bags[os.path.split(bagfile)[-1]] = dict(
            left=ssd(np.ma.asarray(left)),
            right=ssd(np.ma.asarray(right))
        )

        ssd_left = bags[os.path.split(bagfile)[-1]]['left']
        ssd_right = bags[os.path.split(bagfile)[-1]]['right']
        # l_counts, l_edges = np.histogram(ssd_left, bins=N_BINS)
        # r_counts, r_edges = np.histogram(ssd_right, bins=N_BINS)

        # print l_edges[LOWER_CUTOFF_BIN], r_edges[LOWER_CUTOFF_BIN]

        # l_indices_to_keep = ssd_left > l_edges[LOWER_CUTOFF_BIN]
        # r_indices_to_keep = ssd_right > r_edges[LOWER_CUTOFF_BIN]
        l_indices_to_keep = ssd_left > 35
        r_indices_to_keep = ssd_right > 35

        l_stamps = np.array(l_stamps)[l_indices_to_keep.nonzero()[0]]
        r_stamps = np.array(r_stamps)[r_indices_to_keep.nonzero()[0]]

        if l_indices_to_keep.sum() + r_indices_to_keep.sum() > 0:
            clusters = find_clusters(np.concatenate((l_stamps, r_stamps)), dist=0.5)

            left_keep_scans = left[l_indices_to_keep]
            right_keep_scans = right[r_indices_to_keep]

            path, filename = os.path.split(bagfile)
            bag_out = rosbag.Bag(os.path.join(path, 'compressed_'+filename), 'w', compression=rosbag.Compression.BZ2)

            extract_clusters(clusters, topics, bag, bag_out)

            # for cluster in clusters:
            #     bag_iterator = bag.read_messages(
            #         topics=topics,
            #         start_time=rospy.Time(cluster[0]),
            #         end_time=rospy.Time(cluster[1])
            #     )
            #     for topic, msg, stamp in bag_iterator:
            #         bag_out.write(topic, msg, stamp)
            bag_out.close()
            print 'wrote ', bag_out._filename
        else:
            print 'no movement - skipping bag'