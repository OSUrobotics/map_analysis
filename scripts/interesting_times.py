#!/usr/bin/env python
from map_analysis.math import ssd_mat as ssd
from map_analysis.bag_utils import extract_clusters
import rosbag
import rospy
import numpy as np
import os
import yaml

from sklearn.cluster import MeanShift


N_BINS = 100
LOWER_CUTOFF_BIN = 6

ACCEL_THRESH = 0.1
ANG_VEL_THRESH = 0.05

IMU_THRESH = (ACCEL_THRESH, ACCEL_THRESH, ACCEL_THRESH, ANG_VEL_THRESH)

# note: in indigo, this can be replaced with bag.get_message_count
def get_n_messages(topic, info):
    try:
        return next(t['messages'] for t in info['topics'] if t['topic'] == topic)
    except StopIteration:
        return 0

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

def merge_clusters(l, dist=3.00):
    merged = []
    last_len = len(l)
    n_clusters = len(l)
    while len(merged) != last_len:
        last_interval = l[0]
        merged = [l[1]]
        for start, end in l[1:]:
            # if we are close enough to the last interval, update it
            if start - dist <= last_interval[1]:
                last_interval[1] = end
            # otherwise create a new interval
            else:
                last_interval = [start, end]
                merged.append(last_interval)
            last_end = end
        last_len = len(l)
        l = merged[:]
    return l


if __name__ == '__main__':
    argv = rospy.myargv()[1:]
    topics = [
        '/wheelchair_lasers/left',
        '/wheelchair_lasers/right',
        '/wifi_info',
        '/imu/data'
    ]
    bags = {}
    for bagfile in argv:
        print bagfile
        bag = rosbag.Bag(bagfile, 'r')
        bag_info = yaml.load(bag._get_yaml_info())

        n_messages_left = get_n_messages('/wheelchair_lasers/left', bag_info)
        n_messages_right = get_n_messages('/wheelchair_lasers/right', bag_info)
        n_messages_wifi = get_n_messages('/wifi_info', bag_info)
        n_messages_imu = get_n_messages('/imu/data', bag_info)

        n_pts = next(len(msg.ranges) for (topic, msg, stamp) in bag.read_messages(topics=['/wheelchair_lasers/right']))

        # preallocate the numpy arrays
        left = np.zeros((n_messages_left, n_pts))
        right = np.zeros((n_messages_right, n_pts))
        wifi_left = np.zeros(n_messages_left, dtype=bool)
        wifi_right = np.zeros(n_messages_right, dtype=bool)
        imu_left = np.zeros((n_messages_left, 4))
        imu_right = np.zeros((n_messages_right, 4))
        # wifi_status = np.zeros((n_messages_wifi, 2))

        l_stamps = []
        r_stamps = []
        ind_l = 0
        ind_r = 0
        ind_w = 0

        last_wifi_status = True
        last_imu = (0,0,0)

        print '\tReading data'

        for topic, msg, stamp in bag.read_messages(topics=topics):
            if topic.endswith('left'):
                left[ind_l, :] = msg.ranges
                imu_left[ind_l, :] = last_imu
                l_stamps.append(stamp.to_sec())
                wifi_left[ind_l] = last_wifi_status
                ind_l += 1
            elif topic.endswith('right'):
                right[ind_r, :] = msg.ranges
                imu_right[ind_r, :] = last_imu
                r_stamps.append(stamp.to_sec())
                wifi_right[ind_r] = last_wifi_status
                ind_r += 1
            elif topic.endswith('wifi_info'):
                last_wifi_status = len(msg.essid) > 0
            elif 'imu' in topic:
                last_imu = (msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z,
                            msg.angular_velocity.z,
                            )

        print '\tFinding indices'

        bags[os.path.split(bagfile)[-1]] = dict(
            left=ssd(np.ma.asarray(left)),
            right=ssd(np.ma.asarray(right))
        )

        ssd_left = bags[os.path.split(bagfile)[-1]]['left']
        ssd_right = bags[os.path.split(bagfile)[-1]]['right']

        # differentiate the accelerations to get rid of any DC bias
        # leave ang velocity alone since the gyros seem less 
        # susceptible to bias)
        imu_left[1:, :3] = np.diff(imu_left[:, :3], axis=0)
        imu_right[1:, :3] = np.diff(imu_right[:, :3], axis=0)

        l_imu_indices_to_keep = (np.abs(imu_left) > IMU_THRESH).any(axis=1)
        r_imu_indices_to_keep = (np.abs(imu_right) > IMU_THRESH).any(axis=1)

        l_indices_to_keep = (ssd_left > 35) & wifi_left[:-1] & l_imu_indices_to_keep[:-1]
        r_indices_to_keep = (ssd_right > 35) & wifi_right[:-1] & r_imu_indices_to_keep[:-1]


        l_stamps = np.array(l_stamps)[l_indices_to_keep.nonzero()[0]]
        r_stamps = np.array(r_stamps)[r_indices_to_keep.nonzero()[0]]

        if l_indices_to_keep.sum() + r_indices_to_keep.sum() > 0:
            print '\tFinding clusters'
            clusters = find_clusters(np.concatenate((l_stamps, r_stamps)), dist=0.5)
            non_tiny_clusters = np.asarray(clusters)[(np.diff(clusters) > 0.75).squeeze(),:]

            ######################### Do Mean shift #########################
            # X = np.atleast_2d(np.asarray(non_tiny_clusters).flatten()).T
            # ms = MeanShift(bin_seeding=True)
            # ms.fit(X)

            # labels = ms.labels_
            # cluster_centers = ms.cluster_centers_

            # labels_unique = np.unique(labels)
            # n_clusters_ = len(labels_unique)

            # ms_clusters = []
            # for k in range(n_clusters_):
            #     cl = X[labels == k]
            #     ms_clusters.append((cl.min(), cl.max()))
            ######################### /Do Mean shift #########################

            # left_keep_scans = left
            # if left.shape[0]:
            #     left_keep_scans = left[l_indices_to_keep]

            # right_keep_scans = right
            # if right.shape[0]:
            #     right_keep_scans = right[r_indices_to_keep]

            path, filename = os.path.split(bagfile)
            # bag_out = rosbag.Bag(os.path.join(path, 'compressed_'+filename), 'w', compression=rosbag.Compression.BZ2)
            bag.close()

            print '\tWriting new bag'
            # extract_clusters(clusters, topics, bag, bag_out)
            out_path = os.path.join(path, 'compressed_'+filename)
            # extract_clusters(ms_clusters, topics, bagfile, out_path)
            # extract_clusters(clusters, topics, bagfile, out_path)
            extract_clusters(merge_clusters(non_tiny_clusters), topics, bagfile, out_path)

            # bag_out.close()
            # print 'wrote ', bag_out._filename
            print 'wrote ', out_path
        else:
            print 'no movement - skipping bag'
