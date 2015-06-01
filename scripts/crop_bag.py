#!/usr/bin/env python

import rosbag
import rospy
from rqt_bag_annotator.utils import annotation_path
from map_analysis.bag_utils import extract_clusters, get_info
import os

def dur_to_startend(stamp, duration):
    return stamp, stamp + duration


def crop_bag(bag_file, annotation_file):
    if not os.path.exists(annotation_file):
        print 'no annotations - skipping'
        return
    annotation_bag = rosbag.Bag(annotation_file, 'r')

    bag = rosbag.Bag(bag_file.replace('compressed_',''), 'r')
    wheelchair_bag_info = get_info(bag)
    wheelchair_topics = [i['topic'] for i in wheelchair_bag_info['topics']]

    annotation_bag_info = get_info(annotation_bag)
    annotation_topics = [i['topic'] for i in annotation_bag_info['topics']]
    if '/bag_interval' in annotation_topics:
        annotation_topics.remove('/bag_interval')

    intervals = []
    for _, msg, time in annotation_bag.read_messages(topics=['/bag_interval']):
        intervals.append(dur_to_startend(time, msg.data))

    out_bag_path = annotation_path(bag_file, 'cropped')
    out_bag = rosbag.Bag(out_bag_path, 'w')

    extract_clusters(intervals, wheelchair_topics, bag, out_bag)

    if annotation_topics:
        for data in annotation_bag.read_messages(topics=annotation_topics):
            out_bag.write(*data)

    out_bag.close()
    print 'wrote', out_bag_path

if __name__ == '__main__':
    bag = rospy.myargv()[1]
    crop_bag(bag, annotation_path(bag))
