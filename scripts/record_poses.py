#!/usr/bin/env python
import datetime
import rosbag
import rospy
import shlex
import yaml
import glob
import sys
import os


def stamp_to_str(stamp):
    return datetime.datetime.fromtimestamp(stamp).strftime('%Y-%m-%d-%H-%M-%S')


def get_start_end_times(bags):
    min_time = float('inf')
    max_time = 0
    for b in bags:
        bag_info = yaml.load(b._get_yaml_info())
        if 'start' in bag_info and 'end' in bag_info:
            start = bag_info['start']
            end = bag_info['end']
            if start < min_time:
                min_time = start
            if end > max_time:
                max_time = end
    return min_time, max_time


def make_bag_name(start, end):
    return '%s_%s' % (stamp_to_str(start), stamp_to_str(end))


def main():
    argv = rospy.myargv()
    bagfiles = sum([glob.glob(f) for f in argv[1:]], [])
    bags = [rosbag.Bag(b, 'r') for b in bagfiles]
    start, end = get_start_end_times(bags)
    bag_name = make_bag_name(start, end)
    out_path = os.path.join(os.path.split(bagfiles[0])[0], bag_name)
    rosbag.rosbag_main.record_cmd(
        shlex.split('rosbag record -O %s /amcl_pose /map /map_metadata' % out_path))

if __name__ == '__main__':
    try:
        main()
    except Exception, ex:
        import ipdb
        ipdb.post_mortem(sys.exc_info()[2])
