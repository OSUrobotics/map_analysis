#!/usr/bin/env python
import rosbag
import rospy
import numpy as np
import yaml
import tf

if __name__ == '__main__':
    bagfile = rospy.myargv()[1]
    bag = rosbag.Bag(bagfile, 'r')
    # info = yaml.load(bag._get_yaml_info())
    # tft = tf.Transformer(True, rospy.Duration(info['duration']))

    bag_out = rosbag.Bag('bag_out.bag', 'w')

    odoms = []
    for topic, msg, t in bag.read_messages():#topics=['/tf']): #, '/odom_temp']):
        # if msg.child_frame_id
            # for msg in msg.transforms:
                # tft.setTransform(msg)
        # elif topic == '/odom_temp':
            # odoms.append(msg)
        if topic == '/tf':
            new_msg = tf.msg.tfMessage()
            for transform in msg.transforms:
                if transform.child_frame_id != 'base_footprint':
                    new_msg.transforms.append(transform)
            msg = new_msg

        bag_out.write(topic, msg, t)