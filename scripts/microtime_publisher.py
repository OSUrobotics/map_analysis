#!/usr/bin/env python

import argparse
import rospy
import genpy
import time
from rosgraph_msgs.msg import Clock
from threading import RLock


class AnySubscriber(object):

    def __init__(self, topics):
        self.subscribers = {}
        self.locks = {}
        for topic in topics:
            self.subscribers[topic] = rospy.Subscriber(
                topic, rospy.AnyMsg, self.temp_cb)
            self.locks[topic] = RLock()

        self.messages = {}
        self.publishers = {}

    def temp_cb(self, msg):
        topic = msg._connection_header['topic']
        with self.locks[topic]:
            msg_cls = genpy.message.get_message_class(
                msg._connection_header['type'])
            self.subscribers[topic].unregister()
            self.subscribers[topic] = rospy.Subscriber(topic, msg_cls, self.cb)
            self.publishers[topic] = rospy.Publisher(topic, msg_cls)

    def cb(self, msg):
        topic = msg._connection_header['topic']
        with self.locks[topic]:
            self.messages[topic] = msg

    def publish_at(self, at_time):
        for topic, pub in self.publishers.iteritems():
            with self.locks[topic]:
                if topic in self.messages:
                    msg = self.messages[topic]
                    if hasattr(msg, 'header'):
                        msg.header.stamp = at_time
                    pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(
        description='Publish topics with tiny time increments')
    parser.add_argument('topics', type=str, nargs='+')

    rospy.init_node('microtime')
    args = parser.parse_args(rospy.myargv()[1:])

    clock_pub = rospy.Publisher('/clock', Clock)

    sub = AnySubscriber(args.topics)
    last_time = rospy.Time.now()
    last_published_time = rospy.Time.now()
    nanosec = rospy.Duration(0.000000001)
    while not rospy.is_shutdown():
        print last_time, rospy.Time.now()
        if last_time == rospy.Time.now():
            print 'asdf'
            last_published_time += nanosec
            clock_pub.publish(Clock(last_published_time))
            sub.publish_at(last_published_time)
        else:
            last_published_time = rospy.Time.now()

        now = rospy.Time.now()
        # if now != last_published_time:
        last_time = now
        time.sleep(0.1)


if __name__ == '__main__':
    main()
