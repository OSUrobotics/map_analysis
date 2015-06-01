#!/usr/bin/env python
import rospy
import tf
import numpy as np
from angles import normalize
from geometry_msgs.msg import TwistStamped
import PyKDL

dims = 3
dt = 0.1
xk_1 = np.zeros(dims)
vk_1 = np.zeros(dims)
a = 0.85
b = 0.005

xk = np.zeros(dims)
vk = np.zeros(dims)
rk = np.zeros(dims)

if __name__ == '__main__':
    rospy.init_node('alphabeta')
    fixed_frame = rospy.get_param('~fixed_frame')
    base_frame = rospy.get_param('~base_frame')

    twist_pub = rospy.Publisher('vel', TwistStamped)
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform(
        fixed_frame, base_frame, rospy.Time(), rospy.Duration(100))

    rate = rospy.Rate(dt)
    ts = TwistStamped()
    ts.header.frame_id = base_frame
    while not rospy.is_shutdown():
        try:
            linear, angular = tf_listener.lookupTwist(
                base_frame, fixed_frame, rospy.Time(), rospy.Duration(0.2))
            pos, quat = tf_listener.lookupTransform(fixed_frame, base_frame, rospy.Time())
            rot = PyKDL.Rotation.Quaternion(*quat).GetRPY()
        except tf.ExtrapolationException:
            continue
        except tf.Exception:
            continue
        xm = np.array([pos[0], pos[1], rot[2]])
        xk = xk_1 + (vk_1 * dt)
        vk = vk_1

        rk = xm - xk

        xk += a * rk
        vk += (b * rk) / dt

        # vk[2] = normalize(vk[2], -np.pi, np.pi)

        xk_1 = xk
        vk_1 = vk

        ts.header.stamp = rospy.Time.now()
        ts.twist.linear.x = vk[0]
        ts.twist.linear.y = vk[1]
        ts.twist.angular.z = vk[2]

        # ts.twist.linear.x = linear[0]
        # ts.twist.linear.y = linear[1]
        # ts.twist.angular.z = linear[2]


        twist_pub.publish(ts)

        # if abs(linear[0]) > 3:
        #     print vk, pos[0]
    rate.sleep()
