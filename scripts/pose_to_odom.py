#!/usr/bin/env rosh

rostype(topics.laser_pose, msg.nav_msgs.Odometry)

for pose in topics.pose_stamped[:]:
    odom = msg.nav_msgs.Odometry()
    odom.header = pose.header
    odom.child_frame_id = 'base_footprint'
    odom.pose.pose = pose.pose
    topics.laser_pose(odom)