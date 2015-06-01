#!/usr/bin/env python

from map_analysis.math import distanceBetweenCurves
from nav_msgs.msg import Odometry
import rospy
import yaml
import os

rospy.init_node('hausdorff_odom')
yamlpath = rospy.myargv()[1]
yamlpieces = yamlpath.split(os.sep)
resultpath = '/'+os.path.join(*(yamlpieces[:-2] + ['laser_opt_results'] + [yamlpieces[-1]+'.result']))
print 'writing result to', resultpath
gt = []
est = []

while not rospy.is_shutdown():
    # ground_truth, estimate = topics.odom_temp[0], topics.laser_pose[0]
    try:
        ground_truth = rospy.wait_for_message('/odom_temp', Odometry)
        estimate = rospy.wait_for_message('/laser_pose', Odometry)
        pos_gt = ground_truth.pose.pose.position
        pos_est = estimate.pose.pose.position
        gt.append([pos_gt.x, pos_gt.y])
        est.append([pos_est.x, pos_est.y])

        dist = float(distanceBetweenCurves(gt, est))
        print dist
        try:
            with open(resultpath, 'w') as f:
                f.writelines([str(dist), yamlpath])
                f.write('\n')
        except IOError:
            continue
    except rospy.exceptions.ROSInterruptException:
        break

