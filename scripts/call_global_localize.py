#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty

if __name__ == '__main__':
    srv = rospy.ServiceProxy('global_localization', Empty)
    rospy.wait_for_service('global_localization')
    srv()
