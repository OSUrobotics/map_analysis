#!/usr/bin/env python

from std_srvs.srv import Empty as EmptySrv
from std_msgs.msg import Empty as EmptyMsg
import rospy


def call_service(msg, srv):
    print 'called'
    srv(EmptySrv())

if __name__ == '__main__':
    rospy.init_node('call_global_localization')
    srv = rospy.ServiceProxy('/global_localization', EmptySrv)
    srv.wait_for_service()
    rospy.Subscriber('/global_localization', EmptyMsg, call_service, callback_args=[srv])
    rospy.spin()
