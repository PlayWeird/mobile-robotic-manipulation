#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from base_control.srv import *


def base_control_client(pose1):
    rospy.wait_for_service("base_control")
    try:
        base_control_feedback= rospy.ServiceProxy("base_control", BaseControlSrv)

        resp1 = base_control_feedback(pose1)
        print(resp1.status)
        return resp1.status
    except rospy.ServiceException as e:
        print("Service call failed: %s " %e)

if __name__=="__main__":
    modepose = Pose()
    modepose.position.x = 4.2
    modepose.position.y = 1.2
    modepose.position.z = 0
    modepose.orientation.x = 0
    modepose.orientation.y = 0
    modepose.orientation.z = 0
    modepose.orientation.w = 1
   
    base_control_client(modepose)