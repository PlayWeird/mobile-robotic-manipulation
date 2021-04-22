#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from motion_planning.srv import yamRotate
import sys




def yam_90_rotate_client(pose):
    rospy.wait_for_service('target_points_yam_90_rotate')
    try:
        target_points_yam_90_rotate = rospy.ServiceProxy('target_points_yam_90_rotate', yamRotate)
        resp1 = target_points_yam_90_rotate(pose)
        return resp1.rotatedPose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
  
  made_pose = Pose()
  made_pose.position.x = 2.0
  made_pose.position.y = 1.5
  made_pose.position.z = 0.0
  made_pose.orientation.x = 0.0
  made_pose.orientation.y = 2.0
  made_pose.orientation.z = 1.0
  made_pose.orientation.w = 1.0

  print(yam_90_rotate_client(made_pose))
