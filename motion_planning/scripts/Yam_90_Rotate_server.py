#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import tf
from motion_planning.srv import yamRotate
import numpy as np
import math as m


def yamRotatefunc(theta):
     return np.matrix([[ m.cos(theta), -m.sin(theta), 0 ],
                   [ m.sin(theta), m.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])



def handle_yam_90_rotate(req):
    theta = -m.pi/2
 
    #Transfer quaternion to euler
    quaternionOrig = (req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w)
    
    #quaternionOrig = (x, y,z, w)
    eulerOrig = tf.transformations.euler_from_quaternion(quaternionOrig)
    
    #Rotate orientation of target points around yaw 90 degress counter clockwise with euler coodinator
    eulerRotated = np.dot(eulerOrig, yamRotatefunc(theta)) 

    #Transfer euler to quaternion
    eulerRotated = eulerRotated.reshape(3,1)
    roll = float(eulerRotated[0])
    pitch = float(eulerRotated[1])
    yam = float(eulerRotated[2])
    quaternionRotated = tf.transformations.quaternion_from_euler(roll, pitch, yam)


    rotatedPose = Pose()
    rotatedPose.position.x = req.pose.position.x
    rotatedPose.position.y = req.pose.position.y
    rotatedPose.position.z = req.pose.position.z
    rotatedPose.orientation.x = quaternionRotated[0]
    rotatedPose.orientation.y = quaternionRotated[1]
    rotatedPose.orientation.z = quaternionRotated[2]
    rotatedPose.orientation.w = quaternionRotated[3]
    
    return(rotatedPose)

def yam_90_rotate_server():
  # Create a ROS node.
  rospy.init_node('target_points_yam_90_rotate_server')
  # Create a service
  s = rospy.Service('target_points_yam_90_rotate', yamRotate, handle_yam_90_rotate)
  # spin service   
  print "Ready to rotate!"
  rospy.spin()



if __name__== '__main__':
  yam_90_rotate_server()
    
