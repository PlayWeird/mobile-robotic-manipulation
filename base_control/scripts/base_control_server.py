#!/usr/bin/env python  
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose
from base_control.srv import *



def handle_base_control(req):
    SUCCEEDED = 0
    PLANNING_ERROR =-1
    UNKNOWN_ERROR =-2
    rospy.loginfo("Base control request received")
    ac = actionlib.SimpleActionClient("/bvr_SIM/move_base", MoveBaseAction)
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")

    goal = MoveBaseGoal()
    
    goal.target_pose.pose.position = req.pose.position
    goal.target_pose.pose.orientation = req.pose.orientation
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    ac.send_goal(goal)
    ac.wait_for_result()
    
    if(ac.get_state() == GoalStatus.SUCCEEDED):
        
        rospy.loginfo("Reached the destination")
        return SUCCEEDED
    elif(ac.get_state() == GoalStatus.REJECTED):
        rospy.logwarn("Cannot plan move-base path")
        return PLANNING_ERROR
    else:
        rospy.logwarn("Unknown move-base error")
        return UNKNOWN_ERROR
    



def base_control_server():
    rospy.init_node("base_control_server", anonymous=True)
    rospy.loginfo(("Initialized BaseControl"))
    s = rospy.Service("base_control", BaseControlSrv, handle_base_control)
    rospy.spin()


if __name__ == '__main__':
    base_control_server()

   