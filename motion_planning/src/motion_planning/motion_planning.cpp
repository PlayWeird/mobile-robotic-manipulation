#include <ros/ros.h>
#include "motion_planning.h"
#include <geomtry_msgs/Pose.h>


MotionPlanning::MotionPlanning(int argc, char **argv) :
  nh_(new ros::NodeHandle()){
  init();
  ROS_INFO("Initalized MotionPlanning");
}





bool MotionPlanning::run() {

}


// Move the robot.
// Input: poses must be expressed in the world/map frame.
// Return  0: control execution successful.
// Return -1: service call failed.
// Return -2: control execution failed.
int MotionPlanning::move(const geometry_msgs::Pose &base_pose, const std::vector<geometry_msgs::Pose> &end_effector_poses) {
  // Prepare service call message
  move_control::MoveControlSrv move_control_srv;
  move_control_srv.request.base_pose = base_pose;
  move_control_srv.request.arm_poses = end_effector_poses;

  // If cannot call service, return error
  if (!control_clt.call(move_control_srv)) return -1;
  // If control execution failed, return error
  if (!move_control_srv.response.status) return -2;

  // Return successful status
  return 0;
}
