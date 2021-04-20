#include <ros/ros.h>
#include "motion_planning.h"
#include <geometry_msgs/Pose.h>


MotionPlanning::MotionPlanning(int argc, char **argv) :
  nh_(new ros::NodeHandle()){
  init();
  ROS_INFO("Initalized MotionPlanning");
}

/* ---------Experiment starts here--------- */
geometry_msgs::Pose makeBasePose(){
  geometry_msgs::Pose made_pose;
  float random_x;
  float range = 1.0;
  random_x = ((float)std::rand() / (float)RAND_MAX) * range;

  made_pose.position.x = random_x;
  made_pose.position.y = 1.5;
  made_pose.position.z = 0.0;
  made_pose.orientation.x = 0.0;
  made_pose.orientation.y = 0.0;
  made_pose.orientation.z = 0.0;
  made_pose.orientation.w = 1.0;

  return made_pose;
}


std::vector<geometry_msgs::Pose> makeEndEffectorPoses(const geometry_msgs::Pose &base_pose) {
  geometry_msgs::Pose ee_pose1;
  ee_pose1.position.x = base_pose.position.x + 1.0;
  ee_pose1.position.y = base_pose.position.y - 0.05;
  ee_pose1.position.z = 0.5;
  ee_pose1.orientation.x = base_pose.orientation.x;
  ee_pose1.orientation.y = base_pose.orientation.y;
  ee_pose1.orientation.z = base_pose.orientation.z;
  ee_pose1.orientation.w = base_pose.orientation.w;

  geometry_msgs::Pose ee_pose2;
  ee_pose2.position.x = base_pose.position.x + 0.8;
  ee_pose2.position.y = base_pose.position.y - 0.05;
  ee_pose2.position.z = 0.75;
  ee_pose2.orientation.x = base_pose.orientation.x;
  ee_pose2.orientation.y = base_pose.orientation.y;
  ee_pose2.orientation.z = base_pose.orientation.z;
  ee_pose2.orientation.w = base_pose.orientation.w;

  std::vector<geometry_msgs::Pose> ee_poses;
  ee_poses.push_back(ee_pose2);
  ee_poses.push_back(ee_pose1);

  return ee_poses;
}


BaseEndEffectorPoses getFakeBaseEndEffectorPoses() {
  auto base_pose = makeBasePose();
  return BaseEndEffectorPoses{.base_pose = base_pose, .end_effector_poses = makeEndEffectorPoses(base_pose)};
}
/* ---------Experiment ends here--------- */


bool MotionPlanning::run() {
  bool run_successful = true;

  //Trial 1
  auto base_end_effector_poses_1 = getFakeBaseEndEffectorPoses();
  switch(move(base_end_effector_poses_1)) {
  case 0:
    ROS_INFO("Control SUCCEEDED");
    break;
  case -1:
    ROS_ERROR("Failed to call move_control service");
    run_successful = false;
    break;
  case -2:
    ROS_WARN("Control FAILED");
    run_successful = false;
    break;
  default:
    break;
  }

  if (!run_successful) {
    return run_successful;
  }

  ROS_INFO("Trial 1 SUCEEDED");

  //Trial 2
  auto base_end_effector_poses_2 = getFakeBaseEndEffectorPoses();
  switch(move(base_end_effector_poses_2)) {
  case 0:
    ROS_INFO("Control SUCCEEDED");
    break;
  case -1:
    ROS_ERROR("Failed to call move_control service");
    run_successful = false;
    break;
  case -2:
    ROS_WARN("Control FAILED");
    run_successful = false;
  default:
    break;
  }

  ROS_INFO("Trial 2 SUCEEDED");

  return run_successful;
}


// Move the robot.
// Input: poses must be expressed in the world/map frame.
// Return  0: control execution successful.
// Return -1: service call failed.
// Return -2: control execution failed.
int MotionPlanning::move(const BaseEndEffectorPoses &base_end_effector_poses) {
  // Prepare service call message
  move_control::MoveControlSrv move_control_srv;
  move_control_srv.request.base_pose = base_end_effector_poses.base_pose;
  move_control_srv.request.arm_poses = base_end_effector_poses.end_effector_poses;

  // If cannot call service, return error
  if (!control_clt_.call(move_control_srv)) return -1;
  // If control execution failed, return error
  if (move_control_srv.response.status) return -2;

  // Return successful status
  return 0;
}
