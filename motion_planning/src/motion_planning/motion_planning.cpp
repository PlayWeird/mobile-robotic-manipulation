#include <ros/ros.h>
#include "motion_planning.h"
#include <geometry_msgs/Pose.h>


MotionPlanning::MotionPlanning(int argc, char **argv, PlannerMetric metric) :
  nh_(new ros::NodeHandle()), touch_planner_(metric) {
  init();
  ROS_INFO("Initalized MotionPlanning");
}


bool MotionPlanning::run() {
  bool run_successful = true;

  while (touch_planner_.has_next_task()) {
    auto base_end_effector_poses1 = touch_planner_.nextTask();

    switch(move(base_end_effector_poses1)) {
    case SUCCEEDED:
      ROS_INFO("Control SUCCEEDED");
      break;
    case SERVICE_CALL_ERROR:
      ROS_ERROR("Failed to call move_control service");
      run_successful = false;
      break;
    case SERVICE_EXECUTION_ERROR:
      ROS_WARN("Control execution FAILED");
      run_successful = false;
      break;
    default:
      break;
    }
  }

  return run_successful;
}


// Move the robot.
// Input: poses must be expressed in the world/map frame.
// Return  0: control execution successful.
// Return -1: service call failed.
// Return -2: control execution failed.
MotionPlanning::ServiceStatus MotionPlanning::move(const Task &base_end_effector_poses) {
  // Prepare service call message
  move_control::MoveControlSrv move_control_srv;
  move_control_srv.request.base_pose = base_end_effector_poses.base_pose;
  move_control_srv.request.arm_poses = base_end_effector_poses.end_effector_poses;

  // If cannot call service, return error
  if (!control_clt_.call(move_control_srv)) return SERVICE_CALL_ERROR;
  // If control execution failed, return error
  if (move_control_srv.response.status) return SERVICE_EXECUTION_ERROR;

  // Return successful status
  return SUCCEEDED;
}
