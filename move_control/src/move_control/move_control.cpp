#include <ros/ros.h>
#include "move_control.h"
#include <arm_control/ArmControlSrv.h>
#include <base_control/BaseControlSrv.h>


MoveControl::MoveControl(int argc, char **argv) :
  nh_(new ros::NodeHandle()) {
  init();
  ROS_INFO("Initialized MoveControl");
}


void MoveControl::init() {
  move_control_srv_ = nh_->advertiseService("move_control", &MoveControl::move, this);
  arm_control_clt_ = nh_->serviceClient<arm_control::ArmControlSrv>("arm_control");
  base_control_clt_ = nh_->serviceClient<base_control::BaseControlSrv>("base_control");
}


MoveControl::ServiceStatus MoveControl::moveBase(const geometry_msgs::Pose &pose) {
  base_control::BaseControlSrv base_control_srv;
  base_control_srv.request.pose = pose;

  if (!base_control_clt_.call(base_control_srv)) {
    ROS_ERROR("Failed to call service base_control");
    return SERVICE_CALL_ERROR;
  } else if (base_control_srv.response.status) {
    ROS_WARN("Move-Base FAILED");
    return SERVICE_EXECUTION_ERROR;
  }

  ROS_INFO("Move-Base SUCCEEDED");
  return SUCCEEDED;
}


MoveControl::ServiceStatus MoveControl::moveEndEffector(const std::vector<geometry_msgs::Pose> &poses) {
  arm_control::ArmControlSrv arm_control_srv;
  arm_control_srv.request.poses = poses;

  if (!arm_control_clt_.call(arm_control_srv)) {
    ROS_ERROR("Failed to call service arm_control");
    return SERVICE_CALL_ERROR;;
  } else if (arm_control_srv.response.status) {
    ROS_WARN("Move-Arm FAILED");
    return SERVICE_EXECUTION_ERROR;
  }

  ROS_INFO("Move-Arm SUCCEEDED");
  return SUCCEEDED;
}


bool MoveControl::move(move_control::MoveControlSrv::Request  &req,
                       move_control::MoveControlSrv::Response &res) {
  ROS_INFO("Move control request received");

  // Call base-control service
  if (moveBase(req.base_pose) != SUCCEEDED) {
    res.status = -1;
    return false;
  }

  // Call arm-control service
  if (moveEndEffector(req.arm_poses) != SUCCEEDED) {
    res.status = -2;
    return false;
  }

  res.status = 0;
  return true;
}
