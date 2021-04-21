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
    ROS_ERROR("Move-Base service FAILED");
    return SERVICE_CALL_ERROR;
  } else if (base_control_srv.response.status) {
    ROS_WARN("Move-Base execution status FAILED");
    return SERVICE_EXECUTION_ERROR;
  }

  ROS_INFO("Move-Base SUCCEEDED");
  return SERVICE_SUCCEEDED;
}


MoveControl::ServiceStatus MoveControl::moveEndEffector(const std::vector<geometry_msgs::Pose> &poses) {
  arm_control::ArmControlSrv arm_control_srv;
  arm_control_srv.request.poses = poses;

  if (!arm_control_clt_.call(arm_control_srv)) {
    ROS_ERROR("Move-Arm service FAILED");
    return SERVICE_CALL_ERROR;;
  } else if (arm_control_srv.response.status) {
    ROS_WARN("Move-Arm execution status FAILED");
    return SERVICE_EXECUTION_ERROR;
  }

  ROS_INFO("Move-Arm SUCCEEDED");
  return SERVICE_SUCCEEDED;
}


bool MoveControl::move(move_control::MoveControlSrv::Request  &req,
                       move_control::MoveControlSrv::Response &res) {
  ROS_INFO("Move control request received");

  // Call base-control service
  if (moveBase(req.base_pose) != SERVICE_SUCCEEDED) {
    res.status = BASE_CONTROL_ERROR;
    return true;
  }

  // Call arm-control service
  if (moveEndEffector(req.arm_poses) != SERVICE_SUCCEEDED) {
    res.status = ARM_CONTROL_ERROR;
    return true;
  }

  res.status = CONTROL_SUCCEEDED;
  ROS_INFO("Move control executed successfully");
  return true;
}
