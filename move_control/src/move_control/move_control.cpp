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


bool MoveControl::move(move_control::MoveControlSrv::Request  &req,
                       move_control::MoveControlSrv::Response &res) {
  // Call base-control service
  base_control::BaseControlSrv base_control_srv;
  base_control_srv.request.pose = req.base_pose;
  if (base_control_clt_.call(base_control_srv)) {
    if (!base_control_srv.response.status)
      ROS_INFO("Move-base SUCCEEDED");
    else {
      ROS_WARN("Move-base FAILED");
      res.status = -1;
      return false;
    }
  } else {
    ROS_ERROR("Failed to call service base_control");
    res.status = -1;
    return false;
  }

  // Call arm-control service
  arm_control::ArmControlSrv arm_control_srv;
  arm_control_srv.request.poses = req.arm_poses;
  if (arm_control_clt_.call(arm_control_srv)) {
    if (!arm_control_srv.response.status)
      ROS_INFO("Move-arm SUCCEEDED");
    else {
      ROS_WARN("Move-arm FAILED");
      res.status = -2;
      return false;
    }
  } else {
    ROS_ERROR("Failed to call service arm_control");
    res.status = -2;
    return false;
  }

  res.status = 0;
  return true;
}
