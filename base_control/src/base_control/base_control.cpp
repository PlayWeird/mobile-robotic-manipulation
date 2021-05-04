#include "base_control.h"
#include <move_base_msgs/MoveBaseAction.h>


BaseControl::BaseControl(int argc, char **argv) :
  nh_(new ros::NodeHandle()) {
  init();
  ROS_INFO("Initialized BaseControl");
}


void BaseControl::init() {
  goal_publisher_ = nh_->advertise<move_base_msgs::MoveBaseActionGoal>("/bvr_SIM/move_base/goal", 1);
  goal_status_subscriber_ = nh_->subscribe("/bvr_SIM/move_base/status", 1, &BaseControl::moveBaseStateCallback, this);
  base_control_srv_ = nh_->advertiseService("base_control", &BaseControl::move, this);
  move_base_status_ = actionlib_msgs::GoalStatus::PENDING;
  latest_goal_id_ = std::to_string(ros::Time::now().toSec());
}


void BaseControl::moveBaseStateCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
  for (const auto &goal_status : msg->status_list)
    if (goal_status.goal_id.id == latest_goal_id_) {
      move_base_status_ = goal_status.status;
      return;
    }
}


bool BaseControl::move(base_control::BaseControlSrv::Request  &req,
                       base_control::BaseControlSrv::Response &res) {
  ROS_INFO("Base control request received");

  auto time = ros::Time::now();
  latest_goal_id_ = std::to_string(time.toSec());

  // Wait for move_base_status_ to update
  ros::Duration(0.2).sleep();

  // Reset status.
  move_base_status_ = actionlib_msgs::GoalStatus::PENDING;

  // Keep publishing until move-base acknowledges the request
  ros::Rate loop_rate_publishing(2);
  while(ros::ok() && move_base_status_ == actionlib_msgs::GoalStatus::PENDING) {
    publishGoal(req.pose);
    ros::spinOnce();
    loop_rate_publishing.sleep();
  }

  // Looping until move-base finishes moving
  ros::Rate loop_rate_waiting(2);
  while(ros::ok() && move_base_status_ == actionlib_msgs::GoalStatus::ACTIVE) {
    if (ros::Time::now() - time > ros::Duration(10.0)) {
      ROS_INFO("Still planning or moving ...");
      time = ros::Time::now();
    }
    ros::spinOnce();
    loop_rate_waiting.sleep();
  }

  switch(move_base_status_) {
  case actionlib_msgs::GoalStatus::SUCCEEDED:
    res.status = SUCCEEDED;
    return true;
    break;
  case actionlib_msgs::GoalStatus::ABORTED:
  case actionlib_msgs::GoalStatus::REJECTED:
    ROS_WARN("Cannot plan move-base path");
    res.status = PLANNING_ERROR;
    return true;
    break;
  default:
    ROS_WARN_STREAM("Unknown move-base error: " << static_cast<int>(move_base_status_));
    res.status = UNKNOWN_ERROR;
    return false;
    break;
  }
}


void BaseControl::publishGoal(const geometry_msgs::Pose &target_pose) {
  const auto time_now = ros::Time::now();

  move_base_msgs::MoveBaseActionGoal goal;
  goal.goal.target_pose.pose.position = target_pose.position;
  goal.goal.target_pose.pose.orientation = target_pose.orientation;
  goal.goal.target_pose.header.stamp = time_now;
  goal.goal.target_pose.header.frame_id = "map";

  goal.goal_id.stamp = time_now;
  goal.goal_id.id = latest_goal_id_;

  goal.header.frame_id = "map";
  goal.header.stamp = time_now;

  goal_publisher_.publish(goal);
}
