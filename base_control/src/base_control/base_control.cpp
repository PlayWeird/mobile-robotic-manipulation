#include "base_control.h"
#include <move_base_msgs/MoveBaseAction.h>


BaseControl::BaseControl(int argc, char **argv) :
  nh_(new ros::NodeHandle()) {
  init();
  ROS_INFO("Initialized BaseControl");
}


void BaseControl::init() {
  goal_publisher_ = nh_->advertise<move_base_msgs::MoveBaseActionGoal>("/bvr_SIM/move_base/goal", 10);
  goal_status_subscriber_ = nh_->subscribe("/bvr_SIM/move_base/status", 10, &BaseControl::moveBaseStateCallback, this);
  base_control_srv_ = nh_->advertiseService("base_control", &BaseControl::move, this);
  move_base_status_ = 0;
  latest_goal_id_ = std::to_string(ros::Time::now().toSec());
}


void BaseControl::moveBaseStateCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
  if (!msg->status_list.empty() && msg->status_list.begin()->goal_id.id == latest_goal_id_) {
    move_base_status_ = msg->status_list.begin()->status;
  } else {
    move_base_status_ = 0;
  }
}


bool BaseControl::move(base_control::BaseControlSrv::Request  &req,
                       base_control::BaseControlSrv::Response &res) {
  latest_goal_id_ = std::to_string(ros::Time::now().toSec());;

  // Keep publishing until move-base acknowledges the request
  ros::Rate loop_rate_publishing(2);
  bool publishing = true;
  while(ros::ok() && publishing) {
    publishing = publishGoal(req.pose);
    ros::spinOnce();
    loop_rate_publishing.sleep();
  }

  // Looping until move-base finishes moving
  ros::Rate loop_rate_waiting(2);
  while(ros::ok() && move_base_status_ == 1) {
    ros::spinOnce();
    loop_rate_waiting.sleep();
  }

  res.status = 0;
  return true;
}


bool BaseControl::publishGoal(const geometry_msgs::Pose &target_pose) {
  // Keep publishing goals until move-base status is not pending
  if (!move_base_status_) {
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

    return true;
  } else {
    return false;
  }
}
