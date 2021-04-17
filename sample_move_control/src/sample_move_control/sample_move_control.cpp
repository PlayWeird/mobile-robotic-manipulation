#include "sample_move_control.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <string>


SampleMoveControl::SampleMoveControl(int argc, char **argv) :
  nh_(new ros::NodeHandle()) {
  init();
  ROS_INFO("Initialized SampleMoveControl");
}


SampleMoveControl::~SampleMoveControl() {
  nh_.reset();
}


void SampleMoveControl::init() {
  goal_publisher_ = nh_->advertise<move_base_msgs::MoveBaseActionGoal>("/bvr_SIM/move_base/goal", 10);
  goal_status_subscriber_ = nh_->subscribe("/bvr_SIM/move_base/status", 10, &SampleMoveControl::moveBaseStateCallback, this);
  sample_move_base_status_ = 0;
}


void SampleMoveControl::moveBaseStateCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
  if (!msg->status_list.empty()) {
    sample_move_base_status_ = msg->status_list.begin()->status;
    // std::cout << msg->status_list.size() << ", " << (int)sample_move_base_status_ << std::endl;
    if (sample_move_base_status_ == 3){
    	sample_move_base_status_ = 0;
    }
  }
}


bool SampleMoveControl::publishGoal(const geometry_msgs::Pose &target_pose, std::string goal_id) {
  // Keep publishing goals until move-base status is not pending
  if (!sample_move_base_status_) {
    const auto time_now = ros::Time::now();

    move_base_msgs::MoveBaseActionGoal goal;
    goal.goal.target_pose.pose.position = target_pose.position;
    goal.goal.target_pose.pose.orientation = target_pose.orientation;
    goal.goal.target_pose.header.stamp = time_now;
    goal.goal.target_pose.header.frame_id = "map";

    goal.goal_id.stamp = time_now;
    goal.goal_id.id = goal_id;

    goal.header.frame_id = "map";
    goal.header.stamp = time_now;

    goal_publisher_.publish(goal);

    return true;
  } else {
    return false;
  }

}

char SampleMoveControl::getStatus() {
	return sample_move_base_status_;
}
