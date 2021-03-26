#include "sample_move_control.h"

SampleMoveControl::SampleMoveControl(int argc, char **argv) :
	nh_(new ros::NodeHandle()),
	pnh_(new ros::NodeHandle("~")) {
	goal_publisher = nh_->advertise<move_base_msgs::MoveBaseActionGoal>("bvr_SIM/move_base/goal", 10);
	ROS_INFO("Initialized SampleMoveControl");
}

SampleMoveControl::~SampleMoveControl() {
	nh_.reset();
	pnh_.reset();
}

bool SampleMoveControl::move(const geometry_msgs::Pose &target_pose) {
	// MoveBaseClient ac("move_base", true);

	// move_base_msgs::MoveBaseGoal goal;
	// goal.target_pose.pose.position = target_pose.position;

	// goal.target_pose.header.frame_id = "bvr_SIM/bvr_base_link";
	// goal.target_pose.header.stamp = ros::Time::now();

	// ROS_INFO("Sending goal");
	// ac.sendGoal(goal);
	// ac.waitForResult(ros::Duration(5.0));

	// if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	// 	ROS_INFO("Hooray!");
	// 	return 0;
	// }
	// else{
	// 	ROS_INFO("FUCK!!!");
	// 	return -1;
	// }

	move_base_msgs::MoveBaseActionGoal goal;
	goal.goal.target_pose.pose.position = target_pose.position;
	goal.goal.target_pose.pose.orientation = target_pose.orientation;

	goal.header.frame_id = "bvr_SIM/bvr_base_link";
	goal.header.stamp = ros::Time::now();

	// geometry_msgs::PoseStamped goal_msg;
	// goal_msg.header.frame_id = "bvr_SIM/bvr_base_link";
	// goal_msg.header.stamp = ros::Time::now();
	// goal_msg.pose = target_pose;
	goal_publisher.publish(goal);

	return true;
}

