#include <ros/ros.h>
#include "move_control.h"
#include <move_base_msgs/MoveBaseAction.h>

#include <string>
#include <memory>


MoveControl::MoveControl(int argc, char **argv) :
  nh_(new ros::NodeHandle()), pnh_(new ros::NodeHandle("~")) {
  // Initialize move-base
  initMoveBase();

  // Initialize moveit
  std::string arm_namespace;
  pnh_->getParam("arm_namespace", arm_namespace);
  std::string robot_namespace;
  pnh_->getParam("robot_namespace", robot_namespace);
  initMoveIt(arm_namespace, robot_namespace);

  goal_id_num = 0;

  ROS_INFO("Initialized MoveControl");
}


MoveControl::~MoveControl() {
  nh_.reset();
  pnh_.reset();
  move_group_.reset();
}


void MoveControl::initMoveBase() {
  move_base_goal_publisher_ = nh_->advertise<move_base_msgs::MoveBaseActionGoal>("/bvr_SIM/move_base/goal", 10);
  move_base_goal_status_subscriber_ = nh_->subscribe("/bvr_SIM/move_base/status", 10, &MoveControl::moveBaseStateCallback, this);
  move_base_status_ = 0;
}


void MoveControl::initMoveIt(const std::string &planning_group, const std::string &robot_namespace) {
  move_group_ = std::make_unique<MoveGroupInterface>(planning_group);

  planning_frame_ = move_group_->getPlanningFrame();
  end_effector_frame_ = robot_namespace + "/" + planning_group + "/gripper_manipulation_link";

  joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup(planning_group);

  // Configure robot movement control
  move_group_->setMaxVelocityScalingFactor(0.25);
  move_group_->setMaxAccelerationScalingFactor(1.0);
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(100);
  move_group_->setGoalTolerance(0.02);
  move_group_->setPlannerId("RRTConnectkConfigDefault");
}


void MoveControl::moveBaseStateCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
  if (!msg->status_list.empty()) {
    move_base_status_ = msg->status_list.begin()->status;
    if (move_base_status_ == 3) {
      move_base_status_ = 0;
    }
  }
}


bool MoveControl::moveBaseEndEffector(const geometry_msgs::Pose &target_base_pose,
                                      const std::list<geometry_msgs::Pose> &target_end_effector_poses) {
  // Move base
  ROS_INFO("Moving base");
  moveBase(target_base_pose);
  ++goal_id_num;
  // Loop until move base finishes moving
  ros::Rate loop_rate(2);
  while(ros::ok() && move_base_status_ != 0) {
    ROS_INFO("Waiting for move-base");
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Move end effector
  ROS_INFO("Moving end-effector");
  for (const auto &target_end_effector_pose : target_end_effector_poses) {
    if(!moveEndEffector(target_end_effector_pose))
      return false;
  }

  ROS_INFO("Base End-Effector Movement Completed");
  return true;
}


bool MoveControl::moveEndEffector(const geometry_msgs::Pose &target_pose) {
  geometry_msgs::PoseStamped target_pose_stamped;
  target_pose_stamped.header.frame_id = planning_frame_;
  target_pose_stamped.header.stamp = ros::Time::now();
  target_pose_stamped.pose = target_pose;

  // Set the target pose for the end effector frame in the planning frame
  move_group_->setPoseTarget(target_pose_stamped, end_effector_frame_);

  // Compute a plan to reach the target end effector frame pose
  MoveGroupInterface::Plan plan;
  bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_STREAM("Planning " << (success ? "SUCCEEDED" : "FAILED"));

  // If there exists a such a plan, then execute the plan and move the gripper
  if (success) {
    move_group_->execute(plan);
  }

  return success;
}


void MoveControl::moveBase(const geometry_msgs::Pose &target_pose) {
  ros::Rate loop_rate(2);
  bool publishing = true;
  while(ros::ok() && publishing) {
    publishing = publishGoal(target_pose);
    ros::spinOnce();
    loop_rate.sleep();
  }
}


bool MoveControl::publishGoal(const geometry_msgs::Pose &target_pose) {
  // Keep publishing goals until move-base status is not pending
  if (!move_base_status_) {
    const auto time_now = ros::Time::now();

    move_base_msgs::MoveBaseActionGoal goal;
    goal.goal.target_pose.pose.position = target_pose.position;
    goal.goal.target_pose.pose.orientation = target_pose.orientation;
    goal.goal.target_pose.header.stamp = time_now;
    goal.goal.target_pose.header.frame_id = "map";

    goal.goal_id.stamp = time_now;
    goal.goal_id.id = std::string("move_base_goal") + std::to_string(goal_id_num);

    goal.header.frame_id = "map";
    goal.header.stamp = time_now;

    move_base_goal_publisher_.publish(goal);

    return true;
  } else {
    return false;
  }
}
