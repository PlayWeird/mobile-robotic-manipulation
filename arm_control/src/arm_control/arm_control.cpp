#include <ros/ros.h>
#include "arm_control.h"
#include <move_base_msgs/MoveBaseAction.h>

#include <string>
#include <memory>

constexpr int RETRIES = 2;


ArmControl::ArmControl(int argc, char **argv) :
  nh_(new ros::NodeHandle()), pnh_(new ros::NodeHandle("~")) {
  std::string arm_namespace;
  pnh_->getParam("arm_namespace", arm_namespace);
  std::string robot_namespace;
  pnh_->getParam("robot_namespace", robot_namespace);
  init(arm_namespace, robot_namespace);

  ROS_INFO("Initialized ArmControl");
}


ArmControl::~ArmControl() {
  nh_.reset();
  pnh_.reset();
  move_group_.reset();
}


void ArmControl::init(const std::string &planning_group, const std::string &robot_namespace) {
  arm_control_srv_ = nh_->advertiseService("arm_control", &ArmControl::move, this);

  move_group_ = std::make_unique<MoveGroupInterface>(planning_group);

  planning_frame_ = move_group_->getPlanningFrame();
  end_effector_frame_ = robot_namespace + "/" + planning_group + "/gripper_manipulation_link";

  joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup(planning_group);

  // Configure robot movement control
  move_group_->setMaxVelocityScalingFactor(0.25);
  move_group_->setMaxAccelerationScalingFactor(1.0);
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(100);
  move_group_->setGoalTolerance(0.01);
  move_group_->setPlannerId("RRTConnectkConfigDefault");

  bool state_monitor_started = move_group_->startStateMonitor();
  ROS_INFO_STREAM("State monitor start " << (state_monitor_started ? "SUCCEEDED" : "FAILED"));

  // Save default arm configuration
  const robot_state::JointModelGroup* joint_model_group =
    move_group_->getCurrentState()->getJointModelGroup(planning_group);
  move_group_->getCurrentState()->copyJointGroupPositions(joint_model_group, default_arm_configuration_);
}


bool ArmControl::move(arm_control::ArmControlSrv::Request  &req,
                      arm_control::ArmControlSrv::Response &res) {
  ROS_INFO("Arm control request received");

  res.status = SUCCEEDED;

  bool moved = false;
  for (const auto &target_end_effector_pose : req.poses) {
    if(!moveEndEffectorTarget(target_end_effector_pose))
      res.status = EXECUTION_ERROR;
    else if (!moved)
      moved = true;
  }

  if (moved && !resetToDefaultConfiguration())
    res.status = CONFIGURATION_RESTORATION_ERROR;

  return true;
}


void ArmControl::setCurrentState() {
  // Get current arm pose
  geometry_msgs::Pose start_pose = move_group_->getCurrentPose().pose;
  robot_state::RobotState start_state(*move_group_->getCurrentState());
  start_state.setFromIK(joint_model_group_, start_pose);
  move_group_->setStartState(start_state);
}


bool ArmControl::moveEndEffector() {
  MoveGroupInterface::Plan plan;

  bool success = false;

  setCurrentState();

  success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_STREAM("Arm planning " << (success ? "SUCCEEDED" : "FAILED"));

  if (!success) return false;

  // If there exists a such a plan, then execute the plan and move the gripper
  success = false;
  for (int i = 0; i < RETRIES && !success; ++i) {
    success = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Arm execution trial " << i + 1 << (success ? " SUCCEEDED" : " FAILED"));
  }

  return success;
}


bool ArmControl::moveEndEffectorTarget(const geometry_msgs::Pose &target_pose) {
  geometry_msgs::PoseStamped target_pose_stamped;
  target_pose_stamped.header.frame_id = planning_frame_;
  target_pose_stamped.header.stamp = ros::Time::now();
  target_pose_stamped.pose = target_pose;

  // Set the target pose for the end effector frame in the planning frame
  move_group_->setPoseTarget(target_pose_stamped, end_effector_frame_);

  return moveEndEffector();
}


bool ArmControl::resetToDefaultConfiguration() {
  move_group_->setJointValueTarget(default_arm_configuration_);

  return moveEndEffector();
}
