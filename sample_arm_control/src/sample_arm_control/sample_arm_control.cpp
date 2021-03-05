#include "sample_arm_control.h"


SampleArmControl::SampleArmControl(int argc, char **argv) :
  nh_(new ros::NodeHandle()),
  pnh_(new ros::NodeHandle("~")) {
  std::string arm_namespace;
  pnh_->getParam("arm_namespace", arm_namespace);
  std::string robot_namespace;
  pnh_->getParam("robot_namespace", robot_namespace);
  init(arm_namespace, robot_namespace);

  ROS_INFO("Initialized SampleArmControl");
}

SampleArmControl::~SampleArmControl() {
  nh_.reset();
  pnh_.reset();
  move_group_.reset();
}

void SampleArmControl::init(const std::string &planning_group, const std::string &robot_namespace) {
  move_group_ = std::make_unique<MoveGroupInterface>(planning_group);

  planning_frame_ = move_group_->getPlanningFrame();
  end_effector_frame_ = robot_namespace + "/" + planning_group + "/gripper_manipulation_link";

  ROS_INFO_STREAM("Planning Frame: " << planning_frame_);
  ROS_INFO_STREAM("End Effector Frame: " << end_effector_frame_);

  joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup(planning_group);

  move_group_->setMaxVelocityScalingFactor(0.25);
  move_group_->setMaxAccelerationScalingFactor(1.0);
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(100);
  move_group_->setGoalTolerance(0.02);
  move_group_->setPlannerId("RRTConnectkConfigDefault");
}

bool SampleArmControl::move(const geometry_msgs::Pose &target_pose) {
  geometry_msgs::Pose start_pose = move_group_->getCurrentPose().pose;

  robot_state::RobotState start_state(*move_group_->getCurrentState());
  start_state.setFromIK(joint_model_group_, start_pose);
  move_group_->setStartState(start_state);

  ROS_INFO_STREAM("Start pose: " << start_pose.position.x
                                 << ' ' << start_pose.position.y
                                 << ' ' << start_pose.position.z
                                 << ' ' << start_pose.orientation.x
                                 << ' ' << start_pose.orientation.y
                                 << ' ' << start_pose.orientation.z
                                 << ' ' << start_pose.orientation.w);

  geometry_msgs::PoseStamped target_pose_stamped;
  target_pose_stamped.header.frame_id = planning_frame_;
  target_pose_stamped.header.stamp = ros::Time::now();
  target_pose_stamped.pose = target_pose;

  move_group_->setPoseTarget(target_pose_stamped, end_effector_frame_);

  MoveGroupInterface::Plan plan;
  bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_STREAM("Planning " << (success ? "SUCCEEDED" : "FAILED"));

  if (success)
    move_group_->asyncExecute(plan);

  return success;
}
