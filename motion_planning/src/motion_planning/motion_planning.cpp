#include <ros/ros.h>
#include "motion_planning.h"
#include <geometry_msgs/Pose.h>


MotionPlanning::MotionPlanning(int argc, char **argv, PlannerMetric metric) :
  nh_(new ros::NodeHandle()), touch_planner_(metric) {
  init();
  ROS_INFO("Initalized MotionPlanning");
}

/* ---------Experiment starts here--------- */
geometry_msgs::Pose makeBasePose(){
  geometry_msgs::Pose made_pose;
  float random_x;
  float range = 1.0;
  random_x = ((float)std::rand() / (float)RAND_MAX) * range;

  made_pose.position.x = random_x;
  made_pose.position.y = 1.5;
  made_pose.position.z = 0.0;
  made_pose.orientation.x = 0.0;
  made_pose.orientation.y = 0.0;
  made_pose.orientation.z = 0.0;
  made_pose.orientation.w = 1.0;

  return made_pose;
}


std::vector<geometry_msgs::Pose> makeEndEffectorPoses(const geometry_msgs::Pose &base_pose) {
  geometry_msgs::Pose ee_pose1;
  ee_pose1.position.x = base_pose.position.x + 1.0;
  ee_pose1.position.y = base_pose.position.y - 0.05;
  ee_pose1.position.z = 0.5;
  ee_pose1.orientation.x = base_pose.orientation.x;
  ee_pose1.orientation.y = base_pose.orientation.y;
  ee_pose1.orientation.z = base_pose.orientation.z;
  ee_pose1.orientation.w = base_pose.orientation.w;

  geometry_msgs::Pose ee_pose2;
  ee_pose2.position.x = base_pose.position.x + 0.8;
  ee_pose2.position.y = base_pose.position.y - 0.05;
  ee_pose2.position.z = 0.75;
  ee_pose2.orientation.x = base_pose.orientation.x;
  ee_pose2.orientation.y = base_pose.orientation.y;
  ee_pose2.orientation.z = base_pose.orientation.z;
  ee_pose2.orientation.w = base_pose.orientation.w;

  std::vector<geometry_msgs::Pose> ee_poses;
  ee_poses.push_back(ee_pose2);
  ee_poses.push_back(ee_pose1);

  return ee_poses;
}


Task getFakeTask() {
  auto base_pose = makeBasePose();
  return Task{.base_pose = base_pose, .end_effector_poses = makeEndEffectorPoses(base_pose)};
}


Task get_69_73_poses() {
  geometry_msgs::Pose pose_73;
  pose_73.position.x = 2.667;
  pose_73.position.y = -0.472;
  pose_73.position.z = 0.440;
  pose_73.orientation.x = 0.116;
  pose_73.orientation.y = 0.036;
  pose_73.orientation.z = 0.000;
  pose_73.orientation.w = 0.993;

  geometry_msgs::Pose pose_69;
  pose_69.position.x = 2.975;
  pose_69.position.y = -0.469;
  pose_69.position.z = 0.458;
  pose_69.orientation.x = 0.061;
  pose_69.orientation.y = -0.018;
  pose_69.orientation.z = 0.000;
  pose_69.orientation.w = 0.998;

  std::vector<geometry_msgs::Pose> ee_poses;
  ee_poses.push_back(pose_73);
  ee_poses.push_back(pose_69);

  geometry_msgs::Pose base_pose;
  base_pose.position.x = 2.033;
  base_pose.position.y = -1.1;
  base_pose.position.z = 0.0;
  base_pose.orientation.x = 0.0;
  base_pose.orientation.y = 0.0;
  base_pose.orientation.z = 0.0;
  base_pose.orientation.w = 1.0;

  return Task{.base_pose = base_pose, .end_effector_poses = ee_poses};
}
/* ---------Experiment ends here--------- */


bool MotionPlanning::run() {
  bool run_successful = true;

  // TODO: call touch planner to get the next task.
  // TODO: report task status to touch planner to get the next task.
  while (touch_planner_.has_next_task()) {
    auto base_end_effector_poses1 = touch_planner_.nextTask();

    switch(move(base_end_effector_poses1)) {
    case SUCCEEDED:
      ROS_INFO("Control SUCCEEDED");
      break;
    case SERVICE_CALL_ERROR:
      ROS_ERROR("Failed to call move_control service");
      run_successful = false;
      break;
    case SERVICE_EXECUTION_ERROR:
      ROS_WARN("Control execution FAILED");
      run_successful = false;
      break;
    default:
      break;
    }
  }
  // auto base_end_effector_poses1 = getFakeTask();

  // switch(move(base_end_effector_poses1)) {
  // case SUCCEEDED:
  //   ROS_INFO("Control SUCCEEDED");
  //   break;
  // case SERVICE_CALL_ERROR:
  //   ROS_ERROR("Failed to call move_control service");
  //   run_successful = false;
  //   break;
  // case SERVICE_EXECUTION_ERROR:
  //   ROS_WARN("Control execution FAILED");
  //   run_successful = false;
  //   break;
  // default:
  //   break;
  // }
  //
  // auto base_end_effector_poses2 = getFakeTask();
  // switch(move(base_end_effector_poses2)) {
  // case SUCCEEDED:
  //   ROS_INFO("Control SUCCEEDED");
  //   break;
  // case SERVICE_CALL_ERROR:
  //   ROS_ERROR("Failed to call move_control service");
  //   run_successful = false;
  //   break;
  // case SERVICE_EXECUTION_ERROR:
  //   ROS_WARN("Control execution FAILED");
  //   run_successful = false;
  //   break;
  // default:
  //   break;
  // }

  return run_successful;
}


// Move the robot.
// Input: poses must be expressed in the world/map frame.
// Return  0: control execution successful.
// Return -1: service call failed.
// Return -2: control execution failed.
MotionPlanning::ServiceStatus MotionPlanning::move(const Task &base_end_effector_poses) {
  // Prepare service call message
  move_control::MoveControlSrv move_control_srv;
  move_control_srv.request.base_pose = base_end_effector_poses.base_pose;
  move_control_srv.request.arm_poses = base_end_effector_poses.end_effector_poses;

  // If cannot call service, return error
  if (!control_clt_.call(move_control_srv)) return SERVICE_CALL_ERROR;
  // If control execution failed, return error
  if (move_control_srv.response.status) return SERVICE_EXECUTION_ERROR;

  // Return successful status
  return SUCCEEDED;
}
