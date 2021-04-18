#include <ros/ros.h>
#include "move_control/MoveControlSrv.h"
#include "geometry_msgs/Pose.h"

#include <vector>


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


int main(int argc, char** argv) {
  ros::init(argc, argv, "sample_control");
  ros::NodeHandle nh;

  ros::ServiceClient control_clt = nh.serviceClient<move_control::MoveControlSrv>("move_control");
  move_control::MoveControlSrv move_control_srv;

  // Trial 1
  auto base_pose1 = makeBasePose();
  auto ee_poses1 = makeEndEffectorPoses(base_pose1);

  move_control_srv.request.base_pose = base_pose1;
  move_control_srv.request.arm_poses = ee_poses1;

  ROS_INFO("Service message prepared");

  if (control_clt.call(move_control_srv)) {
    // Check that status is successful (status=0)
    if (!move_control_srv.response.status)
      ROS_INFO("Control SUCCEEDED");
    else {
      ROS_WARN("Control FAILED");
      return -1;
    }
  } else {
    ROS_ERROR("Failed to call service move_control");
    return -1;
  }

  ROS_INFO("Trial 1 SUCCEEDED");

  // Trial 2
  auto base_pose2 = makeBasePose();
  auto ee_poses2 = makeEndEffectorPoses(base_pose2);

  move_control_srv.request.base_pose = base_pose2;
  move_control_srv.request.arm_poses = ee_poses2;

  ROS_INFO("Service message prepared");

  if (control_clt.call(move_control_srv)) {
    // Check that status is successful (status=0)
    if (!move_control_srv.response.status)
      ROS_INFO("Control SUCCEEDED");
    else {
      ROS_WARN("Control FAILED");
      return -1;
    }
  } else {
    ROS_ERROR("Failed to call service move_control");
    return -1;
  }

  ROS_INFO("Trial 2 SUCCEEDED");

  ros::waitForShutdown();
  return 0;
}
