#include <ros/ros.h>
#include "move_control.h"
#include <geometry_msgs/Pose.h>


geometry_msgs::Pose makePose(){
  geometry_msgs::Pose made_pose;
  float random_x;
  float range = 1.0;
  random_x = ((float)std::rand() / (float)RAND_MAX) * range;

  made_pose.position.x = random_x;
  // made_pose.position.x = 1.5;
  made_pose.position.y = 1.5;
  made_pose.position.z = 0.0;
  made_pose.orientation.x = 0.0;
  made_pose.orientation.y = 0.0;
  made_pose.orientation.z = 0.0;
  made_pose.orientation.w = 1.0;

  return made_pose;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "move_control");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  MoveControl move_control{argc, argv};

  // First Trial
  auto base_pose1 = makePose();

  ROS_WARN_STREAM("X value: " << base_pose1.position.x);

  geometry_msgs::Pose ee_pose1;
  ee_pose1.position.x = base_pose1.position.x + 1.0;
  ee_pose1.position.y = base_pose1.position.y - 0.05;
  ee_pose1.position.z = 0.5;
  ee_pose1.orientation.x = base_pose1.orientation.x;
  ee_pose1.orientation.y = base_pose1.orientation.y;
  ee_pose1.orientation.z = base_pose1.orientation.z;
  ee_pose1.orientation.w = base_pose1.orientation.w;

  geometry_msgs::Pose ee_pose2;
  ee_pose2.position.x = base_pose1.position.x + 0.8;
  ee_pose2.position.y = base_pose1.position.y - 0.05;
  ee_pose2.position.z = 0.75;
  ee_pose2.orientation.x = base_pose1.orientation.x;
  ee_pose2.orientation.y = base_pose1.orientation.y;
  ee_pose2.orientation.z = base_pose1.orientation.z;
  ee_pose2.orientation.w = base_pose1.orientation.w;

  std::list<geometry_msgs::Pose> ee_poses1;
  ee_poses1.push_back(ee_pose2);
  ee_poses1.push_back(ee_pose1);

  if(!move_control.moveBaseEndEffector(base_pose1, ee_poses1))
    ROS_WARN("Move control failed");

  // Second trial
  auto base_pose2 = makePose();

  ROS_WARN_STREAM("X value: " << base_pose2.position.x);

  ee_pose1.position.x = base_pose2.position.x + 1.0;
  ee_pose1.position.y = base_pose2.position.y - 0.05;
  ee_pose1.orientation.x = base_pose2.orientation.x;
  ee_pose1.orientation.y = base_pose2.orientation.y;
  ee_pose1.orientation.z = base_pose2.orientation.z;
  ee_pose1.orientation.w = base_pose2.orientation.w;

  ee_pose2.position.x = base_pose2.position.x + 0.8;
  ee_pose2.position.y = base_pose2.position.y - 0.05;
  ee_pose2.orientation.x = base_pose2.orientation.x;
  ee_pose2.orientation.y = base_pose2.orientation.y;
  ee_pose2.orientation.z = base_pose2.orientation.z;
  ee_pose2.orientation.w = base_pose2.orientation.w;

  std::list<geometry_msgs::Pose> ee_poses2;
  ee_poses2.push_back(ee_pose2);
  ee_poses2.push_back(ee_pose1);

  if(!move_control.moveBaseEndEffector(base_pose2, ee_poses2))
    ROS_WARN("Move control failed");

  ros::waitForShutdown();
  return 0;
}
