#include "sample_arm_control.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_arm_control");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  SampleArmControl sample_arm_control(argc, argv);

  geometry_msgs::Pose target_pose;
  target_pose.position.x = 0.747;
  target_pose.position.y = -0.131;
  target_pose.position.z = 1.295;
  target_pose.orientation.x = -0.001;
  target_pose.orientation.y = 0.117;
  target_pose.orientation.z = 0.001;
  target_pose.orientation.w = 0.993;

  sample_arm_control.move(target_pose);

  ros::waitForShutdown();
  return 0;
}
