#include "sample_move_control.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_move_control");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  SampleMoveControl sample_move_control(argc, argv);

  geometry_msgs::Pose target_pose;
  target_pose.position.x = 1.0;
  target_pose.position.y = 1.0;
  target_pose.position.z = 0.0;
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 1.0;

  sample_move_control.move(target_pose);

  ros::waitForShutdown();
  return 0;
}
