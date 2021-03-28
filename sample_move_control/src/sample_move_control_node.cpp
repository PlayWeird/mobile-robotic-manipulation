#include "sample_move_control.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_move_control");

  SampleMoveControl sample_move_control(argc, argv);

  geometry_msgs::Pose target_pose;
  target_pose.position.x = 1.0;
  target_pose.position.y = 1.0;
  target_pose.position.z = 0.0;
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 1.0;

  ROS_INFO("Sending a goal");

  // Send a goal every 0.5 second until goal received by move-base
  ros::Rate loop_rate(2);
  bool publishing = true;
  while(ros::ok() && publishing) {
    publishing = sample_move_control.publishGoal(target_pose);
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Goal acknowledged by move-base, stopped sending the goal");

  ros::waitForShutdown();
  return 0;
}
