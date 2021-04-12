#include "sample_move_control.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose make_pose(){
  geometry_msgs::Pose made_pose;
  float random_x;
  float range = 5.0;
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_move_control");

  SampleMoveControl sample_move_control(argc, argv);
  std::vector<geometry_msgs::Pose> poses;
  int num_poses = 3;

  for(int i=0; i<num_poses; i++){
    poses.push_back(make_pose());
  }

  ros::Rate loop_rate(2);
  bool publishing = true;

  // Move through a vector of goals
  for(int i=0; i<num_poses; i++){
    std::string goal_id = "goal_" + std::to_string(i);
    publishing = true;
    ROS_INFO(("Sending goal: " + goal_id).c_str());
    // Send a goal every 0.5 second until goal received by move-base
    while(ros::ok() && publishing) {
      publishing = sample_move_control.publishGoal(poses[i], goal_id);
      ros::spinOnce();
      loop_rate.sleep();
    }

    ROS_INFO(("Goal acknowledged by move-base, stopped sending the goal: " + goal_id).c_str());
    // Wait for robot to reach current goal before moving to next.
    while(ros::ok() && sample_move_control.getStatus() != 0){
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  ROS_INFO("All way points visited!");

  ros::waitForShutdown();
  return 0;
}
