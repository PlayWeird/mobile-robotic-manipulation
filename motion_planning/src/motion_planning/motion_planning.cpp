#include <ros/ros.h>
#include "motion_planning.h"
#include <geomtry_msgs/Pose.h>


MotionPlanning::MotionPlanning(int argc, char **argv) :
  nh_(new ros::NodeHandle()){
  init();
  ROS_INFO("Initalized MotionPlanning");
}


bool MotionPlanning::run() {

}
