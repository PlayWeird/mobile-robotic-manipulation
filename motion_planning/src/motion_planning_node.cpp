#include <ros/ros.h>
#include "motion_planning.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_planning");

  MotionPlanning motion_planning{argc, argv};

  if(motion_planning.run())
    ROS_INFO("Run SUCCEEDED");
  else
    ROS_WARN("Run FAILED");

  ros::waitForShutdown();
  return 0;
}
