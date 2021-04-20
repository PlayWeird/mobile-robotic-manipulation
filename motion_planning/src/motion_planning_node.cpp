#include <ros/ros.h>
#include "motion_planning.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_planning");

  MotionPlanning motion_planning{argc, argv};

  motion_planning.run();

  ros::waitForShutdown();
  return 0;
}
