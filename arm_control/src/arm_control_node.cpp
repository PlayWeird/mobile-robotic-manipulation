#include <ros/ros.h>
#include "arm_control.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "arm_control");
  // Need at least 2 spinner threads
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ArmControl arm_control{argc, argv};

  ros::waitForShutdown();
  return 0;
}
