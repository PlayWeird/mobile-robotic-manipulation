#include <ros/ros.h>
#include "base_control.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_control");

  BaseControl base_control{argc, argv};

  ros::waitForShutdown();
  return 0;
}
