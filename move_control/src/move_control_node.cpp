#include <ros/ros.h>
#include "move_control.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "move_control");

  MoveControl move_control{argc, argv};

  ros::spin();
  return 0;
}
