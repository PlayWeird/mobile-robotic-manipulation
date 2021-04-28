#include <iostream>
#include "touch_planner.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "touch_planner");
  TouchPlanner planner{argc, argv};

  ROS_INFO("All points acquired!");
  return 0;
}
