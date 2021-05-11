#include <iostream>
#include "touch_planner.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "touch_planner");
  // PlannerMetric metric(1.0);
  // UR5Metric* metric = new UR5Metric(1.3, 180.0);
  PlannerMetric metric(1.0, 15.0);
  TouchPlanner planner{metric};

  ROS_INFO("All points acquired!");
  return 0;
}
