#include <ros/ros.h>
#include "touch_planner.h"

constexpr float DISTANCE_THRESHOLD = 1.7;
constexpr float ANGLE_DEGREE_THRESHOLD = 15.0;


int main(int argc, char **argv){
  ros::init(argc, argv, "touch_planner");
  PlannerMetric metric{DISTANCE_THRESHOLD, ANGLE_DEGREE_THRESHOLD};
  TouchPlanner planner{metric};

  ROS_INFO("All points acquired!");
  return 0;
}
