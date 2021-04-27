#include <iostream>
#include "touch_planner.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "touch_planner");
	TouchPlanner planner(0, NULL);
	ros::Rate loop_rate(20);
	while(ros::ok && planner.waiting_for_points){
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("All points acquired!");
}