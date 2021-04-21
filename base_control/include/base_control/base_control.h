#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Pose.h>

#include "base_control/BaseControlSrv.h"

#include <memory>
#include <string>


class BaseControl {
public:
BaseControl(int argc, char** argv);
~BaseControl() {
  nh_.reset();
}

bool move(base_control::BaseControlSrv::Request  &req,
          base_control::BaseControlSrv::Response &res);

private:
enum ServiceStatus {
  SUCCEEDED = 0,
  PLANNING_ERROR = -1,
  UNKNOWN_ERROR = -2
};

void init();

void moveBaseStateCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

bool publishGoal(const geometry_msgs::Pose &target_pose);

std::unique_ptr<ros::NodeHandle> nh_;
ros::Publisher goal_publisher_;
ros::Subscriber goal_status_subscriber_;
ros::ServiceServer base_control_srv_;

char move_base_status_;

std::string latest_goal_id_;
};
