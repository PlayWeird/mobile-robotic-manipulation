#include <ros/ros.h>
#include "move_control/MoveControlSrv.h"
#include <geometry_msgs/Pose.h>

#include <vector>
#include <memory>


struct BaseEndEffectorPoses {
  geometry_msgs::Pose base_pose;
  std::vector<geometry_msgs::Pose> end_effector_poses;
};


class MotionPlanning {
public:
MotionPlanning(int argc, char **argv);
~MotionPlanning() {
  nh_.reset();
}

bool run();

private:

enum ServiceStatus {
  SUCCEEDED = 0,
  SERVICE_CALL_ERROR,
  SERVICE_EXECUTION_ERROR
};

void init() {
  control_clt_ = nh_->serviceClient<move_control::MoveControlSrv>("move_control");
}

ServiceStatus move(const BaseEndEffectorPoses &base_end_effector_poses);

std::unique_ptr<ros::NodeHandle> nh_;
ros::ServiceClient control_clt_;
};
