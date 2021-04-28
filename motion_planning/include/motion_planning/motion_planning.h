#include <ros/ros.h>
#include <touch_planner/touch_planner.h>
#include "move_control/MoveControlSrv.h"
#include <geometry_msgs/Pose.h>

#include <vector>
#include <memory>


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

ServiceStatus move(const Task &base_end_effector_poses);

std::unique_ptr<ros::NodeHandle> nh_;
ros::ServiceClient control_clt_;

TouchPlanner touch_planner_;
};
