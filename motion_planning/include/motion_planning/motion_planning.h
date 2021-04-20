#include <ros/ros.h>
#include <move_control.h>
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

void init() {
  control_clt = nh_->serviceClient<move_control::MoveControlSrv>("move_control");
}

int move(const BaseEndEffectorPoses &base_end_effector_poses);

std::unique<ros::NodeHandle> nh_;
ros::ServiceClient control_clt_;
};
