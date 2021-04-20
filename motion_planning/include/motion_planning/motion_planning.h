#include <ros/ros.h>
#include <move_control.h>
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
void init() {
  control_clt = nh_->serviceClient<move_control::MoveControlSrv>("move_control");
}

int move(const geometry_msgs::Pose &base_pose, const std::vector<geometry_msgs::Pose> &end_effector_poses);

std::unique<ros::NodeHandle> nh_;
ros::ServiceClient control_clt_;
};
