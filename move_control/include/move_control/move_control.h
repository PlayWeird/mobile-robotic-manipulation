#include <ros/ros.h>
#include <move_control/MoveControlSrv.h>

#include <vector>


class MoveControl {
public:
MoveControl(int argc, char **argv);
~MoveControl() {
  nh_.reset();
}

bool move(move_control::MoveControlSrv::Request  &req,
          move_control::MoveControlSrv::Response &res);

private:
void init();

std::unique_ptr<ros::NodeHandle> nh_;

ros::ServiceServer move_control_srv_;

ros::ServiceClient arm_control_clt_;
ros::ServiceClient base_control_clt_;
};
