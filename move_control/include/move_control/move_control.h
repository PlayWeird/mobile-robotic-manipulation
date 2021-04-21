#include <ros/ros.h>
#include <move_control/MoveControlSrv.h>
#include <geometry_msgs/Pose.h>

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
enum ServiceStatus {
  SERVICE_SUCCEEDED = 0,
  SERVICE_CALL_ERROR,
  SERVICE_EXECUTION_ERROR
};

enum ControlStatus {
  CONTROL_SUCCEEDED = 0,
  BASE_CONTROL_ERROR = -1,
  ARM_CONTROL_ERROR = -2,
};

void init();

ServiceStatus moveBase(const geometry_msgs::Pose &pose);
ServiceStatus moveEndEffector(const std::vector<geometry_msgs::Pose> &poses);

std::unique_ptr<ros::NodeHandle> nh_;

ros::ServiceServer move_control_srv_;

ros::ServiceClient arm_control_clt_;
ros::ServiceClient base_control_clt_;
};
