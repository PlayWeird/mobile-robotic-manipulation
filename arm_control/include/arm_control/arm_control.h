#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <arm_control/ArmControlSrv.h>

#include <memory>
#include <string>
#include <vector>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;


class ArmControl {
public:
ArmControl(int argc, char **argv);
~ArmControl();

bool move(arm_control::ArmControlSrv::Request  &req,
          arm_control::ArmControlSrv::Response &res);

private:
enum ServiceStatus {
  SUCCEEDED = 0,
  PLANNING_ERROR = -1,
  CONFIGURATION_RESTORATION_ERROR = -2
};

void init(const std::string &planning_group, const std::string &robot_namespace);

// Move end effector to a target pose in planning frame
bool moveEndEffector(const geometry_msgs::Pose &target_pose);

bool resetToDefaultConfiguration();

std::unique_ptr<ros::NodeHandle> nh_;
std::unique_ptr<ros::NodeHandle> pnh_;

ros::ServiceServer arm_control_srv_;

std::unique_ptr<MoveGroupInterface> move_group_;

std::string planning_frame_;
std::string end_effector_frame_;

std::vector<double> default_arm_configuration_;
};
