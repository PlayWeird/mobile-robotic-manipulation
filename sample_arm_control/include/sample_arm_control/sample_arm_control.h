#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <string>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;


class SampleArmControl {
public:
SampleArmControl(int argc, char **argv);
~SampleArmControl();

bool move(const geometry_msgs::Pose &target_pose);

private:
void init(const std::string &planning_group, const std::string &robot_namespace);

std::unique_ptr<ros::NodeHandle> nh_;
std::unique_ptr<ros::NodeHandle> pnh_;

std::unique_ptr<MoveGroupInterface> move_group_;
const robot_state::JointModelGroup* joint_model_group_;

std::string planning_frame_;
std::string end_effector_frame_;
};
