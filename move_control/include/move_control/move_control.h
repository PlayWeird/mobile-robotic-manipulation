#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <list>
#include <string>


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;


class MoveControl {
public:
MoveControl(int argc, char **argv);
~MoveControl();

bool moveBaseEndEffector(const geometry_msgs::Pose &target_base_pose,
                         const std::list<geometry_msgs::Pose> &target_end_effector_poses);

private:
void initMoveBase();
void initMoveIt(const std::string &planning_group, const std::string &robot_namespace);

void moveBaseStateCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

// Move end effector to a target pose in planning frame
bool moveEndEffector(const geometry_msgs::Pose &target_pose);

// Move robot base to a target pose in planning frame
void moveBase(const geometry_msgs::Pose &target_pose);

bool publishGoal(const geometry_msgs::Pose &target_pose);

std::unique_ptr<ros::NodeHandle> nh_;
std::unique_ptr<ros::NodeHandle> pnh_;

std::unique_ptr<MoveGroupInterface> move_group_;
const robot_state::JointModelGroup* joint_model_group_;

ros::Publisher move_base_goal_publisher_;
ros::Subscriber move_base_goal_status_subscriber_;

char move_base_status_;

std::string planning_frame_;
std::string end_effector_frame_;

int goal_id_num;
};
