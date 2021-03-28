#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Pose.h>
#include <memory>


class SampleMoveControl {
public:
SampleMoveControl(int argc, char **argv);
~SampleMoveControl();

// Move robot base to a target pose in planning frame
bool publishGoal(const geometry_msgs::Pose &target_pose);

private:

void init();

void moveBaseStateCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

std::unique_ptr<ros::NodeHandle> nh_;
ros::Publisher goal_publisher_;
ros::Subscriber goal_status_subscriber_;

char sample_move_base_status_;
};
