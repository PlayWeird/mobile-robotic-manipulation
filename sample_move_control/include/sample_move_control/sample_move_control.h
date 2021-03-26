#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <memory>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SampleMoveControl {
public:
SampleMoveControl(int argc, char **argv);
~SampleMoveControl();

// Move robot base to a target pose in planning frame
bool move(const geometry_msgs::Pose &target_pose);

private:

std::unique_ptr<ros::NodeHandle> nh_;
std::unique_ptr<ros::NodeHandle> pnh_;
ros::Publisher goal_publisher;
};
