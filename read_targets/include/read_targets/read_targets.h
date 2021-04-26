#include <geometry_msgs/Pose.h>

#include <string>
#include <vector>


using PoseVector = std::vector<geometry_msgs::Pose>;

// Get poses of target points from dae file.
PoseVector read_targets(const std::string &file_path);
