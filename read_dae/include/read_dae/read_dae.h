#include <geometry_msgs/Pose.h>

#include <string>
#include <vector>


using PoseVector = std::vector<geometry_msgs::Pose>;


PoseVector read_dae(const std::string &file_path);
