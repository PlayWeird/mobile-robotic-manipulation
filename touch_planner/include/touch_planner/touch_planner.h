#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_msgs/TFMessage.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <memory>
#include <string>
#include <vector>
#include <utility>

using namespace cv;
using PointList2D = std::vector<Point2f>;


struct Task {
  geometry_msgs::Pose base_pose;
  std::vector<geometry_msgs::Pose> end_effector_poses;
};


class TouchPlanner {
public:
TouchPlanner(int argc, char **argv);

~TouchPlanner() {
  nh_.reset();
}

Task nextTask();
void reportStatus(); // Should have input variables

private:
struct Clusters {
  std::vector<geometry_msgs::Pose> way_points;
  std::vector<geometry_msgs::Pose> touch_points;
  std::vector<std::pair<int, int> > way_touch_association;
};

void init();

std::vector<geometry_msgs::Pose> getWayPoints(const std::vector<geometry_msgs::Pose> &touch_points);

Clusters getClusters();
Clusters clustering(const std::vector<geometry_msgs::Pose> &way_points,
                    const std::vector<geometry_msgs::Pose> &touch_points);

void padConvexHull(float pad, PointList2D &hull_points);
PointList2D subdividePath(int num_subdivisions, const PointList2D &hull_points);

std::unique_ptr<ros::NodeHandle> nh_;

Clusters clusters_;
};
