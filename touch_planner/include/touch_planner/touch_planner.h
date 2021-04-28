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
using PointList3D = std::vector<Point3f>;


class TouchPlanner {
public:
TouchPlanner(int argc, char **argv);

~TouchPlanner() {
  nh_.reset();
}

bool waiting_for_points;

private:
struct Clusters {
  std::vector<geometry_msgs::Pose> way_points;
  std::vector<geometry_msgs::Pose> touch_points;
  std::vector<std::pair<int, int> > way_touch_association;
};

void init() {
  auto clusters = getClusters();
  // TODO: filter clusters
}

Clusters getClusters();
// Clusters clustering();

void padConvexHull(float pad, PointList2D &hull_points);
PointList2D subdividePath(int num_subdivisions, const PointList2D &hull_points);

std::unique_ptr<ros::NodeHandle> nh_;

PointList2D mesh_points_2D_;
PointList3D mesh_points_3D_;
};
