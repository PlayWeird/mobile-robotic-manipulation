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
#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext.hpp>

using namespace cv;
using PointList2D = std::vector<Point2f>;
using PoseList = std::vector<geometry_msgs::Pose>;


struct Task {
  geometry_msgs::Pose base_pose;
  PoseList end_effector_poses;
};


class TouchPlanner {
public:
TouchPlanner(int argc, char **argv);

~TouchPlanner() {
  nh_.reset();
}

Task nextTask();
void reportStatus(); // Should have input variables

  class PlannerMetric {
  public:
    PlannerMetric(){};
    ~PlannerMetric(){};
    // virtual
  };

  private:
  struct Clusters {
    PoseList way_points;
    PoseList touch_points;
    std::vector<std::pair<int, int> > way_touch_association;
  };

void init();

PoseList getWayPoints(const PoseList &touch_points);

Clusters getClusters();
Clusters clustering(const PoseList &way_points,
                    const PoseList &touch_points,
                    PlannerMetric &metric);

void padConvexHull(double pad, PointList2D &hull_points);
PointList2D subdividePath(int num_subdivisions, const PointList2D &hull_points);

std::unique_ptr<ros::NodeHandle> nh_;

Clusters clusters_;
};
