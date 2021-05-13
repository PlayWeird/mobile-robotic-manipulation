#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_msgs/TFMessage.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <algorithm>
#include <unordered_map>
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

class PlannerMetric {
public:
PlannerMetric(float distance_threshold, float angle_threshold);
~PlannerMetric(){
};

float cost(geometry_msgs::Pose way_point,
           geometry_msgs::Pose touch_point);

float getDistanceThreshold() {
  return distance_threshold_;
}

float getAngleThreshold() {
  return angle_threshold_;
}

protected:
float distance_threshold_;
float angle_threshold_;
};

class TouchPlanner {
public:
TouchPlanner(PlannerMetric &metric);
~TouchPlanner() {
  nh_.reset();
}

PoseList touch_points;
std::vector<int> untouchables;
Task nextTask();
void reportStatus();   // Should have input variables
bool has_next_task();
bool isVisited(bool visited[], const int& N);
void TSP(const int& N, int **cost, std::vector<int> &path);

private:
struct Clusters {
  PoseList way_points;
  PoseList touch_points;
  std::vector<std::vector<int> > way_touch_association;
};
std::unique_ptr<ros::NodeHandle> nh_;
std::vector<std::vector<double> > metric_matrix;
Clusters clusters_;
PlannerMetric metric;
std::vector<std::vector<int> > touchables_table;
int task_cluster_idx;

void init();

PoseList getWayPoints();
Clusters getClusters();
Clusters clustering(const PoseList &way_points);
void padConvexHull(double pad, PointList2D &hull_points);
PointList2D subdividePath(
  int num_subdivisions,
  const PointList2D &hull_points
  );
std::vector<int> get_waypoint_touchables(int waypoint_idx);
void populate_metric_matrix(const PoseList &way_points);
void populate_touchables_table(const PoseList &way_points);
void remove_redundant_wp(
  const PoseList &way_points,
  std::vector<bool> &is_waypoint_considered,
  const std::vector<bool> &is_touchpoint_covered
  );
void check_for_unique_tp(
  std::vector<bool> &is_touchpoint_covered,
  std::vector<bool> &is_waypoint_considered,
  std::vector<int> &final_waypoint_idxs
  );

void remove_worst_wp(std::vector<bool> &is_waypoint_considered);

void package_clusters(
  Clusters &clusters,
  const PoseList &way_points,
  const std::vector<int> &final_waypoint_idxs
  );

void printClusters(Clusters clusters);

void sort_clusters_touchpoints(Clusters &clusters);
};
