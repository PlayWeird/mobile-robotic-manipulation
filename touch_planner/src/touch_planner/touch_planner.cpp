#include "touch_planner.h"
#include "read_targets/read_targets.h"


float magnitude_calculator(Point2f v);
Point2f unit_vector_calculator(Point2f v);


TouchPlanner::TouchPlanner(int argc, char **argv) :
  nh_(new ros::NodeHandle()) {
  init();
  ROS_INFO("Initialized TouchPlanner");
}


void TouchPlanner::init() {
  const auto clusters = getClusters();
  clusters_.way_points = clusters.way_points;
  clusters_.touch_points = clusters.touch_points;
  clusters_.way_touch_association = clusters.way_touch_association;
}


Task TouchPlanner::nextTask() {
  // TODO: send a task to the motion planning
  return Task();
}


void TouchPlanner::reportStatus() {
  // TODO: allow motion planning to report status of task
}


std::vector<geometry_msgs::Pose> TouchPlanner::getWayPoints(const std::vector<geometry_msgs::Pose> &touch_points) {
  float x, y, z;
  PointList2D hull_points;
  // Project transforms onto ground plane and store
  PointList2D mesh_points_2D;
  for(auto pose : touch_points) {
    x = pose.position.x;
    y = pose.position.y;
    z = pose.position.z;
    mesh_points_2D.push_back(Point2f(x, y));
  }

  // Get the convex hull of the boat
  convexHull(mesh_points_2D, hull_points, true);

  // Pad convex hull
  padConvexHull(.3, hull_points);

  // Subdivide path
  auto sample_points = subdividePath(100, hull_points);

  std::vector<geometry_msgs::Pose> way_points;
  for (const auto &point : sample_points) {
    geometry_msgs::Pose way_point;
    way_point.position.x = point.x;
    way_point.position.y = point.y;
    way_point.position.z = 0.0;
    // TODO compute orientation for waypoints
    way_point.orientation.x = 0.0;
    way_point.orientation.y = 0.0;
    way_point.orientation.z = 0.0;
    way_point.orientation.w = 1.0;

    way_points.push_back(way_point);
  }

  return way_points;
}


TouchPlanner::Clusters TouchPlanner::getClusters() {
  const auto touch_points = read_targets(
    "/home/eric/Documents/ROS_Projects/autonomous_mobile_manipulation_ws/src/mobile-robotic-manipulation/read_targets/preprocessing/Triangle_center_position.txt",
    "/home/eric/Documents/ROS_Projects/autonomous_mobile_manipulation_ws/src/mobile-robotic-manipulation/read_targets/preprocessing/Triangle_normals.txt"
    );

  const auto way_points = getWayPoints(touch_points);

  return clustering(way_points, touch_points);
};


TouchPlanner::Clusters TouchPlanner::clustering(const std::vector<geometry_msgs::Pose> &way_points,
                                                const std::vector<geometry_msgs::Pose> &touch_points) {
  Clusters clusters;

  // heuristic metric can be a function for more complex heuristics
  float heuristic_metric = 1.0;             // in meters
  // carry along cluster metric for base_waypoint elimination
  float worst_cluster_center_metric;
  int worst_cluster_center_index = 0;

  // TODO: write clustering algorithm

  return clusters;
};


void TouchPlanner::padConvexHull(float pad, PointList2D &hull_points){
  // pad the boats hull
  auto M = moments(hull_points);
  float cx = M.m10 / (M.m00 + 0.0001);
  float cy = M.m01 / (M.m00 + 0.0001);
  Point2f offset = Point2f(cx, cy);
  for(int i=0; i < hull_points.size(); i++) {
    hull_points[i] -= offset;
    hull_points[i] += pad * unit_vector_calculator(hull_points[i]);
    hull_points[i] += offset;
  }
}


PointList2D TouchPlanner::subdividePath(int num_subdivisions, const PointList2D &hull_points){
  // find the length of the path.
  PointList2D unit_vectors;
  std::vector<float> path_distances;
  float total_path_length = 0;
  Point2f tangent_vector;
  for(int i=0; i < hull_points.size(); i++) {
    path_distances.push_back(total_path_length);
    if(i < hull_points.size() - 1) {
      tangent_vector = hull_points[i+1] - hull_points[i];
      unit_vectors.push_back(unit_vector_calculator(tangent_vector));
      total_path_length += magnitude_calculator(tangent_vector);
    }
  }

  // subdivide path into evenly spaced points
  PointList2D sample_points;
  int hull_index = 1;
  Point2f vec_from_prev;
  float distance_from_prev;
  for(int i=0; i < num_subdivisions; i++) {
    float sample_distance = i * total_path_length / (num_subdivisions - 1);
    while(sample_distance > path_distances[hull_index]) {
      hull_index++;
    }
    distance_from_prev = sample_distance - path_distances[hull_index-1];
    vec_from_prev = unit_vectors[hull_index-1] * distance_from_prev;
    sample_points.push_back(hull_points[hull_index-1] + vec_from_prev);
  }
  return sample_points;
}


float magnitude_calculator(Point2f v){
  return pow(v.x * v.x + v.y * v.y, 0.5);
}


Point2f unit_vector_calculator(Point2f v){
  return v / magnitude_calculator(v);
}
