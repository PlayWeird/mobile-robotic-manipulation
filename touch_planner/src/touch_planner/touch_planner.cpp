#include "touch_planner.h"
#include "read_targets/read_targets.h"


float magnitude_calculator(Point2f v);
Point2f unit_vector_calculator(Point2f v);


TouchPlanner::TouchPlanner(int argc, char **argv) :
  nh_(new ros::NodeHandle()) {
  init();
  ROS_INFO("Initialized TouchPlanner");
  waiting_for_points = true;
}


TouchPlanner::Clusters TouchPlanner::getClusters() {
  const auto target_end_effector_poses = read_targets(
    "/home/eric/Documents/ROS_Projects/autonomous_mobile_manipulation_ws/src/mobile-robotic-manipulation/read_targets/preprocessing/Triangle_center_position.txt",
    "/home/eric/Documents/ROS_Projects/autonomous_mobile_manipulation_ws/src/mobile-robotic-manipulation/read_targets/preprocessing/Triangle_normals.txt"
    );

  float x, y, z;
  PointList2D hull_points;
  // Project transforms onto ground plane and store
  for(auto pose : target_end_effector_poses) {
    x = pose.position.x;
    y = pose.position.y;
    z = pose.position.z;
    mesh_points_2D_.push_back(Point2f(x, y));
    mesh_points_3D_.push_back(Point3f(x, y, z));
  }

  waiting_for_points = false;

  // std::cout << mesh_points_2D_ << std::endl << std::endl;
  // Get the convex hull of the boat
  convexHull(mesh_points_2D_, hull_points, true);

  std::cout << hull_points << std::endl << std::endl;

  // Pad convex hull
  padConvexHull(.3, hull_points);
  std::cout << hull_points << std::endl << std::endl;

  // Subdivide path
  auto sample_points = subdividePath(100, hull_points);

  std::cout << sample_points << std::endl << std::endl;
  // Cluster hull_points to all sampled points

  // Variables

  // heuristic metric can be a function for more complex heuristics
  float heuristic_metric = 1.0;   // in meters
  // carry along cluster metric for base_waypoint elimination
  float worst_cluster_center_metric;
  int worst_cluster_center_index = 0;
};


// TouchPlanner::Clusters TouchPlanner::clustering(const std::vector<geometry_msgs::Pose> &way_points,
//                                                 const std::vector<geometry_msgs::Pose> &touch_points) {
//   // TODO: perform clustering
// };


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
