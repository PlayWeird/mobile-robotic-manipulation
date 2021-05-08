#include "touch_planner.h"
#include "read_targets/read_targets.h"
#include <eigen3/Eigen/Eigen>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <limits>
#include <cmath>

using Eigen::Vector3d;


double magnitude_calculator(Point2f v);
Point2f unit_vector_calculator(Point2f v);
bool all_true(std::vector<bool> bools);

PlannerMetric::PlannerMetric(float distance_threshold, float angle_threshold) {
  distance_threshold_ = distance_threshold;
  angle_threshold_ = angle_threshold;
}


float PlannerMetric::cost(geometry_msgs::Pose way_point,
                          geometry_msgs::Pose touch_point) {
    // double dx = way_point.position.x - touch_point.position.x;
    // double dy = way_point.position.y - touch_point.position.y;
    // double dz = way_point.position.z - touch_point.position.z;
    //
    // return pow(dx*dx + dy*dy + dz*dz, 0.5);

    // Compute distance cost
    double dx = way_point.position.x - touch_point.position.x;
    double dy = way_point.position.y - touch_point.position.y;
    double dz = way_point.position.z - touch_point.position.z;
    double distance_cost = pow(dx*dx + dy*dy + dz*dz, 0.5);

    // Compute angle cost
    double way_touch_vec_x = touch_point.position.x - way_point.position.x;
    double way_touch_vec_y = touch_point.position.y - way_point.position.y;

    double angle = atan2(way_touch_vec_y, way_touch_vec_x);
    if (way_touch_vec_x < 0.0)
      angle += M_PI;
    else if (way_touch_vec_y < 0.0)
      angle += 2*M_PI;

    tf2::Vector3 z_axis_tf2(0.0, 0.0, 1.0);
    tf2::Quaternion touch_point_orientation(z_axis_tf2, angle);

    tf2::Quaternion way_point_orientation;
    tf2::fromMsg(way_point.orientation, way_point_orientation);

    double angle_cost = fabs(static_cast<double>(way_point_orientation.angle(touch_point_orientation)));
    std::cout << angle_cost << std::endl;
    return angle_cost <= angle_threshold_ ? distance_cost : std::numeric_limits<float>::max();
}


TouchPlanner::TouchPlanner(PlannerMetric &metric) :
  nh_(new ros::NodeHandle()), metric(metric) {
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


void TouchPlanner::reportStatus() { // have input variable here
  // TODO: allow motion planning to report status of task
}


PoseList TouchPlanner::getWayPoints() {
  double x, y, z;
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
  float pad_size = .2;
  padConvexHull(pad_size, hull_points);

  // Subdivide path
  int num_subdivisions = 10;
  auto sample_points = subdividePath(num_subdivisions, hull_points);

  PoseList way_points;
  for (int i = 0; i < sample_points.size(); ++i) {
    auto current_point = sample_points[i % sample_points.size()];
    auto next_point = sample_points[(i + 1) % sample_points.size()];

    geometry_msgs::Pose way_point;
    way_point.position.x = current_point.x;
    way_point.position.y = current_point.y;
    way_point.position.z = 0.0;

    double vec_x = next_point.x - current_point.x;
    double vec_y = next_point.y - current_point.y;
    double vec_z = 0.0;

    double z_axis_x = 0.0;
    double z_axis_y = 0.0;
    double z_axis_z = 1.0;

    Vector3d vec;
    vec << vec_x, vec_y, vec_z;

    Vector3d z_axis;
    z_axis << z_axis_x, z_axis_y, z_axis_z;

    auto heading = vec.cross(z_axis);

    double angle = atan2(heading(1), heading(0));
    if (heading(0) < 0.0)
      angle += M_PI;
    else if (heading(1) < 0.0)
      angle += 2*M_PI;

    tf2::Vector3 z_axis_tf2(0.0, 0.0, 1.0);
    tf2::Quaternion orientation(z_axis_tf2, angle);

    way_point.orientation = tf2::toMsg(orientation);

    way_points.push_back(way_point);
  }

  return way_points;
}


TouchPlanner::Clusters TouchPlanner::getClusters() {
  touch_points = read_targets(
    "/home/gaetano/autonomous_mobile_manipulation_ws/src/mobile-robotic-manipulation/read_targets/preprocessing/Triangle_center_position.txt",
    "/home/gaetano/autonomous_mobile_manipulation_ws/src/mobile-robotic-manipulation/read_targets/preprocessing/Triangle_normals.txt"
    );
  const auto way_points = getWayPoints();

  return clustering(way_points);
}


TouchPlanner::Clusters TouchPlanner::clustering(const PoseList &way_points){
  Clusters clusters;

  std::vector<bool> is_touchpoint_covered(touch_points.size(), false);
  std::vector<bool> is_waypoint_considered(way_points.size(), true);
  std::vector<int> final_waypoint_idxs;

  // populate metric matrix with distance from every touchpoint to every waypoint
  populate_metric_matrix(way_points);
  populate_touchables_table(way_points);
  // check for unique touch points
  check_for_unique_tp(
    is_touchpoint_covered,
    is_waypoint_considered,
    final_waypoint_idxs
  );

  while(ros::ok() && !all_true(is_touchpoint_covered)){
    // remove redundant waypoints
    remove_redundant_wp(way_points, is_waypoint_considered, is_touchpoint_covered);

    // remove worst waypoint
    remove_worst_wp(is_waypoint_considered);

    // check for unique touch points
    check_for_unique_tp(
      is_touchpoint_covered,
      is_waypoint_considered,
      final_waypoint_idxs
    );
  }

  // sort the final waypoints and add waypoint Poses to the cluster
  std::sort(final_waypoint_idxs.begin(), final_waypoint_idxs.end());
  package_clusters(clusters, way_points, final_waypoint_idxs);

  sort_clusters_touchpoints(clusters);

  // PRINT-OUT TO VISUALIZE FINAL CLUSTER IN DESMOS
  for(int i=0; i < clusters.way_points.size(); i++){
    auto wp_pose = clusters.way_points[i];
    std::cout << "(" << wp_pose.position.x << ", " << wp_pose.position.y << ")" << std::endl;
    for(int j=0; j < clusters.way_touch_association[i].size(); j++){
      auto pose = clusters.touch_points[clusters.way_touch_association[i][j]];
      std::cout << "(" << pose.position.x << ", " << pose.position.y << "),";
    }
    std::cout << std::endl;
  }

  return clusters;
}


void TouchPlanner::padConvexHull(double pad, PointList2D &hull_points){
  // pad the boats hull
  auto M = moments(hull_points);
  double cx = M.m10 / (M.m00 + 0.0001);
  double cy = M.m01 / (M.m00 + 0.0001);
  Point2f offset = Point2f(cx, cy);
  for(int i=0; i < hull_points.size(); i++) {
    hull_points[i] -= offset;
    hull_points[i] += pad * unit_vector_calculator(hull_points[i]);
    hull_points[i] += offset;
  }
}


PointList2D TouchPlanner::subdividePath(int num_subdivisions,
                                        const PointList2D &hull_points){
  // find the length of the path.
  PointList2D unit_vectors;
  std::vector<double> path_distances;
  double total_path_length = 0;
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
  double distance_from_prev;
  for(int i=0; i < num_subdivisions; i++) {
    double sample_distance = i * total_path_length / (num_subdivisions - 1);
    while(sample_distance > path_distances[hull_index]) {
      hull_index++;
    }
    distance_from_prev = sample_distance - path_distances[hull_index-1];
    vec_from_prev = unit_vectors[hull_index-1] * distance_from_prev;
    sample_points.push_back(hull_points[hull_index-1] + vec_from_prev);
  }
  return sample_points;
}


std::vector<int> TouchPlanner::get_waypoint_touchables(int waypoint_idx) {
    std::vector<int> touchables;
    for(int i=0; i < metric_matrix[0].size(); i++){
      if(metric_matrix[waypoint_idx][i] <= metric.getDistanceThreshold()){
        touchables.push_back(i);
      }
    }
    return touchables;
}


void TouchPlanner::populate_metric_matrix(const PoseList &way_points){
  metric_matrix.clear();
  for(int i=0; i < way_points.size(); i++){
    std::vector<double> point_group;
    for(int j=0; j < touch_points.size(); j++){
      point_group.push_back(metric.cost(way_points[i], touch_points[j]));
    }
    metric_matrix.push_back(point_group);
  }
}


void TouchPlanner::populate_touchables_table(const PoseList &way_points){
  // touchables_table[i] is the indices of the touchpoints that way_points[i] can touch
  touchables_table.clear();
  for(int wp_idx=0; wp_idx < way_points.size(); wp_idx++){
    touchables_table.push_back(get_waypoint_touchables(wp_idx));
  }
}


void TouchPlanner::remove_redundant_wp(
  const PoseList &way_points,
  std::vector<bool> &is_waypoint_considered,
  const std::vector<bool> &is_touchpoint_covered
){
  for(int wp_idx=0; wp_idx < way_points.size(); wp_idx++){
    if(is_waypoint_considered[wp_idx]){
      bool is_redundant = true;
      int size = touchables_table[wp_idx].size();
      for(int table_idx=0; table_idx < size; table_idx++){
        int tp_idx = touchables_table[wp_idx][table_idx];
        if(!is_touchpoint_covered[tp_idx]){
          is_redundant = false;
          break;
        }
      }
      if(is_redundant){
        is_waypoint_considered[wp_idx] = false;
      }
    }
  }
}


void TouchPlanner::check_for_unique_tp(
  std::vector<bool> &is_touchpoint_covered,
  std::vector<bool> &is_waypoint_considered,
  std::vector<int> &final_waypoint_idxs
){
  /*
  Update touchpoints_covered and final_waypoint_idxs if we find a touch point
  that can only be reached by a single waypoint
  */
  for(int tp_idx=0; tp_idx < is_touchpoint_covered.size(); tp_idx++){
    // if current touchpoint is not covered check for unique waypoint
    if(!is_touchpoint_covered[tp_idx]){
      int times_touched = 0;
      int who_touched = -1;
      for(int wp_idx=0; wp_idx < is_waypoint_considered.size(); wp_idx++){
        if(is_waypoint_considered[wp_idx]){
          // if waypoint can touch the touchpoint then mark
          auto wp_touchables = touchables_table[wp_idx];
          if(std::count(wp_touchables.begin(), wp_touchables.end(), tp_idx)){
            times_touched++;
            if(times_touched > 1){
              break;
            }
            who_touched = wp_idx;
          }
        }
      }
      // if no waypoint can touch this touchpoint: uh oh
      if(times_touched == 0){
        std::cout << "ERROR: touchpoint " << tp_idx << ": not touchable!" << std::endl;
        untouchables.push_back(tp_idx);
        // ignoring unreachable touchpoint in subsequent passes.
        is_touchpoint_covered[tp_idx] = true;
      }
      // if touchpoint is only touched by one wp
      else if(times_touched == 1){
        // add wp idx to the final wp list
        final_waypoint_idxs.push_back(who_touched);
        // update touchpoints covered list
        for(int touch_idx : touchables_table[who_touched]){
          is_touchpoint_covered[touch_idx] = true;
        }
        // remove new final waypoint from considered list
        is_waypoint_considered[who_touched] = false;
      }
    }
  }
}


void TouchPlanner::remove_worst_wp(std::vector<bool> &is_waypoint_considered){
  float max_distance = 0.0;
  int max_index = -1;
  for(int wp_idx=0; wp_idx < is_waypoint_considered.size(); wp_idx++){
    if(is_waypoint_considered[wp_idx]){
      float distance_sum = 0.0;
      int size = touchables_table[wp_idx].size();
      for(int table_idx=0; table_idx < size; table_idx++){
        int tp_idx = touchables_table[wp_idx][table_idx];
        distance_sum += metric_matrix[wp_idx][tp_idx];
      }
      float average_distance = distance_sum / size;
      if(average_distance > max_distance){
        max_distance = average_distance;
        max_index = wp_idx;
      }
    }
  }
  is_waypoint_considered[max_index] = false;
}


void TouchPlanner::package_clusters(
  Clusters &clusters,
  const PoseList &way_points,
  const std::vector<int> &final_waypoint_idxs
){
  // mapping from idx in way_points to idx in Clusters.way_points
  std::unordered_map<int, int> idx_lookup;
  int cluster_wp_idx = 0;
  for(int final_idx : final_waypoint_idxs){
    clusters.way_points.push_back(way_points[final_idx]);
    idx_lookup[final_idx] = cluster_wp_idx;
    cluster_wp_idx++;
    clusters.way_touch_association.push_back(std::vector<int>());
  }

  // sort touchpoints to connect to closest final waypoints
  for(int tp_idx=0; tp_idx < touch_points.size(); tp_idx++){
    clusters.touch_points.push_back(touch_points[tp_idx]);
    auto it = std::find(untouchables.begin(), untouchables.end(), tp_idx);
    if(it == untouchables.end()){
      float min_distance = 9999999.9;
      int min_wp_idx = -1;
      for(int idx=0; idx < final_waypoint_idxs.size(); idx++){
        int final_idx = final_waypoint_idxs[idx];
        if(metric_matrix[final_idx][tp_idx] < min_distance){
          min_distance = metric_matrix[final_idx][tp_idx];
          min_wp_idx = final_idx;
        }
      }
      int idx = idx_lookup[min_wp_idx];
      clusters.way_touch_association[idx].push_back(tp_idx);
    }
  }
}

void TouchPlanner::sort_clusters_touchpoints(Clusters &clusters){
  //TODO Write sorting algorithm for touchpoint shortest path
  /* this function should update the way_touch_association in clusters
    you will need the metric_matric inorder to get the waypoint to touchpoint
    distances
  */
}

double magnitude_calculator(Point2f v){
  return pow(v.x * v.x + v.y * v.y, 0.5);
}


Point2f unit_vector_calculator(Point2f v){
  return v / magnitude_calculator(v);
}


bool all_true(std::vector<bool> bools){
  return std::all_of(bools.begin(), bools.end(), [](bool v) {return v;});
}
