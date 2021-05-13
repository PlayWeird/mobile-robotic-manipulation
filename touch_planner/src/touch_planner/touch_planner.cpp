#include "touch_planner.h"
#include "read_targets/read_targets.h"
#include <eigen3/Eigen/Eigen>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/package.h>

#include <iostream>
#include <vector>
#include <climits>


using Eigen::Vector3d;

constexpr float PADDING_SIZE = 0.9;
constexpr int PATH_SUBDIVISIONS = 150;


double magnitude_calculator(Point2f v);
Point2f unit_vector_calculator(Point2f v);
double distance_calculator(geometry_msgs::Pose pt1, geometry_msgs::Pose pt2);
bool all_true(std::vector<bool> bools);

PlannerMetric::PlannerMetric(float distance_threshold, float angle_threshold) {
  distance_threshold_ = distance_threshold;
  angle_threshold_ = (angle_threshold * M_PI) / 180.0;
}


float PlannerMetric::cost(geometry_msgs::Pose way_point,
                          geometry_msgs::Pose touch_point) {

  // Compute distance cost
  double dx = way_point.position.x - touch_point.position.x;
  double dy = way_point.position.y - touch_point.position.y;
  double dz = way_point.position.z - touch_point.position.z;
  double distance_cost = pow(dx*dx + dy*dy + dz*dz, 0.5);

  // Compute angle cost
  double way_touch_vec_x = touch_point.position.x - way_point.position.x;
  double way_touch_vec_y = touch_point.position.y - way_point.position.y;
  double angle = atan2(way_touch_vec_y, way_touch_vec_x);

  tf2::Vector3 z_axis_tf2(0.0, 0.0, 1.0);
  tf2::Quaternion touch_point_orientation(z_axis_tf2, angle);

  tf2::Quaternion way_point_orientation;
  tf2::fromMsg(way_point.orientation, way_point_orientation);

  double angle_cost = fabs(static_cast<double>(way_point_orientation.angle(touch_point_orientation)));
  bool is_feasible = angle_cost <= angle_threshold_;
  is_feasible &= (signbit(way_point.position.y) == signbit(touch_point.position.y));

  return is_feasible ? distance_cost : std::numeric_limits<float>::max();
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
  task_cluster_idx = 0;
}


Task TouchPlanner::nextTask() {
  Task task;
  task.base_pose = clusters_.way_points[task_cluster_idx];

  for(int idx=0; idx < clusters_.way_touch_association[task_cluster_idx].size(); idx++) {
    int touch_idx = clusters_.way_touch_association[task_cluster_idx][idx];
    task.end_effector_poses.push_back(clusters_.touch_points[touch_idx]);
  }

  ROS_INFO_STREAM("Sending planning task: " << task_cluster_idx);

  task_cluster_idx++;
  return task;
}

bool TouchPlanner::has_next_task(){
  int max_index = clusters_.way_touch_association[task_cluster_idx].size();
  return task_cluster_idx < max_index;
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
  float pad_size = PADDING_SIZE;
  padConvexHull(pad_size, hull_points);

  // Subdivide path
  int num_subdivisions = PATH_SUBDIVISIONS;
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

    tf2::Vector3 z_axis_tf2(0.0, 0.0, 1.0);
    tf2::Quaternion orientation(z_axis_tf2, angle);

    way_point.orientation = tf2::toMsg(orientation);

    way_points.push_back(way_point);
  }

  return way_points;
}


TouchPlanner::Clusters TouchPlanner::getClusters() {
  std::string pkg_path = ros::package::getPath("read_targets");
  touch_points = read_targets(
    pkg_path + "/preprocessing/Triangle_center_position.txt",
    pkg_path + "/preprocessing/Triangle_normals.txt"
    );
  const auto way_points = getWayPoints();

  return clustering(way_points);
}


void TouchPlanner::printClusters(Clusters clusters) {
  for(int i=0; i < clusters.way_points.size(); i++) {
    auto wp_pose = clusters.way_points[i];
    std::cout << "(" << wp_pose.position.x << ", " << wp_pose.position.y << ")" << std::endl;
    for(int j=0; j < clusters.way_touch_association[i].size(); j++) {
      auto pose = clusters.touch_points[clusters.way_touch_association[i][j]];
      std::cout << "(" << pose.position.x << ", " << pose.position.y << "),";
    }
    std::cout << std::endl;
  }
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

  while(ros::ok() && !all_true(is_touchpoint_covered)) {
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

  // TODO: fix segfault in sorting cluster
  // sort_clusters_touchpoints(clusters);

  // printClusters(clusters);

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
  for(int i=0; i < metric_matrix[0].size(); i++) {
    if(metric_matrix[waypoint_idx][i] <= metric.getDistanceThreshold()) {
      touchables.push_back(i);
    }
  }
  return touchables;
}


void TouchPlanner::populate_metric_matrix(const PoseList &way_points){
  metric_matrix.clear();
  for(int i=0; i < way_points.size(); i++) {
    std::vector<double> point_group;
    for(int j=0; j < touch_points.size(); j++) {
      point_group.push_back(metric.cost(way_points[i], touch_points[j]));
    }
    metric_matrix.push_back(point_group);
  }
}


void TouchPlanner::populate_touchables_table(const PoseList &way_points){
  // touchables_table[i] is the indices of the touchpoints that way_points[i] can touch
  touchables_table.clear();
  for(int wp_idx=0; wp_idx < way_points.size(); wp_idx++) {
    touchables_table.push_back(get_waypoint_touchables(wp_idx));
  }
}


void TouchPlanner::remove_redundant_wp(
  const PoseList &way_points,
  std::vector<bool> &is_waypoint_considered,
  const std::vector<bool> &is_touchpoint_covered
  ){
  for(int wp_idx=0; wp_idx < way_points.size(); wp_idx++) {
    if(is_waypoint_considered[wp_idx]) {
      bool is_redundant = true;
      int size = touchables_table[wp_idx].size();
      for(int table_idx=0; table_idx < size; table_idx++) {
        int tp_idx = touchables_table[wp_idx][table_idx];
        if(!is_touchpoint_covered[tp_idx]) {
          is_redundant = false;
          break;
        }
      }
      if(is_redundant) {
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
  for(int tp_idx=0; tp_idx < is_touchpoint_covered.size(); tp_idx++) {
    // if current touchpoint is not covered check for unique waypoint
    if(!is_touchpoint_covered[tp_idx]) {
      int times_touched = 0;
      int who_touched = -1;
      for(int wp_idx=0; wp_idx < is_waypoint_considered.size(); wp_idx++) {
        if(is_waypoint_considered[wp_idx]) {
          // if waypoint can touch the touchpoint then mark
          auto wp_touchables = touchables_table[wp_idx];
          if(std::count(wp_touchables.begin(), wp_touchables.end(), tp_idx)) {
            times_touched++;
            if(times_touched > 1) {
              break;
            }
            who_touched = wp_idx;
          }
        }
      }
      // if no waypoint can touch this touchpoint: uh oh
      if(times_touched == 0) {
        std::cout << "ERROR: touchpoint " << tp_idx << ": not touchable!" << std::endl;
        untouchables.push_back(tp_idx);
        // ignoring unreachable touchpoint in subsequent passes.
        is_touchpoint_covered[tp_idx] = true;
      }
      // if touchpoint is only touched by one wp
      else if(times_touched == 1) {
        // add wp idx to the final wp list
        final_waypoint_idxs.push_back(who_touched);
        // update touchpoints covered list
        for(int touch_idx : touchables_table[who_touched]) {
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
  for(int wp_idx=0; wp_idx < is_waypoint_considered.size(); wp_idx++) {
    if(is_waypoint_considered[wp_idx]) {
      float distance_sum = 0.0;
      int size = touchables_table[wp_idx].size();
      for(int table_idx=0; table_idx < size; table_idx++) {
        int tp_idx = touchables_table[wp_idx][table_idx];
        distance_sum += metric_matrix[wp_idx][tp_idx];
      }
      float average_distance = distance_sum / size;
      if(average_distance > max_distance) {
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
  for(int final_idx : final_waypoint_idxs) {
    clusters.way_points.push_back(way_points[final_idx]);
    idx_lookup[final_idx] = cluster_wp_idx;
    cluster_wp_idx++;
    clusters.way_touch_association.push_back(std::vector<int>());
  }

  // sort touchpoints to connect to closest final waypoints
  for(int tp_idx=0; tp_idx < touch_points.size(); tp_idx++) {
    clusters.touch_points.push_back(touch_points[tp_idx]);
    auto it = std::find(untouchables.begin(), untouchables.end(), tp_idx);
    if(it == untouchables.end()) {
      float min_distance = 9999999.9;
      int min_wp_idx = -1;
      for(int idx=0; idx < final_waypoint_idxs.size(); idx++) {
        int final_idx = final_waypoint_idxs[idx];
        if(metric_matrix[final_idx][tp_idx] < min_distance) {
          min_distance = metric_matrix[final_idx][tp_idx];
          min_wp_idx = final_idx;
        }
      }
      int idx = idx_lookup[min_wp_idx];
      clusters.way_touch_association[idx].push_back(tp_idx);
    }
  }
}

bool TouchPlanner::isVisited(bool visited[], const int& N){
  for(int i = 1; i<N; i++) {
    if(visited[i] == false) {
      return false;
    }
  }
  return true;
}


void TouchPlanner::TSP(const int& N, int **cost, std::vector<int> &path){

  static const int M = 1 << (N-1);
  int dp[N][M];
  for(int i = 0; i < N; i++) {
    dp[i][0] = cost[i][0];
  }
  for(int j = 1; j < M; j++) {
    for(int i = 0; i < N; i++ ) {
      dp[i][j] = INT_MAX;
      if( ((j >> (i-1)) & 1) == 1) {
        continue;
      }
      for(int k = 1; k < N; k++) {
        if( ((j >> (k-1)) & 1) == 0) {
          continue;
        }
        if( dp[i][j] > cost[i][k] + dp[k][j^(1<<(k-1))]) {
          dp[i][j] = cost[i][k] + dp[k][j^(1<<(k-1))];
        }
      }
    }
  }

  bool visited[N] = {false};
  int pioneer = 0,min = INT_MAX, S = M - 1,temp;
  path.push_back(0);

  while(!isVisited(visited, N)) {
    for(int i=1; i<N; i++) {
      if(visited[i] == false && (S&(1<<(i-1))) != 0) {
        if(min > cost[i][pioneer] + dp[i][(S^(1<<(i-1)))]) {
          min = cost[i][pioneer] + dp[i][(S^(1<<(i-1)))];
          temp = i;
        }
      }
    }
    pioneer = temp;
    path.push_back(pioneer);
    visited[pioneer] = true;
    S = S ^ (1<<(pioneer - 1));
    min = INT_MAX;
  }
}



void TouchPlanner::sort_clusters_touchpoints(Clusters &clusters){
  //TODO Write sorting algorithm for touchpoint shortest path
  /* this function should update the way_touch_association in clusters
     you will need the metric_matric inorder to get the waypoint to touchpoint
     distances
   */
  for(int index = 0; index < clusters.way_points.size(); index++) {
    int nums = clusters.way_touch_association[index].size();
    int **cost;
    std::vector<int> path;

    cost = new int *[nums+1];
    for(int i = 0; i < nums+1; i++)
      cost[i] = new int[nums+1];

    cost[0][0]=0;
    for(int k = 1; k < nums+1; k++) {
      cost[k][0] = distance_calculator(clusters.way_points[index],clusters.touch_points[k]);
      cost[0][k] = distance_calculator(clusters.way_points[index],clusters.touch_points[k]);
    }

    for(int k= 1; k < nums+1; k++) {
      for(int p = 1; p < nums+1; p++) {
        cost[k][p] = distance_calculator(clusters.touch_points[k],clusters.touch_points[p]);
        cost[p][k] = distance_calculator(clusters.touch_points[k],clusters.touch_points[p]);
      }
    }

    int N = nums+1;
    TSP(N, cost, path);

    for(int k = 0; k< nums; k++) {
      clusters.way_touch_association[index][k] = path[k];
    }
  }
}


double magnitude_calculator(Point2f v){
  return pow(v.x * v.x + v.y * v.y, 0.5);
}


double distance_calculator(geometry_msgs::Pose pt1, geometry_msgs::Pose pt2){
  double dx = pt1.position.x - pt2.position.x;
  double dy = pt1.position.y - pt2.position.y;
  double dz = pt1.position.z - pt2.position.z;
  return pow(dx * dx + dy * dy + dz * dz, 0.5);

}


Point2f unit_vector_calculator(Point2f v){
  return v / magnitude_calculator(v);
}


bool all_true(std::vector<bool> bools){
  return std::all_of(bools.begin(), bools.end(), [](bool v) {
    return v;
  });
}
