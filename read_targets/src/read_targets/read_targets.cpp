#include <ros/ros.h>
#include "read_targets.h"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

#include <sstream>
#include <vector>
#include <fstream>
#include <string>
using namespace std;

using Eigen::Vector3d;
using Eigen::Quaterniond;


PoseVector read_targets(const std::string &file_path, const std::string &file_path2) {
  PoseVector pose_vector;
  int size_points = 596;
  pose_vector.resize(size_points);

  string data;
  ifstream file_coord;
  double x_values[size_points];
  double y_values[size_points];
  double z_values[size_points];
  double x_normals[size_points];
  double y_normals[size_points];
  double z_normals[size_points];

  // read position of normals
  file_coord.open(file_path);
  int i = 0, j = 0;
  string data_values;

  if (!file_coord) {
    ROS_ERROR_STREAM("Failed to load " << file_path << " file");
    return pose_vector;
  }

  while (file_coord.good()) {
    getline(file_coord, data);
    j=0;
    istringstream ss(data);

    while(ss>>data_values) {
      // get x axis data
      if (j == 0) {
        x_values[i] = stod(data_values);
        pose_vector[i].position.x = x_values[i];
      }
      // get y axis data
      if (j == 1) {
        y_values[i] = stod(data_values);
        pose_vector[i].position.y = y_values[i];
      }
      // get z axis data
      if (j == 2) {
        z_values[i] = stod(data_values);
        pose_vector[i].position.z = z_values[i];
      }
      ++j;
    }
    ++i;
  }

  file_coord.close(); 

  // read orientation of normals
  ifstream file_orientation;
  file_orientation.open(file_path2);
  i = 0, j = 0;

  if (!file_orientation) {
    ROS_ERROR_STREAM("Failed to load " << file_path2 << " file");
    return pose_vector;
  }

  while (file_orientation.good()) {
    getline(file_orientation, data);
    j=0;
    istringstream ss2(data);
    while(ss2>>data_values) {
      // get x axis data
      if (j == 0) {
        x_normals[i] = stod(data_values);
      }
      // get y axis data
      if (j == 1) {
        y_normals[i] = stod(data_values);
      }
      // get z axis data
      if (j == 2) {
        z_normals[i] = stod(data_values);
      }
      ++j;
    }
  ++i;
  }

  file_orientation.close(); 

  // calculate quaternions from two vectors
  for(int i = 0; i < size_points; ++i) {

    Vector3d normal;
    normal << x_normals[i], y_normals[i], z_normals[i];
    Vector3d axis_vec;
    axis_vec << -1.0, 0.0, 0.0;
    // Finds rotation that maps x-axis vector to normal
    Quaterniond rotation_quaternion = Quaterniond::FromTwoVectors(axis_vec, normal);

    pose_vector[i].orientation.x = rotation_quaternion.x();
    pose_vector[i].orientation.y = rotation_quaternion.y();
    pose_vector[i].orientation.z = rotation_quaternion.z();
    pose_vector[i].orientation.w = rotation_quaternion.w();

  }


  return pose_vector;
}
