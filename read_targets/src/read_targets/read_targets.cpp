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
  string data;
  ifstream file_coord;
  vector<double> x_values;
  vector<double> y_values;
  vector<double> z_values;
  vector<double> x_normals;
  vector<double> y_normals;
  vector<double> z_normals;

  // read position of normals
  file_coord.open(file_path);
  int count = 0, j = 0;
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
        x_values.push_back(stod(data_values));
      }
      // get y axis data
      if (j == 1) {
        y_values.push_back(stod(data_values));
      }
      // get z axis data
      if (j == 2) {
        z_values.push_back(stod(data_values));
      }
      ++j;
    }
    ++count;
  }

  file_coord.close(); 

  // read orientation of normals
  ifstream file_orientation;
  file_orientation.open(file_path2);
  count = 0, j = 0;

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
        x_normals.push_back(stod(data_values));
      }
      // get y axis data
      if (j == 1) {
        y_normals.push_back(stod(data_values));
      }
      // get z axis data
      if (j == 2) {
        z_normals.push_back(stod(data_values));
      }
      ++j;
    }
  ++count;
  }

  file_orientation.close(); 

  pose_vector.resize(count-1);

  // assigning data to pose vector
  for(int i = 0; i < count-1; ++i) {

  	pose_vector[i].position.x = x_values[i];
  	pose_vector[i].position.y = y_values[i];
  	pose_vector[i].position.z = z_values[i];

    // calculate quaternions from two vectors
    // finds rotation that maps x-axis vector to normal
    Vector3d normal;
    normal << x_normals[i], y_normals[i], z_normals[i];
    Vector3d axis_vec;
    axis_vec << -1.0, 0.0, 0.0;
    Quaterniond rotation_quaternion = Quaterniond::FromTwoVectors(axis_vec, normal);

    pose_vector[i].orientation.x = rotation_quaternion.x();
    pose_vector[i].orientation.y = rotation_quaternion.y();
    pose_vector[i].orientation.z = rotation_quaternion.z();
    pose_vector[i].orientation.w = rotation_quaternion.w();

  }

  return pose_vector;
}
