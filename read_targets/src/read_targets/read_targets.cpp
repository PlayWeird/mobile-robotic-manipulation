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
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;

constexpr double END_EFFECTOR_OFFSET = -0.045;


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
    // calculate quaternions from two vectors
    // finds rotation that maps x-axis vector to normal
    Vector3d normal;
    normal << x_normals[i], y_normals[i], z_normals[i];
    Vector3d axis_vec;
    axis_vec << -1.0, 0.0, 0.0;
    Quaterniond rotation_quaternion = Quaterniond::FromTwoVectors(axis_vec, normal);

    // Offset the target frames along its x axis.
    // Get rotation matrix from quaternion
    Matrix3d R_matrix = rotation_quaternion.toRotationMatrix();
    // Get translation vector from position
    Vector3d t_vector;
    t_vector << x_values[i], y_values[i], z_values[i];

    // Construct transformation matrix from world frame to target end effector frame
    Matrix4d T_world_ee;
    T_world_ee.setIdentity();
    T_world_ee.block<3,3>(0,0) = R_matrix;
    T_world_ee.block<3,1>(0,3) = t_vector;

    // Construct offset matrix from target end effector to offsetted end effector frame
    Matrix4d T_ee_eeoffset;
    T_ee_eeoffset.setIdentity();
    T_ee_eeoffset(0,3) = END_EFFECTOR_OFFSET;

    // Compute transformation from world to offsetted end effector frame
    Matrix4d T_world_eeoffset = T_world_ee * T_ee_eeoffset;

    // Fill in translation and rotation information.
    pose_vector[i].position.x = T_world_eeoffset(0, 3);
    pose_vector[i].position.y = T_world_eeoffset(1, 3);
    pose_vector[i].position.z = T_world_eeoffset(2, 3);

    Quaterniond rotation_offsetted(T_world_eeoffset.block<3,3>(0, 0));
    pose_vector[i].orientation.x = rotation_offsetted.x();
    pose_vector[i].orientation.y = rotation_offsetted.y();
    pose_vector[i].orientation.z = rotation_offsetted.z();
    pose_vector[i].orientation.w = rotation_offsetted.w();
  }

  return pose_vector;
}
