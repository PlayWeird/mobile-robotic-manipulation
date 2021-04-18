#include <ros/ros.h>
#include <tinyxml.h>
#include "read_dae.h"
#include <eigen3/Eigen/Eigen>

#include <sstream>
#include <vector>


using Eigen::Matrix3d;
using Eigen::Vector3d;


Matrix3d computeRotation(Vector3d vec_source, Vector3d vec_target);
Matrix3d skew_symmetric(Vector3d vec);


PoseVector read_dae(const std::string &file_path) {
  PoseVector pose_vector;

  TiXmlDocument doc{file_path};
  bool check_file = doc.LoadFile();
  if (check_file) {
    ROS_INFO_STREAM("Opened " << file_path << "file");
  } else {
    ROS_ERROR_STREAM("Failed to load " << file_path << " file");
    return pose_vector;
  }

  int size_points;
  int size_total;
  std::vector<double> x_values;
  std::vector<double> y_values;
  std::vector<double> z_values;

  std::vector<double> x_normal;
  std::vector<double> y_normal;
  std::vector<double> z_normal;

  double transformation_matrix[4][4];

  TiXmlElement *l_pRootElement = doc.RootElement();
  if(l_pRootElement) {
    // Read the points
    TiXmlElement *l_library_geometries = l_pRootElement->FirstChildElement("library_geometries");
    if (l_library_geometries) {
      TiXmlElement *l_geometry = l_library_geometries->FirstChildElement("geometry");
      if (l_geometry) {
        TiXmlElement *l_mesh = l_geometry->FirstChildElement("mesh");
        if (l_mesh) {
          for (TiXmlElement* l_source = l_mesh->FirstChildElement("source"); l_source; l_source = l_source->NextSiblingElement("source")) {
            const char *attributeOfSource = l_source->Attribute("id");

            if (!strcmp(attributeOfSource, "riva_1-mesh-positions")) {
              TiXmlElement *l_float_array = l_source->FirstChildElement("float_array");
              if (l_float_array) {
                const char *attributeOfFloat_array = l_float_array->Attribute("id");
                const char *attributeOfFloat_array_count = l_float_array->Attribute("count");
                size_total = strtol(attributeOfFloat_array_count, NULL, 10);
                const char* points_str = l_float_array->GetText();

                size_points = size_total / 3;

                x_values.resize(size_points);
                y_values.resize(size_points);
                z_values.resize(size_points);

                std::stringstream points_stream(points_str);
                float data[size_total];
                for(int i = 0; i < size_total; ++i) {
                  points_stream >> data[i];
                  for(int i = 0; i < size_total; ++i) {
                    // Get x axis data
                    if ((i)%3 == 0) {
                      x_values[i/3] = data[i];
                    }
                    // Get y axis data
                    if ((i)%3 == 1) {
                      y_values[i/3] = data[i];
                    }
                    // Get z axis data
                    if ((i)%3 == 2) {
                      z_values[i/3] = data[i];
                    }
                  }
                }
              }
            }

            // Read the normals
            if(strcmp(attributeOfSource,"riva_1-mesh-normals") == 0) {
              TiXmlElement *l_float_array = l_source->FirstChildElement("float_array");
              if (l_float_array) {
                const char* points_str = l_float_array->GetText();

                x_normal.resize(size_points);
                y_normal.resize(size_points);
                z_normal.resize(size_points);

                std::stringstream points_stream(points_str);
                float data[size_total];
                for(int i=0; i < size_total; ++i) {
                  points_stream >> data[i];
                  for(int i=0; i < size_total; ++i) {
                    // Get x-axis data
                    if ((i)%3 == 0) {
                      x_normal[i/3] = data[i];
                    }
                    // Get y-axis data
                    if ((i)%3 == 1) {
                      y_normal[i/3] = data[i];
                    }
                    // Get z-axis data
                    if ((i)%3 == 2) {
                      z_normal[i/3] = data[i];
                    }
                  }
                }
              }
            }
          }
        }
      }
    }

    // Read transformation matrix
    TiXmlElement *l_library_visual_scenes = l_pRootElement->FirstChildElement("library_visual_scenes");
    if (l_library_visual_scenes) {
      TiXmlElement *l_visual_scene = l_library_visual_scenes->FirstChildElement("visual_scene");
      if(l_visual_scene) {
        TiXmlElement *l_node = l_visual_scene->FirstChildElement("node");
        if(l_node) {
          TiXmlElement *l_matrix = l_node->FirstChildElement("matrix");
          if(l_matrix) {
            const char* matrix_str = l_matrix->GetText();

            std::stringstream matrix_stream(matrix_str);

            for (int i=0; i<4; i++) {
              for (int j=0; j<4; j++) {
                matrix_stream >> transformation_matrix[i][j];
              }
            }
          }
        }
      }
    }
  }

  pose_vector.resize(size_points);

  for(int i = 0; i < size_points; ++i) {
    // Compute rotation
    Vector3d normal;
    normal << x_normal[i], y_normal[i], z_normal[i];

    Vector3d x_axis_vec;
    x_axis_vec << 1.0, 0.0, 0.0;

    // Finds rotation that maps x-axis vector to normal
    Matrix3d rotation = computeRotation(x_axis_vec, normal);

    // Compute transformed point
    // double point_transf[] = { 0.0, 0.0, 0.0, 0.0 };
    // double point[] = { x_values[i], y_values[i], z_values[i], 1.0 };
    // for (int i = 0; i < 4; ++i) {
    //   for (int j = 0; j < 4; ++j) {
    //     point_transf[i] += transformation_matrix[i][j] * point[j];
    //   }
    // }

    // Todo: continue computing pose

    // Convert to geometry_msgs Pose message
    pose_vector[i].position.x = point_transf[0];
    pose_vector[i].position.y = point_transf[1];
    pose_vector[i].position.z = point_transf[2];
    pose_vector[i].orientation.x = 0.0;
    pose_vector[i].orientation.y = 0.0;
    pose_vector[i].orientation.z = 0.0;
    pose_vector[i].orientation.w = 1.0;
  }

  return pose_vector;
}


Matrix3d skew_symmetric(Vector3d vec) {
  Matrix3d skew_symmetric_matrix;
  skew_symmetric_matrix <<
    0.0, -vec(2), vec(1),
    vec(2), 0.0, -vec(0),
    -vec(1), vec(0), 0.0;

  return skew_symmetric_matrix;
}


Matrix3d computeRotation(Vector3d vec_source, Vector3d vec_target) {
  Vector3d v = vec_source.cross(vec_target);
  double s = v.norm();
  double c = vec_source.dot(vec_target);

  Matrix3d I = Matrix3d::Identity();
  Matrix3d skew_v = skew_symmetric(v);

  return I + skew_v + skew_v * skew_v * ((1.0 - c) / (s * s));
}