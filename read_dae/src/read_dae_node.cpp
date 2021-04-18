#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "read_dae.h"

#include <string>


int main( int argc, char** argv ) {
  ros::init(argc, argv, "read_dae");
  ros::NodeHandle nh;
  ros::NodeHandle pnh{"~"};

  std::string file_path;
  pnh.getParam("file_path", file_path);

  const auto target_end_effector_poses = read_dae(file_path);

  tf2_ros::StaticTransformBroadcaster static_broadcaster;

  ros::Rate loop_rate(20);
  while(ros::ok()) {
    for (int i = 0; i < target_end_effector_poses.size(); ++i) {
      geometry_msgs::TransformStamped transformed_stamp;
      transformed_stamp.header.stamp = ros::Time::now();
      transformed_stamp.header.frame_id = "map";
      transformed_stamp.child_frame_id = std::to_string(i);
      transformed_stamp.transform.translation.x = target_end_effector_poses[i].position.x;
      transformed_stamp.transform.translation.y = target_end_effector_poses[i].position.y;
      transformed_stamp.transform.translation.z = target_end_effector_poses[i].position.z;
      transformed_stamp.transform.rotation.x = target_end_effector_poses[i].orientation.x;
      transformed_stamp.transform.rotation.y = target_end_effector_poses[i].orientation.y;
      transformed_stamp.transform.rotation.z = target_end_effector_poses[i].orientation.z;
      transformed_stamp.transform.rotation.w = target_end_effector_poses[i].orientation.w;
      static_broadcaster.sendTransform(transformed_stamp);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
