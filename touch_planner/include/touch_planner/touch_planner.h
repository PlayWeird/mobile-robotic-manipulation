#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_msgs/TFMessage.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <memory>
#include <string>
#include <vector>

using namespace cv;
typedef std::vector<Point2f> PointList2D;
typedef std::vector<Point3f> PointList3D;

class TouchPlanner {
	public:
	TouchPlanner(int argc, char **argv);
	~TouchPlanner();

	bool waiting_for_points;

	private:

	void init();
	void tfStaticCallback(const tf2_msgs::TFMessage);
	void padConvexHull(float pad, PointList2D &hull_points);
	PointList2D subdividePath(int num_subdivisions, const PointList2D hull_points);
	
	ros::Subscriber tf_static_subscriber_;
	std::unique_ptr<ros::NodeHandle> nh_;
	std::unique_ptr<ros::NodeHandle> pnh_;

	PointList2D mesh_points_2D;
	PointList3D mesh_points_3D;
};

float magnitude_calculator(Point2f v);
Point2f unit_vector_calculator(Point2f v);