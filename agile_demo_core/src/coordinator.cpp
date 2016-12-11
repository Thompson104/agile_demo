#include "coordinator.hpp"

#include <agile_demo_msgs/PublishPointCloud.h>
#include <dr_eigen/ros.hpp>

namespace agile_demo {
namespace core {

Coordinator::Coordinator() try :
	node_{"~"},
	grasp_planner_{node_},
	motion_planner_{node_}
{
	vision_client_ = node_.serviceClient<agile_demo_msgs::PublishPointCloud>("/pointcloud_publisher/publish_point_cloud", true);

	if (!vision_client_.waitForExistence(ros::Duration(5.0))) ROS_ERROR_STREAM("Vision service is not running.");

	Eigen::Isometry3d pose = Eigen::Translation3d{0.20, 0.30, 0.1} * Eigen::Quaterniond{0.0, 0.0, 0.0, 1.0};
	agile_demo_msgs::PublishPointCloud srv;
	srv.request.pose          = dr::toRosPose(pose);
	srv.request.pcd_path = "/home/wko/dev/demo_workspace/src/agile_demo/agile_demo_vision/data/easter_turtle_sippy_cup.pcd";

	vision_client_.call(srv);
	Eigen::Isometry3d grasp = *grasp_planner_.findGrasp();
	motion_planner_.moveToGoal(grasp);

} catch (std::exception const & e) {
	ROS_ERROR_STREAM(e.what());
}

}}

int main(int argc, char** argv) {
	ros::init(argc, argv, "coordinator");
	ros::NodeHandle node_handle{"~"};

	agile_demo::core::Coordinator coordinator;

	ros::spin();
	return 0;

}
