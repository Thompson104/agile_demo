#include "coordinator.hpp"

#include <ros/package.h>
#include <agile_demo_msgs/PublishPointCloud.h>
#include <dr_eigen/ros.hpp>

namespace {
	/// Find model path from url.
	/// Code from http://docs.ros.org/jade/api/resource_retriever/html/retriever_8cpp_source.html
	std::string resolvePackagePath(const std::string & url) {
		std::string mod_url = url;
		if (url.find("package://") == 0) {
			mod_url.erase(0, strlen("package://"));
			size_t pos = mod_url.find("/");
			if (pos == std::string::npos) {
				std::cout << "Could not parse package:// format into file:// format\n";
				return url;
			}

			std::string package = mod_url.substr(0, pos);
			mod_url.erase(0, pos);
			std::string package_path = ros::package::getPath(package);

			if (package_path.empty()) {
				std::cout << "Package [" + package + "] does not exist \n";
				return url;
			}

			mod_url = package_path + mod_url;
			return mod_url;
		}

		return url;
	}
}

namespace agile_demo {
namespace core {

Coordinator::Coordinator() try :
	node_{"~"},
	grasp_planner_{node_},
	motion_planner_{node_}
{
	vision_client_  = node_.serviceClient<agile_demo_msgs::PublishPointCloud>("/pointcloud_publisher/publish_point_cloud", true);
	command_server_ = node_.advertiseService("command", &Coordinator::execute, this);

	if (!vision_client_.waitForExistence(ros::Duration(5.0))) ROS_ERROR_STREAM("Vision service is not running.");

	agile_demo_msgs::PublishPointCloud srv;
	srv.request.pcd_path = resolvePackagePath("package://agile_demo_vision/data/easter_turtle_sippy_cup.pcd");
	srv.request.pose     = dr::toRosPose(Eigen::Translation3d{-0.070, 0.470, 0.0} * Eigen::Quaterniond::Identity());

	if (!vision_client_.call(srv)) {
		ROS_ERROR_STREAM("Failed to retrieve vision data.");
	}

	ROS_INFO_STREAM("Coordinator started.");
} catch (std::exception const & e) {
	ROS_ERROR_STREAM(e.what());
}

bool Coordinator::execute(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
	boost::optional<control_msgs::FollowJointTrajectoryGoal> action;

	boost::optional<Eigen::Isometry3d> grasp = grasp_planner_.findGrasp();
	if (!grasp) {
		ROS_WARN_STREAM("Grasp vector is empty.");
		return false;
	}

	if (!motion_planner_.cartToAction(*grasp, action)) {
		ROS_WARN_STREAM("No inverse kinematics solution, please try next grasp or place the object within reach.");
		return false;
	}

	if (!motion_planner_.moveToGoal(*action, 2.0)) {
		ROS_WARN_STREAM("Failed to move to goal.");
		return false;
	}

	return true;
}

}}

int main(int argc, char** argv) {
	ros::init(argc, argv, "coordinator");
	ros::NodeHandle node_handle{"~"};

	agile_demo::core::Coordinator coordinator;

	ros::spin();
	return 0;

}
