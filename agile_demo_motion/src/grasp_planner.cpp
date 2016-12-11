#include "grasp_planner.hpp"

#include <dr_eigen/eigen.hpp>
#include <dr_eigen/ros.hpp>
#include <dr_param/param.hpp>

namespace agile_demo {
namespace motion {

GraspPlanner::GraspPlanner(ros::NodeHandle & node) {
	agile_sub = node.subscribe("/find_grasps/grasps", 1, &GraspPlanner::agileGraspCallback, this);
	grasp_pub = node.advertise<geometry_msgs::PoseStamped>("/visualize_grasps", 1, true);
	ROS_INFO_STREAM("Grasp planner is initialised!");
}

boost::optional<Eigen::Isometry3d> GraspPlanner::findGrasp() {
	while (!found_grasps) {
		ROS_WARN_THROTTLE(5, "No grasp detected yet.");
		ros::spinOnce();
	}

	agile_grasp::Grasp chosen_grasp;
	for (auto const grasp : found_grasps->grasps) {
		if (dr::toEigen(grasp.approach).z() > 0) {
			chosen_grasp = grasp;
			break;
		}
	}

	Eigen::Matrix3d orientation = Eigen::Matrix3d::Zero(3, 3);
	orientation.col(0)          = -1.0 * dr::toEigen(chosen_grasp.approach);
	orientation.col(1)          = dr::toEigen(chosen_grasp.axis);
	orientation.col(2)          = orientation.col(0).cross(orientation.col(1));

	Eigen::Isometry3d result            = dr::translate(dr::toEigen(chosen_grasp.center)) * Eigen::Quaterniond{orientation};
	geometry_msgs::PoseStamped ros_pose = dr::toRosPoseStamped(result, "world", ros::Time::now());
	grasp_pub.publish(ros_pose);

	found_grasps = boost::none;
	return result;
}

void GraspPlanner::agileGraspCallback(agile_grasp::Grasps const & grasps) {
	found_grasps = grasps;
}

}}
