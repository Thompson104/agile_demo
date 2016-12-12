#include "grasp_planner.hpp"

#include <dr_eigen/eigen.hpp>
#include <dr_eigen/ros.hpp>
#include <dr_param/param.hpp>

namespace {
	Eigen::Isometry3d toEigen(agile_grasp::Grasp const & grasp) {
		Eigen::Matrix3d orientation = Eigen::Matrix3d::Zero(3, 3);
		orientation.col(0)          = -1.0 * dr::toEigen(grasp.approach);
		orientation.col(1)          = dr::toEigen(grasp.axis);
		orientation.col(2)          = orientation.col(0).cross(orientation.col(1));

		return dr::translate(dr::toEigen(grasp.center)) * Eigen::Quaterniond{orientation};
	}
}

namespace agile_demo {
namespace motion {

GraspPlanner::GraspPlanner(ros::NodeHandle & node) {
	agile_sub = node.subscribe("/find_grasps/grasps", 1, &GraspPlanner::agileGraspCallback, this);
	grasp_pub = node.advertise<geometry_msgs::PoseStamped>("/visualize_grasps", 1, true);
	ROS_INFO_STREAM("Grasp planner is initialised!");
}

boost::optional<Eigen::Isometry3d> GraspPlanner::findGrasp() {
	if (!found_grasps) return boost::none;
	if (found_grasps->grasps.empty()) {
		ROS_WARN_STREAM("Exhausted all grasps! Please relaunch the demo.");
		return boost::none;
	}

	Eigen::Isometry3d result = toEigen(found_grasps->grasps.back());
	geometry_msgs::PoseStamped ros_pose = dr::toRosPoseStamped(result, "world", ros::Time::now());
	grasp_pub.publish(ros_pose);

	found_grasps->grasps.pop_back();
	return result;
}

void GraspPlanner::agileGraspCallback(agile_grasp::Grasps const & grasps) {
	if (!found_grasps)
		found_grasps = grasps;
}

}}
