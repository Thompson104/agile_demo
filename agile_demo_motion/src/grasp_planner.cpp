#include "grasp_planner.hpp"

#include <dr_eigen/eigen.hpp>
#include <dr_eigen/ros.hpp>
#include <dr_param/param.hpp>

namespace agile_demo {
namespace motion {

GraspPlanner::GraspPlanner(ros::NodeHandle & node) {
	agile_sub = node.subscribe("/find_grasps/grasps", 1, &GraspPlanner::agileGraspCallback, this);
	ROS_INFO_STREAM("Grasp planner is initialised!");
}

boost::optional<Eigen::Isometry3d> GraspPlanner::findGrasp() {
	while (!found_grasps) {
		ROS_WARN_THROTTLE(5, "No grasp detected yet.");
		ros::spinOnce();
	}

	agile_grasp::Grasp grasp = found_grasps->grasps.front();
	Eigen::Matrix3d orientation = Eigen::Matrix3d::Zero(3, 3);
	orientation.col(0) = -1.0 * dr::toEigen(grasp.approach);
	orientation.col(1) = dr::toEigen(grasp.axis);
	orientation.col(2) = orientation.col(0).cross(orientation.col(1));

	found_grasps = boost::none;
	return dr::translate(dr::toEigen(grasp.center)) * Eigen::Quaterniond{orientation};
}

void GraspPlanner::agileGraspCallback(agile_grasp::Grasps const & grasps) {
	found_grasps = grasps;
}

}}

//int main(int argc, char** argv) {
//	ros::init(argc, argv, "grasp_planner");
//	ros::NodeHandle node_handle{"~"};
//
//	ros::Publisher grasp_pub = node_handle.advertise<geometry_msgs::PoseStamped>("found_pose", 1, true);
//	agile_demo_motion::GraspPlanner grasp_planner{node_handle};
//	Eigen::Isometry3d pose         = *grasp_planner.findGrasp();
//	geometry_msgs::PoseStamped msg = dr::toRosPoseStamped(pose, "world", ros::Time::now());
//
//	grasp_pub.publish(msg);
//
//	ros::spin();
//	return 0;
//
//}
