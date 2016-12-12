#include "motion_planner.hpp"
#include "util.hpp"

#include <dr_eigen/eigen.hpp>
#include <dr_param/param.hpp>
#include <eigen_conversions/eigen_kdl.h>

namespace agile_demo {
namespace motion {

MotionPlanner::MotionPlanner(ros::NodeHandle & node) :
	joint_action_client{node, "/pos_based_pos_traj_controller/follow_joint_trajectory", true},
	tracik_solver{
		dr::getParam<std::string>(node, "chain_start", "base_link"),
		dr::getParam<std::string>(node, "chain_end", "ee_link"),
		dr::getParam<std::string>(node, "urdf_param", "/robot_description"),
		dr::getParam<double>(node, "timeout", 2.0),
		dr::getParam<double>(node, "eps", 1e-5),
		TRAC_IK::SolveType::Distance
	}
{
	if (!tracik_solver.getKDLChain(kdl_chain)) {
		throw std::runtime_error("There was no valid KDL chain found.");
	}

	joint_names       = dr::getParam<std::vector<std::string>>(node, "/pos_based_pos_traj_controller/joints", joint_names);
	joint_state_sub   = node.subscribe("/joint_states", 1, &MotionPlanner::jointStateCallback, this);
	joint_action_client.waitForServer();

	ROS_INFO_STREAM("Motion planner is initialised!");
}

bool MotionPlanner::moveToGoal(control_msgs::FollowJointTrajectoryGoal const & goal, double timeout) {
	joint_action_client.sendGoal(goal);
	joint_action_client.waitForResult(ros::Duration(timeout));
	if (joint_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO_STREAM("Successfully executed trajectory.");
		return true;
	} else {
		ROS_ERROR_STREAM("Failed to execute trajectory, status is " << joint_action_client.getState().toString());
		return false;
	}
}

bool MotionPlanner::cartToAction(Eigen::Isometry3d const & goal, boost::optional<control_msgs::FollowJointTrajectoryGoal> & action) {
	KDL::Frame kdl_goal;
	tf::transformEigenToKDL(goal, kdl_goal);
	action = boost::none;

	while (!current_joint_state) {
		ROS_WARN_THROTTLE(5, "Still waiting for joint state message.");
		ros::spinOnce();
	}

	KDL::JntArray result(kdl_chain.getNrOfJoints());
	KDL::JntArray kdl_current_joint_state = toKDLJoints(current_joint_state->position);
	int status = tracik_solver.CartToJnt(kdl_current_joint_state, kdl_goal, result);
	if (status < 0) {
		ROS_ERROR_STREAM("Inverse kinematics failed, status is " << status);
		return false;
	}

	try {
		std::vector<double> joint_pos = fromKDLJoints(result);
		action = constructGoal(
			joint_pos,
			{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}},
			joint_names,
			0.5
		);
	} catch (std::runtime_error const & e) {
		ROS_ERROR_STREAM(e.what());
		return false;
	}

	return true;
}

void MotionPlanner::jointStateCallback(sensor_msgs::JointState const & joint_state) {
	current_joint_state = joint_state;
}

}}
