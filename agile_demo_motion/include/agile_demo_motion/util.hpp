#pragma once

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <kdl/jntarray.hpp>

namespace {
	/// Convert std vector to KDL::JntArray.
	inline KDL::JntArray toKDLJoints(std::vector<double> const & joint_values) {
		KDL::JntArray kdl_joints(joint_values.size());
		for (size_t i = 0; i < kdl_joints.rows(); i++) {
			kdl_joints(i) = joint_values.at(i);
		}

		return kdl_joints;
	}

	/// Convert KDL::JntArray to std vector.
	inline std::vector<double> fromKDLJoints(KDL::JntArray const & kdl_array) {
		std::vector<double> joint_values;
		joint_values.resize(kdl_array.rows());
		for (size_t i = 0; i < joint_values.size(); i++) {
			joint_values.at(i) = kdl_array(i);
		}

		return joint_values;
	}

	/// Convert std vector to Eigen::VectorXd.
	inline Eigen::VectorXd toEigen(std::vector<double> const & values) {
		Eigen::VectorXd vector(values.size());
		for (int i = 0; i < vector.rows(); i++) {
			vector(i) = values.at(i);
		}
		return vector;
	}

	/// Convert Eigen::VectorXd to std::vector.
	inline std::vector<double> fromEigen(Eigen::VectorXd const & vector) {
		std::vector<double> values;
		values.resize(vector.rows());
		for (size_t i = 0; i < values.size(); i++) {
			values.at(i) = vector(i);
		}
		return values;
	}

	/// Construct an action goal.
	inline control_msgs::FollowJointTrajectoryGoal constructGoal(
		std::vector<double> const & joint_pos,
		std::vector<double> const & joint_vel,
		std::vector<std::string> const & joint_names,
		double time_from_start
	) {
		if (joint_pos.size() != joint_vel.size() && joint_pos.size()!= joint_names.size()) {
			throw std::runtime_error("Joint position sizes don't match with joint velocities and names.");
		}

		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory.joint_names = joint_names;
		trajectory_msgs::JointTrajectoryPoint joint_waypoint;
		joint_waypoint.positions  = joint_pos;
		joint_waypoint.velocities = joint_vel;
		joint_waypoint.time_from_start = ros::Duration{time_from_start};

		goal.trajectory.points.push_back(joint_waypoint);
		return goal;
	}

	double distance(Eigen::VectorXd const & a, Eigen::VectorXd const & b) {
		if (a.rows() != b.rows()) throw std::runtime_error{"Failed to calculate joint distance due to unmatching sizes."};
		return (a - b).norm();
	}

	double distance(std::vector<double> const & a, std::vector<double> const & b) {
		return distance(toEigen(a), toEigen(b));
	}
}

