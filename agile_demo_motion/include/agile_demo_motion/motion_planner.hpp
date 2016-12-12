#pragma once

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trac_ik/trac_ik.hpp>

namespace agile_demo {
namespace motion {

using JointTrajectoryClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

class MotionPlanner {
protected:
	/// Action client for sending joint trajectories to follow.
	JointTrajectoryClient joint_action_client;

	/// Subscriber for current joint state.
	ros::Subscriber joint_state_sub;

	/// The current joint state.
	boost::optional<sensor_msgs::JointState> current_joint_state = boost::none;

	/// The inverse kinematics solver.
	TRAC_IK::TRAC_IK tracik_solver;

	/// The KDL chain.
	KDL::Chain kdl_chain;

	/// Names of the joints.
	std::vector<std::string> joint_names = {{
		"shoulder_pan_joint"  ,
		"shoulder_lift_joint" ,
		"elbow_joint"         ,
		"wrist_1_joint"       ,
		"wrist_2_joint"       ,
		"wrist_3_joint"
	}};

public:
	/// Constructor.
	MotionPlanner(ros::NodeHandle & node);

	/// Destructor.
	~MotionPlanner() {};

	/// Move robot with a FollowJointTrajectoryGoal command.
	bool moveToGoal(control_msgs::FollowJointTrajectoryGoal const & goal, double timeout = 2.0);

	/// Move robot with a tool pose command.
	bool cartToAction(Eigen::Isometry3d const & goal, boost::optional<control_msgs::FollowJointTrajectoryGoal> & action);

protected:
	void jointStateCallback(sensor_msgs::JointState const & joint_state);

};

}}
