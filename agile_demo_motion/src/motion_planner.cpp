//#include "motion_planner.hpp"

#include <dr_eigen/eigen.hpp>
#include <dr_param/param.hpp>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>

#include <eigen_conversions/eigen_kdl.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

//namespace agile_demo_motion {

using Client = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

//MotionPlanner::MotionPlanner() :
//	node_handle_{"~"},
//	action_client_{node_handle_, "follow_joint_trajectory", true}
//{
//	action_client_.waitForServer();
//	ROS_INFO_STREAM("Motion planner is initialised!");
//}

//}

int main(int argc, char** argv) {
	ros::init(argc, argv, "motion_planner");
	ros::NodeHandle node_handle{"~"};

	TRAC_IK::TRAC_IK tracik_solver(
		dr::getParam<std::string>(node_handle, "chain_start", "base_link"),
		dr::getParam<std::string>(node_handle, "chain_end", "ee_link"),
		dr::getParam<std::string>(node_handle, "urdf_param", "/robot_description"),
		dr::getParam<double>(node_handle, "timeout", 2.0),
		1e-5,
		TRAC_IK::SolveType::Distance
	);

	KDL::Chain chain;
	KDL::JntArray ll, ul; //lower joint limits, upper joint limits

	bool valid = tracik_solver.getKDLChain(chain);
	if (!valid) {
		ROS_ERROR("There was no valid KDL chain found");
		return 0;
	}

	valid = tracik_solver.getKDLLimits(ll,ul);

	if (!valid) {
		ROS_ERROR("There were no valid KDL joint limits found");
		return 0;
	}

	// Create Nominal chain configuration midway between all joint limits
	KDL::JntArray nominal(chain.getNrOfJoints());

	nominal(0) = -2.2029998938189905;
	nominal(1) = -1.7271001974688929;
	nominal(2) = -1.6007002035724085;
	nominal(3) = -0.8079999128924769;
	nominal(4) = 1.5951000452041626;
	nominal(5) = -0.03099996248354131;

	std::vector<KDL::JntArray> JointList;
	KDL::JntArray result(chain.getNrOfJoints());
	Eigen::Isometry3d eigen_pose = Eigen::Translation3d{0.162, 0.334, 0.128} * Eigen::Quaterniond(0.487, -0.500, 0.512, 0.500);
	KDL::Frame end_effector_pose;
	tf::transformEigenToKDL(eigen_pose, end_effector_pose);

	int rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);

	ROS_ERROR_STREAM(rc);

	for (size_t i = 0; i < result.rows(); i++) {
		ROS_INFO_STREAM("Result: " << result.data(i));
	}

	Client client("/pos_based_pos_traj_controller/follow_joint_trajectory", true);
	client.waitForServer();
	control_msgs::FollowJointTrajectoryGoal goal;

	// Fill in goal here
	control_msgs::JointTolerance tolerance;
	tolerance.position     = 0.0;
	tolerance.velocity     = 0.0;
	tolerance.acceleration = 0.0;

	for (size_t i = 0; i < 6; i++) {
		goal.path_tolerance.push_back(tolerance);
		goal.goal_tolerance.push_back(tolerance);
	}

	goal.goal_time_tolerance = ros::Duration(5.0);

	goal.trajectory.joint_names = {{
		"shoulder_pan_joint"  ,
		"shoulder_lift_joint" ,
		"elbow_joint"         ,
		"wrist_1_joint"       ,
		"wrist_2_joint"       ,
		"wrist_3_joint"
	}};

	trajectory_msgs::JointTrajectoryPoint joint_waypoint;
	joint_waypoint.positions     = {{result.data(0), result.data(1), result.data(2), result.data(3), result.data(4), result.data(5)}};
	joint_waypoint.velocities    = {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
	joint_waypoint.accelerations = {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
	joint_waypoint.time_from_start = ros::Duration(1.0);

	goal.trajectory.points.push_back(joint_waypoint);

	client.sendGoal(goal);
	client.waitForResult(ros::Duration(10.0));
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Yay! The robot moved");
	printf("Current State: %s\n", client.getState().toString().c_str());
	return 0;

}
