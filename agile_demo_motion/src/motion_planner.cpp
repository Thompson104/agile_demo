//#include "motion_planner.hpp"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>

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
	ros::init(argc, argv, "joint_trajectory_client");
	Client client("/pos_based_pos_traj_controller/follow_joint_trajectory", true);
	client.waitForServer();
	control_msgs::FollowJointTrajectoryGoal goal;

	// Fill in goal here
	control_msgs::JointTolerance tolerance;
	tolerance.position     = 0.0872665;
	tolerance.velocity     = 2*0.0872665;
	tolerance.acceleration = 2*0.0872665;

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
	joint_waypoint.positions     = {{M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2}};
	joint_waypoint.velocities    = {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
	joint_waypoint.accelerations = {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
	joint_waypoint.time_from_start = ros::Duration(0.5);

	goal.trajectory.points.push_back(joint_waypoint);

	client.sendGoal(goal);
	client.waitForResult(ros::Duration(10.0));
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Yay! The robot moved");
	printf("Current State: %s\n", client.getState().toString().c_str());
	return 0;

}
