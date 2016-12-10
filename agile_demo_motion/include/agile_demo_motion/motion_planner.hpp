#pragma once

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>

namespace agile_demo_motion {

class MotionPlanner {
private:
	/// The ros node handle.
	ros::NodeHandle node_handle_;

	/// Action client for sending joint trajectories to follow.
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;

public:
	/// Constructor.
	MotionPlanner();

	/// Destructor.
	~MotionPlanner();
};

}
