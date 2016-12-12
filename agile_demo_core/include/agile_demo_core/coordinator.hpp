#pragma once

#include <ros/ros.h>
#include <agile_demo_motion/grasp_planner.hpp>
#include <agile_demo_motion/motion_planner.hpp>
#include <std_srvs/Empty.h>

namespace agile_demo {
namespace core {

class Coordinator {
private:
	/// ROS node handle.
	ros::NodeHandle node_;

	/// Grasp planner.
	motion::GraspPlanner grasp_planner_;

	/// Motion planner.
	motion::MotionPlanner motion_planner_;

	/// Vision service client.
	ros::ServiceClient vision_client_;

	/// Command server.
	ros::ServiceServer command_server_;

public:
	/// Constructor.
	Coordinator();

	/// Destructor.
	~Coordinator() {};

private:
	/// Execute a command.
	bool execute(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
};

}}
