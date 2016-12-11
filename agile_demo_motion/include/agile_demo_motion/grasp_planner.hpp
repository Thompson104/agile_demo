#pragma once

#include <agile_grasp/Grasps.h>
#include <boost/optional.hpp>
#include <ros/ros.h>
#include <Eigen/Geometry>

namespace agile_demo {
namespace motion {

class GraspPlanner {
protected:
	/// Subscriber to agile_grasp.
	ros::Subscriber agile_sub;

	/// Pubisher to visualize the chosen grasps.
	ros::Publisher grasp_pub;

	/// The grasps found by agile_grasp.
	boost::optional<agile_grasp::Grasps> found_grasps = boost::none;

public:
	/// Constructor.
	GraspPlanner(ros::NodeHandle & node);

	/// Destructor.
	~GraspPlanner() {};

	/// Find a grasp pose to move to.
	boost::optional<Eigen::Isometry3d> findGrasp();

protected:
	void agileGraspCallback(agile_grasp::Grasps const & grasps);
};

}}
