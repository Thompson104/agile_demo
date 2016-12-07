#pragma once

#include <agile_demo_msgs/PublishPointCloud.h>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>

namespace agile_demo_vision {

class PointCloudPublisher {
private:
	/// Node handle.
	ros::NodeHandle nh_;

	sensor_msgs::PointCloud2 point_cloud_;
	sensor_msgs::PointCloud2 clear_cloud_;

	pcl::PointCloud<pcl::PointXYZ> pcl_clear_cloud_;
	bool clear_is_true_;

	ros::Publisher pub_pointcloud_;
	ros::ServiceServer pub_static_pointcloud_;
	ros::ServiceServer clear_static_pointcloud_;

	ros::Timer publish_timer_;

public:
	PointCloudPublisher();

	void onPublishPointCloud(ros::TimerEvent const &);
	void onClearPointCloud(ros::TimerEvent const &);
	// Publish a cropped static point cloud that is provided by the coordinator.
	bool publishPointCloud(agile_demo_msgs::PublishPointCloud::Request & req, agile_demo_msgs::PublishPointCloud::Response & res);
	bool clearStaticPointCloud(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
};
}
