#pragma once

#include <agile_demo_msgs/PublishPointCloud.h>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>

namespace agile_demo {
namespace vision {

using Point      = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

class PointCloudPublisher {
private:
	/// Node handle.
	ros::NodeHandle node_;

	/// The pointcloud to publish.
	sensor_msgs::PointCloud2 point_cloud_;

	/// Ros publisher for pointclouds.
	ros::Publisher pointcloud_pub_;

	/// Ros server for publishing a pointcloud.
	ros::ServiceServer pub_pointcloud_server_;

	/// Time to periodically publish a pointcloud.
	ros::Timer publish_timer_;

public:
	PointCloudPublisher();

	/// Publish a point cloud from file.
	bool publishPointCloud(agile_demo_msgs::PublishPointCloud::Request & req, agile_demo_msgs::PublishPointCloud::Response &);

private:
	/// Callback for timer.
	void onPublishPointCloud(ros::TimerEvent const &);
};

}}
