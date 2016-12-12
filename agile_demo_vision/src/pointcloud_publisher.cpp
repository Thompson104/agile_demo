#include "pointcloud_publisher.hpp"

#include <cmath>
#include <dr_eigen/ros.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>

namespace agile_demo {
namespace vision {

PointCloudPublisher::PointCloudPublisher() : node_{"~"} {
	ros::Rate publish_rate{0.01};
	pointcloud_pub_         = node_.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 10, true);
	pub_pointcloud_server_  = node_.advertiseService("publish_point_cloud", &PointCloudPublisher::publishPointCloud, this);

	ROS_INFO_STREAM("Point cloud publisher initialised!");
}

bool PointCloudPublisher::publishPointCloud(agile_demo_msgs::PublishPointCloud::Request & req, agile_demo_msgs::PublishPointCloud::Response &) {
	// Read model
	PointCloud::Ptr model = boost::make_shared<PointCloud>();
	if (pcl::io::loadPCDFile(req.pcd_path, *model) == -1) {
		ROS_ERROR_STREAM("No model found. Model path: " << req.pcd_path);
		return false;
	}

	// Check for empty model
	if (model->empty()) {
		ROS_ERROR_STREAM("Model point cloud is empty.");
		return false;
	}

	// Publish
	pcl::PointCloud<pcl::PointXYZ> transformed_point_cloud;

	Eigen::Isometry3d parent_frame = dr::toEigen(req.pose);
	Eigen::Affine3d affine_parent_frame{parent_frame.matrix()};

	/// The pointcloud to publish.
	sensor_msgs::PointCloud2 point_cloud;
	pcl::transformPointCloud(*model, transformed_point_cloud, affine_parent_frame);
	pcl::toROSMsg(transformed_point_cloud, point_cloud);

	point_cloud.header.stamp    = ros::Time::now();
	point_cloud.header.frame_id = "world";
	pointcloud_pub_.publish(point_cloud);

	return true;
}

}}

int main(int argc, char * * argv) {
	ros::init(argc, argv, "point_cloud_publisher");
	agile_demo::vision::PointCloudPublisher pointcloud_publisher;
	ros::spin();
	return 0;
}
