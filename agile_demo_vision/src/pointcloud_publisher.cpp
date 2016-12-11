#include "pointcloud_publisher.hpp"

#include <cmath>
#include <dr_eigen/ros.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>

namespace agile_demo_vision {

PointCloudPublisher::PointCloudPublisher() : nh_{"~"} {
	ros::Rate publish_rate{0.01};
	pointcloud_pub_         = nh_.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 10, true);
	pub_pointcloud_server_  = nh_.advertiseService("publish_point_cloud", &PointCloudPublisher::publishPointCloud, this);
	publish_timer_          = nh_.createTimer(publish_rate, &PointCloudPublisher::onPublishPointCloud, this);

	ROS_INFO_STREAM("Point cloud publisher initialised!");
}

void PointCloudPublisher::onPublishPointCloud(ros::TimerEvent const &) {
	ros::Time now = ros::Time::now();
	point_cloud_.header.stamp = now;
	pointcloud_pub_.publish(point_cloud_);
}

bool PointCloudPublisher::publishPointCloud(agile_demo_msgs::PublishPointCloud::Request & req, agile_demo_msgs::PublishPointCloud::Response &) {
	// Read model
	PointCloud::Ptr model = boost::make_shared<PointCloud>();
	if (pcl::io::loadPCDFile(req.pcd_path.data, *model) == -1) {
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

	pcl::transformPointCloud(*model, transformed_point_cloud, affine_parent_frame);
	pcl::toROSMsg(transformed_point_cloud, point_cloud_);
	point_cloud_.header.frame_id = "world";

	ros::TimerEvent event;
	onPublishPointCloud(event);

	return true;
}

}

int main(int argc, char * * argv) {
	ros::init(argc, argv, "point_cloud_publisher");
	agile_demo_vision::PointCloudPublisher pointcloud_publisher;
	ros::spin();
	return 0;
}
