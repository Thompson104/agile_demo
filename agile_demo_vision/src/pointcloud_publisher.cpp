#include "pointcloud_publisher.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <limits>

namespace agile_demo_vision {

PointCloudPublisher::PointCloudPublisher() : nh_{"~"} {
	ros::Rate publish_rate{0.01};
	pub_pointcloud_          = nh_.advertise<sensor_msgs::PointCloud2>("static_pointcloud", 10, true);
	pub_static_pointcloud_   = nh_.advertiseService("publish_point_cloud", &PointCloudPublisher::publishPointCloud, this);
	clear_static_pointcloud_ = nh_.advertiseService("clear_static_point_cloud", &PointCloudPublisher::clearStaticPointCloud, this);
	publish_timer_           = nh_.createTimer(publish_rate, &PointCloudPublisher::onPublishPointCloud, this);
}

void PointCloudPublisher::onPublishPointCloud(ros::TimerEvent const &) {
	//TODO
}

bool PointCloudPublisher::publishPointCloud(agile_demo_msgs::PublishPointCloud::Request & req, agile_demo_msgs::PublishPointCloud::Response & res) {
	//TODO
}

bool PointCloudPublisher::clearStaticPointCloud(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
	//TODO
}

}

int main(int argc, char * * argv) {
	ros::init(argc, argv, "point_cloud_publisher");
	agile_demo_vision::PointCloudPublisher pointcloud_publisher;
	ros::spin();
	return 0;
}
