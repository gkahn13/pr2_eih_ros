#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL specific includes
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

ros::Publisher pub;
using namespace pcl;

void cloud_cb (const PointCloud<PointXYZRGB>::ConstPtr& input) {
	// ... do data processing

	//sensor_msgs::PointCloud2ConstPtr&
	//PointCloud<PointXYZRGB> cloud = input;
	sensor_msgs::PointCloud2 output;
	//sensor_msgs::Image output;
	toROSMsg(*input, output);
	output.header.stamp = ros::Time::now();
	output.header.frame_id = "/camera_rgb_optical_frame";
	// Publish the data
	pub.publish (output);
}


int main (int argc, char** argv) {
	// Initialize ROS

	// Create a ROS subscriber for the input point cloud
	//ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	ros::init (argc, argv, "camera");
	ros::NodeHandle nh("~");
	std::cout << "Initialized ROS node\n";
	std::string dev;
	if (!nh.getParam("device_number", dev)) {
		dev = "1";
	}
	std::cout << "Device number: " << dev << "\n";
	std::string pointcloud_topic;
	if (!nh.getParam("pointcloud_topic", pointcloud_topic)) {
		pointcloud_topic = "/hand_kinect_points";
		//pointcloud_topic = "depth_registered/points";
	}
	std::cout << "pointcloud_topic: " << pointcloud_topic << "\n";
	Grabber* grabber = new OpenNIGrabber("#" + dev);
	pub = nh.advertise<sensor_msgs::PointCloud2> (pointcloud_topic, 1);
	boost::function<void (const PointCloud<PointXYZRGB>::ConstPtr&)> f =
			boost::bind(&cloud_cb, _1);
	grabber->registerCallback(f);
	grabber->start();
	std::cout << "Started openni callback\n";


	// Spin
	ros::spin ();
	grabber->stop();
}
