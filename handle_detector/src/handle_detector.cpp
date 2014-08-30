#include "handle_detector/affordances.h"
#include "handle_detector/CylinderArrayMsg.h"
#include "handle_detector/CylinderMsg.h"
#include "handle_detector/HandleListMsg.h"
#include <ctype.h>
#include "handle_detector/cylindrical_shell.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include <iostream>
#include "handle_detector/messages.h"


//#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include "handle_detector/visualizer.h"
#define EIGEN_DONT_PARALLELIZE

#include <ros/ros.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

std::string OUTPUT_FRAME; // "/base_link"
//std::string OUTPUT_FRAME; // "/camera_rgb_optical_frame";
std::string RANGE_SENSOR_TOPIC; // "/camera/depth_registered/points";

boost::shared_ptr<tf::TransformListener> listener;

// input and output ROS topic data
PointCloud::Ptr g_cloud(new PointCloud);
Affordances g_affordances;
std::vector<CylindricalShell> g_cylindrical_shells;
std::vector< std::vector<CylindricalShell> > g_handles;

// synchronization
double g_prev_time;
double g_update_interval;
bool g_has_read = false;

void chatterCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
	if (omp_get_wtime() - g_prev_time < g_update_interval) { return; }

	// convert ROS sensor message to PCL point cloud
	PointCloud::Ptr cloud(new PointCloud);
	fromROSMsg(*input, *cloud);
	g_has_read = true;

	// check whether input frame is equivalent to range sensor frame constant
	std::string input_frame = input->header.frame_id;
	if (input_frame.compare(OUTPUT_FRAME) != 0) {
		std::cout << "Input frame and output frame are different\n";
		std::cout << "Transforming from " << input_frame << " to " << OUTPUT_FRAME << "\n";
		PointCloud::Ptr transformed_cloud(new PointCloud);
		tf::StampedTransform tf_transform;
		listener->waitForTransform(OUTPUT_FRAME, input_frame,
				ros::Time(0), ros::Duration(5));
		listener->lookupTransform(OUTPUT_FRAME, input_frame,
				ros::Time(0), tf_transform);

		Eigen::Affine3d transform;
		tf::transformTFToEigen(tf_transform, transform);

		pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
		cloud = transformed_cloud;
	} else {
		std::cout << "Input and output frame are: " << input_frame << "\n";
	}

	g_cloud = cloud;

	// search grasp affordances
	g_cylindrical_shells = g_affordances.searchAffordances(g_cloud);
	if (g_cylindrical_shells.size() == 0)
	{
		printf("No handles found!\n");
		g_prev_time = omp_get_wtime();
		return;
	}

	// search handles
	g_handles = g_affordances.searchHandles(g_cloud, g_cylindrical_shells);

	// store current time
	g_prev_time = omp_get_wtime();
}

int main(int argc, char** argv) {
	// initialize random seed
	srand (time(NULL));

	// initialize ROS
	ros::init(argc, argv, "handle_detector");
	ros::NodeHandle node("~");

	listener.reset(new tf::TransformListener(ros::Duration(20.0)));

	// set point cloud update interval from launch file
	node.param("update_interval", g_update_interval, 10.0);

	// read parameters
	g_affordances.initParams(node);

	std::string output_frame;
	ros::Subscriber sub;

	node.param("output_frame", OUTPUT_FRAME, std::string("/base_link"));
	node.param("camera_topic", RANGE_SENSOR_TOPIC, std::string("/camera/depth_registered/points"));

	// point cloud read from sensor
	printf("Reading point cloud data from sensor topic: %s\n", RANGE_SENSOR_TOPIC.c_str());
	output_frame = OUTPUT_FRAME;
	sub = node.subscribe(RANGE_SENSOR_TOPIC, 10, chatterCallback);

	// visualization of point cloud, grasp affordances, and handles
	Visualizer visualizer(1.0/g_update_interval);
	sensor_msgs::PointCloud2 pc2msg;
	PointCloud::Ptr cloud_vis(new PointCloud);
	ros::Publisher marker_array_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_all_affordances", 10);
	ros::Publisher marker_array_pub_handles = node.advertise<visualization_msgs::MarkerArray>("visualization_all_handles", 10);
	ros::Publisher marker_array_pub_handle_numbers = node.advertise<visualization_msgs::MarkerArray>("visualization_handle_numbers", 10);
	std::vector<visualization_msgs::MarkerArray> marker_arrays;
	visualization_msgs::MarkerArray marker_array_msg;
	visualization_msgs::MarkerArray marker_array_msg_handles;
	visualization_msgs::MarkerArray marker_array_msg_handle_numbers;

	// publication of grasp affordances and handles as ROS topics
	Messages messages;
	ros::Publisher cylinder_pub = node.advertise<handle_detector::CylinderArrayMsg>("cylinder_list", 10);
	ros::Publisher handles_pub = node.advertise<handle_detector::HandleListMsg>("handle_list", 10);
	ros::Publisher pcl_pub = node.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
	std::vector<ros::Publisher> handle_pubs;
	handle_detector::CylinderArrayMsg cylinder_list_msg;
	handle_detector::HandleListMsg handle_list_msg;

	// how often things are published
	ros::Rate rate(10);

	double prev_time = omp_get_wtime();

	while (ros::ok())
	{
		if (g_has_read)
		{
			// create visual point cloud
			cloud_vis = g_affordances.workspaceFilter(g_cloud);
			ROS_INFO("update cloud");

			// create cylinder messages for visualization and ROS topic
			marker_array_msg = visualizer.createCylinders(g_cylindrical_shells, output_frame);
			cylinder_list_msg = messages.createCylinderArray(g_cylindrical_shells, output_frame);
			ROS_INFO("update visualization");

			// create handle messages for visualization and ROS topic
			handle_list_msg = messages.createHandleList(g_handles, output_frame);
			visualizer.createHandles(g_handles, output_frame, marker_arrays,
					marker_array_msg_handles);
			handle_pubs.resize(g_handles.size());
			for (int i=0; i < handle_pubs.size(); i++)
				handle_pubs[i] = node.advertise<visualization_msgs::MarkerArray>("visualization_handle_" + boost::lexical_cast<std::string>(i), 10);

			marker_array_msg_handle_numbers = visualizer.createHandleNumbers(g_handles, output_frame);

			ROS_INFO("update messages");

			g_has_read = false;

			// publish cylinders as ROS topic
			cylinder_pub.publish(cylinder_list_msg);

			// publish handles as ROS topic
			handles_pub.publish(handle_list_msg);

			// publish cylinders for visualization
			marker_array_pub.publish(marker_array_msg);

			// publish handles for visualization
			for (int i=0; i < handle_pubs.size(); i++)
				handle_pubs[i].publish(marker_arrays[i]);

			// publish handles for visualization
			marker_array_pub_handles.publish(marker_array_msg_handles);

			// publish handle numbers for visualization
			marker_array_pub_handle_numbers.publish(marker_array_msg_handle_numbers);
		}

		// publish point cloud
		toROSMsg(*cloud_vis, pc2msg);
		pc2msg.header.stamp = ros::Time::now();
		pc2msg.header.frame_id = output_frame;
		pcl_pub.publish(pc2msg);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
