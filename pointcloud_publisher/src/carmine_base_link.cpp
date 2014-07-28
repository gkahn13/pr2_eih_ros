#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions.h>
#include <pcl_ros/transforms.h>
// PCL specific includes
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

ros::Publisher pub;
tf::TransformListener *listener;
using namespace pcl;

void cloud_cb (const PointCloud<PointXYZRGB>::ConstPtr& input) {
  // pcl -> PointCloud2
  sensor_msgs::PointCloud2 pc2;
  toROSMsg(*input, pc2);
  pc2.header.frame_id = "/camera_rgb_optical_frame";
  pc2.header.stamp = ros::Time::now();
  
  // to /base_link
  sensor_msgs::PointCloud2 pc2_base_link;
  std::string base_link = "/base_link";
  pcl_ros::transformPointCloud(base_link, pc2, pc2_base_link, *listener);
  
  pub.publish(pc2_base_link);
}


int main (int argc, char** argv) {
  // Initialize ROS
  
  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud

  ros::init (argc, argv, "camera");
  listener = new tf::TransformListener();
  ros::NodeHandle nh("~");
  
  std::string dev;
  if (!nh.getParam("device_number", dev)) {
    dev = "1";
  }
  std::string pointcloud_topic;
  if (!nh.getParam("pointcloud_topic", pointcloud_topic)) {
    //pointcloud_topic = "/hand_kinect_points";
    pointcloud_topic = "depth_registered/points";
  }
  std::cout << "pointcloud_topic: " << pointcloud_topic << "\n";
  Grabber* grabber = new OpenNIGrabber("#" + dev);
  pub = nh.advertise<sensor_msgs::PointCloud2> (pointcloud_topic, 1);
  boost::function<void (const PointCloud<PointXYZRGB>::ConstPtr&)> f =
    boost::bind(&cloud_cb, _1);
  grabber->registerCallback(f);
  grabber->start();


  // Spin
  ros::spin ();
  grabber->stop();
}
