#ifndef __VOXEL_GRID_H__
#define __VOXEL_GRID_H__

//#include "pr2-sim.h"
//#include "../utils/rave-utils.h"
//#include "../../util/logging.h"

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include "sensor_msgs/PointCloud2.h"

#include <pcl_conversions.h>
// PCL specific includes
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Eigen>
using namespace Eigen;

#include <openrave-core.h>
namespace rave = OpenRAVE;

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/heap/fibonacci_heap.hpp>

#include <pcl/console/parse.h>
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/gpu/kinfu_large_scale/tsdf_volume.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/containers/device_array.h>

//#include <pcl/console/parse.h>
//#include <pcl/gpu/kinfu/kinfu.h>
//#include <pcl/gpu/kinfu/raycaster.h>
//#include <pcl/gpu/kinfu/marching_cubes.h>
//#include <pcl/gpu/kinfu/tsdf_volume.h>
//#include <pcl/gpu/containers/initialization.h>
//#include <pcl/gpu/containers/device_array.h>

#include <cuda_runtime.h>
#include <assert.h>

typedef float            VoxelT;
typedef short            WeightT;

#define WIDTH_FULL   640 // 256
#define HEIGHT_FULL    480 // 192

#define W_SUB 64 // 64
#define H_SUB 48 // 48
#define N_SUB (W_SUB*H_SUB)

#endif

ros::Publisher pub;
tf::TransformListener listener;
pcl::gpu::kinfuLS::KinfuTracker* pcl_kinfu_tracker;
using namespace pcl;

void 
cloud_cb (const PointCloud<PointXYZRGB>::ConstPtr& input)
{
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
  
  //ros::spinOnce();
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  
  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud

  ros::init (argc, argv, "camera");
  ros::NodeHandle nh("~");
  std::string dev;
  if (!nh.getParam("device_number", dev)) {
    dev = "1";
  }
  std::string pointcloud_topic;
  if (!nh.getParam("pointcloud_topic", pointcloud_topic)) {
    //pointcloud_topic = "/hand_kinect_points";
    pointcloud_topic = "kinfu/points";
  }
  std::cout << "pointcloud_topic: " << pointcloud_topic << "\n";
  Grabber* grabber = new OpenNIGrabber("#" + dev);
  pub = nh.advertise<sensor_msgs::PointCloud2> (pointcloud_topic, 1);
  boost::function<void (const PointCloud<PointXYZRGB>::ConstPtr&)> f =
    boost::bind(&cloud_cb, _1);

  double fx = 525.0, fy = 525.0, cx = 319.5, cy = 239.5; // https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats

  double x = 2, y = 2, z = 2; // x and y must be equal!

  Vector3i gpu_size = Vector3i(y,z,x);
  Vector3d gpu_resolution = Vector3d(pcl::device::kinfuLS::VOLUME_X,
			    pcl::device::kinfuLS::VOLUME_Y,
			    pcl::device::kinfuLS::VOLUME_Z);

  // the transform from /base_link to /kinfu_frame
  // TODO: publish as static transform, use listener to read
  Matrix4d gpu_pcl_tsdf_origin = Matrix4d::Identity();
  gpu_pcl_tsdf_origin.block<3,3>(0,0) << -1, 0, 0,
    0, 0, -1,
    0, -1, 0;
  gpu_pcl_tsdf_origin.block<3,1>(0,3) = Vector3d(0, 0, 0) + Vector3d(x/2., y/2., z/2.);

  tf::StampedTransform kinfu_to_camera;
  // this transform will be published
  listener.lookupTransform("/kinfu_frame", "/camera_rgb_optical_frame",
			   ros::Time(0), kinfu_to_camera);
  
  
  // setup kinfu tracker
  float shifting_distance = 5.0f;
  pcl_kinfu_tracker = new pcl::gpu::kinfuLS::KinfuTracker(gpu_size.cast<float>(), shifting_distance, HEIGHT_FULL, WIDTH_FULL);
  pcl_kinfu_tracker->setDepthIntrinsics(fx, fy, cx, cy);

  // Matrix4d init_cam_pose_gpu = Matrix4d::Identity();
  // Vector3f t = (init_cam_pose_gpu.block<3,1>(0,3)).cast<float>();
  // Matrix3f R = (init_cam_pose_gpu.block<3,3>(0,0)).cast<float>();

  // Affine3f affine_init_cam_pose = Translation3f(t) * AngleAxisf(R);

  Affine3d affine_init_cam_pose;
  tf::transformTFToEigen(kinfu_to_camera, affine_init_cam_pose);
  pcl_kinfu_tracker->setInitialCameraPose(affine_init_cam_pose.cast<float>());
  pcl_kinfu_tracker->reset();
  



  grabber->registerCallback(f);
  grabber->start();

  ros::spinOnce();

  // Spin
  ros::spin ();
  grabber->stop();
}
