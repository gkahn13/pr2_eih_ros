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
//#include <pcl/gpu/kinfu/tsdf_volume.h>

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
boost::shared_ptr<tf::TransformListener> listener;
pcl::gpu::kinfuLS::KinfuTracker* pcl_kinfu_tracker;
bool downloading;
int counter;
using namespace pcl;
using namespace std;

void 
cloud_cb (const PointCloud<PointXYZRGB>::ConstPtr& input)
{

  cout << "is organized: " << input->isOrganized() << endl;
  cout << "height: " << input->height << endl;
  cout << "width: " << input->width << endl;
  
  cout << "entered callback" << endl;
  if (!downloading && (counter % 30 == 0)) {
    

    
    cout << "updating" << endl;
  // get the current location of the camera relative to the kinfu frame
  tf::StampedTransform kinfu_to_camera;
  // published in kinfu.launch
  listener->waitForTransform("/kinfu_frame", "/camera_rgb_optical_frame",
  			     ros::Time(0), ros::Duration(5));
  listener->lookupTransform("/kinfu_frame", "/camera_rgb_optical_frame",
  			    ros::Time(0), kinfu_to_camera);
  
  // convert camera pose to format suitable for kinfu
  Affine3d affine_current_cam_pose;
  tf::transformTFToEigen(kinfu_to_camera, affine_current_cam_pose);

  pcl::PointCloud<PointXYZRGB> transformed_cloud;
  pcl::transformPointCloud(*input, transformed_cloud, affine_current_cam_pose.inverse()); // might not need inverse here


  // convert the data into gpu format for kinfu tracker to use
  pcl::gpu::DeviceArray2D<unsigned short> depth(H_SUB,W_SUB);
  std::vector<unsigned short> data(HEIGHT_FULL*WIDTH_FULL);
  int cols = WIDTH_FULL;
  
  int i;
  pcl::PointCloud<PointXYZRGB>::iterator cloud_iter;
  for(cloud_iter = transformed_cloud.begin(), i = 0;
      cloud_iter != transformed_cloud.end();
      cloud_iter++, i++) {
    data[i] = static_cast<unsigned short>(cloud_iter->z);
  }

  depth.upload(data, cols);

  // update kinfu tracker with new depth map and camera pose
  (*pcl_kinfu_tracker)(depth, affine_current_cam_pose.cast<float>());
  cout << "updated" << endl;
    
  //ros::spinOnce();
  }
  counter++;
}


int
main (int argc, char** argv)
{
  // Initialize ROS

  ros::init(argc, argv, "kinfu");
  //ros::Duration(2).sleep();
  ros::NodeHandle nh("~");
  std::string dev;
  // fill in tf listener
  listener.reset(new tf::TransformListener());

  if (!nh.getParam("device_number", dev)) {
    dev = "1";
  }
  std::string pointcloud_topic;
  if (!nh.getParam("pointcloud_topic", pointcloud_topic)) {
    pointcloud_topic = "kinfu_points";
  }
  std::cout << "pointcloud_topic: " << pointcloud_topic << "\n";
  
  // grabber to get data from the device
  Grabber* grabber = new OpenNIGrabber("#" + dev);
  
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> (pointcloud_topic, 1);
  boost::function<void (const PointCloud<PointXYZRGB>::ConstPtr&)> f =
    boost::bind(&cloud_cb, _1);
  

  // depth-camera intrinsics
  double fx = 525.0, fy = 525.0, cx = 319.5, cy = 239.5; // https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats

  double x = 2, y = 2, z = 2; // x and y must be equal!

  Vector3i gpu_size = Vector3i(y,z,x);
  Vector3d gpu_resolution = Vector3d(pcl::device::kinfuLS::VOLUME_X,
  			    pcl::device::kinfuLS::VOLUME_Y,
  			    pcl::device::kinfuLS::VOLUME_Z);

  // the transform from /base_link to /kinfu_frame
  // now published in kinfu.launch, so the tf listener can just read it in

  // Matrix4d gpu_pcl_tsdf_origin = Matrix4d::Identity();
  // gpu_pcl_tsdf_origin.block<3,3>(0,0) << -1, 0, 0,
  //   0, 0, -1,
  //   0, -1, 0;
  // gpu_pcl_tsdf_origin.block<3,1>(0,3) = Vector3d(0, 0, 0) + Vector3d(x/2., y/2., z/2.);

  tf::StampedTransform kinfu_to_camera;
  // published in kinfu.launch
  listener->waitForTransform("/kinfu_frame", "/camera_rgb_optical_frame",
  			     ros::Time(0), ros::Duration(5));
  listener->lookupTransform("/kinfu_frame", "/camera_rgb_optical_frame",
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

  cout << "ready to publish clouds" << endl;
  downloading = false;
  pcl::PointCloud<PointXYZ> current_cloud;
  sensor_msgs::PointCloud2 output;
  while(ros::ok()) {
    string response;
    getline(cin, response); // wait for key press
    if (response == "q") {
      grabber->stop();
      pub.publish(output);
      pcl::io::savePCDFileASCII("kinfu.pcd", current_cloud);
      exit(0);
    }
    downloading = true;
    cout << "publishing cloud" << endl;
    // Download tsdf and convert to pointcloud
    pcl::gpu::kinfuLS::TsdfVolume tsdf = pcl_kinfu_tracker->volume();
    tsdf.fetchCloudHost(current_cloud);

    // Publish the data
    toROSMsg(current_cloud, output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "/kinfu_frame";
    pub.publish (output);
    downloading = false;
    cout << "published" << endl;
  }


  // Spin
  ros::spin ();
  grabber->stop();
}
