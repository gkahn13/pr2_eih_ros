#ifndef OCCLUDED_REGION_FINDER_H_INCLUDED
#define OCCLUDED_REGION_FINDER_H_INCLUDED

#include <pcl_utils/tsdf_converter.h>
#include <pcl_utils/cluster_extraction.h>
#include <pcl_utils/cluster_projection.h>

// for visualization
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include <ros/console.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions.h>
#include <pcl_utils/OccludedRegion.h>
#include <pcl_utils/OccludedRegionArray.h>
#include <pcl_utils/BoundingBox.h>


namespace occluded_region_finder {

void find_occluded_regions(std::vector<float> tsdf_distances, std::vector<short> tsdf_weights, Eigen::Matrix4d transformation_matrix, bool saving, std::string outfile, ros::Publisher markers_pub,
                           ros::Publisher points_pub, ros::Publisher regions_pub, ros::Publisher plane_pub, ros::Publisher object_points_pub, ros::Publisher plane_points_pub); //,
                           //pcl::PointCloud<pcl::PointXYZ>::Ptr zero_crossing_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_cloud, PointCloudVoxelGrid::CloudType::Ptr inverse_cloud);

}


#endif
