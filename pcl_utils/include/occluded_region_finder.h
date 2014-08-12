#ifndef OCCLUDED_REGION_FINDER_H_INCLUDED
#define OCCLUDED_REGION_FINDER_H_INCLUDED

#include <tsdf_converter.h>
#include <cluster_extraction.h>
#include <cluster_projection.h>

// for visualization
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions.h>


namespace occluded_region_finder {

void find_occluded_regions(std::vector<float> tsdf_distances, std::vector<short> tsdf_weights, Eigen::Matrix4d transformation_matrix, bool saving, std::string outfile, ros::Publisher pub, ros::Publisher points_pub);

}


#endif
