#ifndef TSDF_CONVERTER_H_INCLUDED
#define TSDF_CONVERTER_H_INCLUDED

#include <ros/console.h>

#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>
#include <algorithm>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include <pcl_utils/pointcloud_voxel_grid.h>

namespace tsdf_converter {

void read_files(std::string distance_file, std::string weight_file, std::vector<float>* tsdf_distances, std::vector<short>* tsdf_weights);

void convert_tsdf(std::vector<float> tsdf_distances, std::vector<short> tsdf_weights, pcl::PointCloud<pcl::PointXYZ>::Ptr zero_crossing_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr inverse_cloud);

void get_weight_cloud(std::vector<float> tsdf_distances, std::vector<short> tsdf_weights, pcl::PointCloud<pcl::PointXYZRGB>::Ptr weights_cloud, int jump);

}


#endif
