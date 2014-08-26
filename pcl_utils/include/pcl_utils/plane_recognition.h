#ifndef PLANE_RECOGNITION_H_INCLUDED
#define PLANE_RECOGNITION_H_INCLUDED

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>

namespace plane_recognition {
    void calculate_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, ros::Publisher plane_pub, visualization_msgs::MarkerArrayPtr markers);
}


#endif // PLANE_RECOGNITION_H_INCLUDED
