#ifndef CLUSTER_PROJECTION_H_INCLUDED
#define CLUSTER_PROJECTION_H_INCLUDED

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace cluster_projection {

pcl::PointCloud<pcl::PointXYZ> calculate_occluded(pcl::PointCloud<pcl::PointXYZ> cluster, pcl::PointCloud<pcl::PointXYZ>::Ptr inverse, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud,
                                                  Eigen::Matrix4d transformation_matrix, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_inverse,
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr projected_inverse, pcl::ModelCoefficients::Ptr plane_coeff,
                                                  int face_direction, int forward_back, pcl::PointXYZ min_point_OBB, pcl::PointXYZ max_point_OBB, Eigen::Vector3f position, Eigen::Matrix3f rotational_matrix_OBB,
                                                  visualization_msgs::MarkerArrayPtr markers, std::vector<Eigen::Vector3f> corners, ros::Publisher plane_pub);

}


#endif
