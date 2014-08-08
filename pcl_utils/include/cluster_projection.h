#ifndef CLUSTER_PROJECTION_H_INCLUDED
#define CLUSTER_PROJECTION_H_INCLUDED

namespace cluster_projection {

pcl::PointCloud<pcl::PointXYZ> calculate_occluded(pcl::PointCloud<pcl::PointXYZ> cluster, pcl::PointCloud<pcl::PointXYZ>::Ptr inverse, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud,
                                                  Eigen::Matrix4d transformation_matrix, pcl::PointCloud<pcl::PointXYZ>::Ptr projected_inverse, pcl::ModelCoefficients::Ptr plane_coeff);

}


#endif
