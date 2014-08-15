#ifndef CLUSTER_PROJECTION_H_INCLUDED
#define CLUSTER_PROJECTION_H_INCLUDED

namespace cluster_projection {

pcl::PointCloud<pcl::PointXYZ> calculate_occluded(pcl::PointCloud<pcl::PointXYZ> cluster, pcl::PointCloud<pcl::PointXYZ>::Ptr inverse, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud,
                                                  Eigen::Matrix4d transformation_matrix, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_inverse,
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr projected_inverse, pcl::ModelCoefficients::Ptr plane_coeff,
                                                  int face_direction, pcl::PointXYZ min_point_OBB, pcl::PointXYZ max_point_OBB, Eigen::Vector3f position, Eigen::Matrix3f rotational_matrix_OBB);

}


#endif
