#ifndef CLUSTER_EXTRACTION_H_INCLUDED
#define CLUSTER_EXTRACTION_H_INCLUDED

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


namespace cluster_extraction {

    std::vector<pcl::PointCloud<pcl::PointXYZ> > extract_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double cluster_tolerance, int min_cluster_size, int max_cluster_size);

}








#endif
