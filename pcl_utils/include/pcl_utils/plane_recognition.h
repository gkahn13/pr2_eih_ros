#ifndef PLANE_RECOGNITION_H_INCLUDED
#define PLANE_RECOGNITION_H_INCLUDED

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace plane_recognition {
    void calculate_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients);
}


#endif // PLANE_RECOGNITION_H_INCLUDED
