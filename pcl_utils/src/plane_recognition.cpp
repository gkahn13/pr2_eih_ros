#include <pcl_utils/plane_recognition.h>
#include <pcl/io/pcd_io.h>
#include "ros/ros.h"

using namespace std;

namespace plane_recognition
{
void calculate_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients)
{

    float leaf_size, distance_threshold;
    ros::param::param<float>("/occlusion_parameters/plane_recognition_leaf_size", leaf_size, 0.05f); // TODO: find the right value for this - could be pretty high I think?
    ros::param::param<float>("/occlusion_parameters/plane_recognition_distance_threshold", distance_threshold, 0.01f);

    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_MLESAC);
//    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance_threshold);

    // input cloud
    seg.setInputCloud (cloud_filtered);

    seg.segment (*inliers, *coefficients);

}
}


// TODO: Compiling was getting mad about this main. I should move it to another file, eventually.
/*
int
main (int argc, char** argv)
{
    string infile = argv[1];
    string outfile = argv[2];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(infile, *cloud);
    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    plane_recognition::calculate_plane(cloud, inliers, coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    pcl::PointCloud<pcl::PointXYZ> new_cloud;

    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
        //std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
        //                                       << cloud->points[inliers->indices[i]].y << " "
        //                                        << cloud->points[inliers->indices[i]].z << std::endl;
        int current_index = inliers->indices[i];
        pcl::PointXYZ current_point = cloud->points[current_index];
        // hardcoded value for end of table
        if (current_point.z < 1.1)
        {
            new_cloud.push_back(current_point);
        }
    }

    pcl::io::savePCDFileASCII(outfile, new_cloud);

    return (0);

}
*/
