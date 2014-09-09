#include <pcl_utils/plane_recognition.h>
#include <pcl/io/pcd_io.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_utils/BoundingBox.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>

using namespace std;

namespace plane_recognition
{
void calculate_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, ros::Publisher plane_pub, visualization_msgs::MarkerArrayPtr markers, ros::Publisher plane_points_pub)
{

    float leaf_size, distance_threshold;
    bool publish_plane_marker;
    ros::param::param<float>("/occlusion_parameters/plane_recognition_leaf_size", leaf_size, 0.05f); // TODO: find the right value for this - could be pretty high I think?
    ros::param::param<float>("/occlusion_parameters/plane_recognition_distance_threshold", distance_threshold, 0.04f);
    ros::param::param<bool>("/occlusion_parameters/publish_plane_marker", publish_plane_marker, false);

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
//    seg.setMethodType(pcl::SAC_MLESAC);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance_threshold);

    // input cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZ> plane_points;

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // Extract the inliers
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative (false);
    extract.filter(plane_points);

    sensor_msgs::PointCloud2 ros_plane_points;
    toROSMsg(plane_points, ros_plane_points);
    ros_plane_points.header.stamp = ros::Time::now();
    ros_plane_points.header.frame_id = "/kinfu_frame";
    plane_points_pub.publish(ros_plane_points);


    tf::TransformListener listener;
    tf::StampedTransform kinfu_to_base;
    listener.waitForTransform("/kinfu_frame", "/base_link",
                               ros::Time(0), ros::Duration(5));
    listener.lookupTransform("/kinfu_frame", "/base_link",
                              ros::Time(0), kinfu_to_base);

    Eigen::Affine3d kinfu_to_base_affine;
    tf::transformTFToEigen(kinfu_to_base, kinfu_to_base_affine);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points_base_link = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(plane_points, *plane_points_base_link, kinfu_to_base_affine.inverse());


    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(plane_points_base_link);
    feature_extractor.compute ();


    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    Eigen::Vector3f max_point_eigen(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);

    Eigen::Matrix3f vectors = rotational_matrix_OBB * max_point_eigen.asDiagonal();

    pcl_utils::BoundingBox bb_msg;
    bb_msg.header.stamp = ros::Time::now();
    bb_msg.header.frame_id = "/base_link";
    bb_msg.center.x = position_OBB.x;
    bb_msg.center.y = position_OBB.y;
    bb_msg.center.z = position_OBB.z;
    for (int i = 0; i < 3; i++) {
        geometry_msgs::Vector3 current_vector;
        current_vector.x = vectors(0, i);
        current_vector.y = vectors(1, i);
        current_vector.z = vectors(2, i);
        bb_msg.vectors.push_back(current_vector);
    }

    plane_pub.publish(bb_msg);

    Eigen::Quaternionf quat (rotational_matrix_OBB);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.id = 1000000;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position_OBB.x;
    marker.pose.position.y = position_OBB.y;
    marker.pose.position.z = position_OBB.z;
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.scale.x = max_point_OBB.x - min_point_OBB.x;
    marker.scale.y = max_point_OBB.y - min_point_OBB.y;
    marker.scale.z = max_point_OBB.z - min_point_OBB.z;
    marker.color.a = 1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    if (publish_plane_marker) {
        markers->markers.push_back(marker);
    }

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
