#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Eigen>

#include <pointcloud_voxel_grid.h>
#include <plane_recognition.h>

namespace cluster_projection {

pcl::PointCloud<pcl::PointXYZ> calculate_occluded(pcl::PointCloud<pcl::PointXYZ> cluster, pcl::PointCloud<pcl::PointXYZ>::Ptr inverse, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud, Eigen::Matrix4d transformation_matrix) {
    // transform pointclouds into common coordinate frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_inverse(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(cluster, *transformed_cluster, transformation_matrix);
    pcl::transformPointCloud(*inverse, *transformed_inverse, transformation_matrix);

    double fx = 525., fy = 525., cx = 319.5, cy = 239.5;

    Eigen::Matrix3d P;
    P << fx, 0, cx,
    0, fy, cy,
    0, 0, 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_inverse(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> inliers;

    // project pointclouds onto image plane
    Eigen::Affine3d affine_transformation = Eigen::Affine3d(P);
    pcl::transformPointCloud(*transformed_cluster, *projected_cluster, affine_transformation);
    pcl::transformPointCloud(*transformed_inverse, *projected_inverse, affine_transformation);

    // find corners for the (axis aligned) bounding box in 2-D
    Eigen::Matrix<double, 6, 1> extremes = PointCloudVoxelGrid::calculate_extremes(projected_cluster);

    std::cout << extremes << std::endl;


    // fit a plane to the zero-crossing points to find a table top
    pcl::PointIndices::Ptr unused(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
    plane_recognition::calculate_plane(plane_cloud, unused, plane_coeff);
    double a = plane_coeff->values[0], b = plane_coeff->values[1],
    c = plane_coeff->values[2], d = plane_coeff->values[3];

    std::cout << "plane coefficients: " << a << ", " << b << ", " << c << ", " << d << std::endl;

    // loop through the clouds, finding the intersection of the inverse cloud and the bounding box
    // of the projected object
    pcl::PointCloud<pcl::PointXYZ>::iterator projected_inverse_iter;
    pcl::PointCloud<pcl::PointXYZ>::iterator inverse_iter;
    pcl::PointCloud<pcl::PointXYZ> occluded_region;
    double tol = 25;
    for (projected_inverse_iter = projected_inverse->begin(), inverse_iter = inverse->begin();
        projected_inverse_iter != projected_inverse->end();
        projected_inverse_iter++, inverse_iter++) {

        pcl::PointXYZ current_projected = *projected_inverse_iter;
        pcl::PointXYZ current_inverse = *inverse_iter;
        if (current_projected.x / current_projected.z >= extremes(0) / extremes(2) - tol &&
            current_projected.y / current_projected.z >= extremes(1) / extremes(2) - tol &&
            current_projected.x / current_projected.z <= extremes(3) / extremes(5) + tol &&
            current_projected.y / current_projected.z <= extremes(4) / extremes(5) + tol &&
            a * current_inverse.x + b * current_inverse.y + c * current_inverse.z + d + 0.05 >= 0) {
            occluded_region.push_back(current_inverse);
        }
    }

    return occluded_region;

}

}

// now handled in occluded_region_finder.cpp

//int main (int argc, char** argv)
//{
//    if (argc < 5)
//    {
//        std::cerr << "not enough input arguments! please specify three input files and an output file" << std::endl;
//        return 1;
//    }
//
//    std::string cluster_file = argv[1];
//    std::string inverse_file = argv[2];
//    std::string plane_file = argv[3];
//    std::string matrix_file = argv[4];
//    std::string outfile1 = argv[5];
//
//    // load in pointclouds
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr inverse(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile(cluster_file, *cluster);
//    pcl::io::loadPCDFile(inverse_file, *inverse);
//    pcl::io::loadPCDFile(plane_file, *plane_cloud);
//
//    // load in the transformation matrix
//    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Zero();
//
//    std::ifstream matrix_instream;
//    matrix_instream.open(matrix_file.c_str());
//    std::string current_input;
//    for (int x = 0; x < transformation_matrix.rows(); x++)
//    {
//        for (int y = 0; y < transformation_matrix.cols(); y++)
//        {
//            getline(matrix_instream, current_input);
//            transformation_matrix(x, y) = std::atof(current_input.c_str());
//        }
//    }
//    matrix_instream.close();
//
//    std::cout << transformation_matrix << std::endl;
//
//    pcl::PointCloud<pcl::PointXYZ> occluded_region = cluster_projection::calculate_occluded(cluster, inverse, plane_cloud, transformation_matrix);
//
//    pcl::io::savePLYFileASCII(outfile1, occluded_region);
//
//    return 0;
//
//}
