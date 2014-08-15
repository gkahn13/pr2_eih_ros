#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Eigen>

#include <pcl_utils/pointcloud_voxel_grid.h>
#include <pcl_utils/plane_recognition.h>
#include <pcl_utils/timer.h>
#include <pcl_utils/cluster_projection.h>

namespace cluster_projection {

pcl::PointCloud<pcl::PointXYZ> calculate_occluded(pcl::PointCloud<pcl::PointXYZ> cluster, pcl::PointCloud<pcl::PointXYZ>::Ptr inverse, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud,
                                                  Eigen::Matrix4d transformation_matrix, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_inverse,
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr projected_inverse, pcl::ModelCoefficients::Ptr plane_coeff,
                                                  int face_direction, pcl::PointXYZ min_point_OBB, pcl::PointXYZ max_point_OBB, Eigen::Vector3f position, Eigen::Matrix3f rotational_matrix_OBB,
                                                  visualization_msgs::MarkerArrayPtr markers) {


    Eigen::Vector3f min_vector(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    //Eigen::Vector3f min_vector_rotated = rotational_matrix_OBB * min_vector;

    Eigen::Matrix3f adder0 = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f adder1 = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f adder2 = Eigen::Matrix3f::Zero();
    Eigen::Vector3f major_vector, middle_vector, minor_vector,
                    major_vector_normalized, middle_vector_normalized, minor_vector_normalized;
    adder0(0, 0) = 1;
    adder1(1, 1) = 1;
    adder2(2, 2) = 1;
    major_vector = rotational_matrix_OBB * adder0 * min_vector;
    middle_vector = rotational_matrix_OBB * adder1 * min_vector;
    minor_vector = rotational_matrix_OBB * adder2 * min_vector;
    major_vector_normalized = major_vector / major_vector.norm();
    middle_vector_normalized = middle_vector / middle_vector.norm();
    minor_vector_normalized = minor_vector / minor_vector.norm();


//    Timer timer = Timer();
    //Timer_tic(&timer);
    // transform pointclouds into common coordinate frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cluster(new pcl::PointCloud<pcl::PointXYZ>);

    //Eigen::Affine3d affine1 = Eigen::Affine3d(transformation_matrix);
    pcl::transformPointCloud(cluster, *transformed_cluster, transformation_matrix);


    double fx = 525., fy = 525., cx = 319.5, cy = 239.5;

    Eigen::Matrix3d P;
    P << fx, 0, cx,
    0, fy, cy,
    0, 0, 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cluster(new pcl::PointCloud<pcl::PointXYZ>);

    // project pointclouds onto image plane
    Eigen::Affine3d affine_transformation = Eigen::Affine3d(P);
    pcl::transformPointCloud(*transformed_cluster, *projected_cluster, affine_transformation);

    // only projects the inverse cloud once, as it will not vary between clusters
    if (projected_inverse->size() == 0) {
        pcl::transformPointCloud(*inverse, *transformed_inverse, transformation_matrix);
        pcl::transformPointCloud(*transformed_inverse, *projected_inverse, affine_transformation);
    }

    //std::cout << "transforming and projecting: " << Timer_toc(&timer) << std::endl;

    // I thought combining the transformations would make things faster, but it didn't seem to help
//    pcl::transformPointCloud(cluster, *projected_cluster, affine_transformation * affine1);
//    pcl::transformPointCloud(*inverse, *projected_inverse, affine_transformation * affine1);

    //Timer_tic(&timer);
    // find corners for the (axis aligned) bounding box in 2-D
    Eigen::Matrix<double, 6, 1> extremes = PointCloudVoxelGrid::calculate_extremes(projected_cluster);
    //std::cout << "calculating extremes: " << Timer_toc(&timer) << std::endl;
    Eigen::Matrix<double, 6, 1> real_extremes = PointCloudVoxelGrid::calculate_extremes(transformed_cluster);


    //Timer_tic(&timer);

    // only do this once
    if (plane_coeff->values.size() == 0) {
        // fit a plane to the zero-crossing points to find a table top
        pcl::PointIndices::Ptr unused(new pcl::PointIndices);
        plane_recognition::calculate_plane(plane_cloud, unused, plane_coeff);
    }

    double a = plane_coeff->values[0], b = plane_coeff->values[1],
    c = plane_coeff->values[2], d = plane_coeff->values[3];

    //std::cout << "plane fitting: " << Timer_toc(&timer) << std::endl;

    //std::cout << "plane coefficients: " << a << ", " << b << ", " << c << ", " << d << std::endl;

    //Timer_tic(&timer);
    // loop through the clouds, finding the intersection of the inverse cloud and the bounding box
    // of the projected object
    pcl::PointCloud<pcl::PointXYZ>::iterator projected_inverse_iter;
    pcl::PointCloud<pcl::PointXYZ>::iterator inverse_iter;
    pcl::PointCloud<pcl::PointXYZ>::iterator transformed_inverse_iter;
    pcl::PointCloud<pcl::PointXYZ> occluded_region;
    double tol = 25;
    std::cout << "face_direction: " << face_direction << std::endl;
    //face_direction = 10;
    for (projected_inverse_iter = projected_inverse->begin(), inverse_iter = inverse->begin(), transformed_inverse_iter = transformed_inverse->begin();
        projected_inverse_iter != projected_inverse->end();
        projected_inverse_iter++, inverse_iter++, transformed_inverse_iter++) {

        pcl::PointXYZ current_projected = *projected_inverse_iter;
        pcl::PointXYZ current_inverse = *inverse_iter;
        Eigen::Vector3f current_inverse_eigen(transformed_inverse_iter->x, transformed_inverse_iter->y, transformed_inverse_iter->z);
        Eigen::Vector3f current_inverse_rotated = rotational_matrix_OBB * current_inverse_eigen;
        if (/*current_projected.x / current_projected.z >= extremes(0) / extremes(2) - tol &&
            current_projected.y / current_projected.z >= extremes(1) / extremes(2) - tol &&
            current_projected.x / current_projected.z <= extremes(3) / extremes(5) + tol &&
            current_projected.y / current_projected.z <= extremes(4) / extremes(5) + tol &&*/
            ((face_direction == 0) || (major_vector_normalized.dot((current_inverse_eigen - position) - major_vector) <= 0)) &&
            ((face_direction == 0) || (major_vector_normalized.dot(-1 * (current_inverse_eigen - position) - major_vector)) <= 0) &&
            ((face_direction == 1) || (middle_vector_normalized.dot((current_inverse_eigen - position) - middle_vector) <= 0)) &&
            ((face_direction == 1) || (middle_vector_normalized.dot(-1 * (current_inverse_eigen - position) - middle_vector)) <= 0) &&
            ((face_direction == 2) || (minor_vector_normalized.dot((current_inverse_eigen - position) - minor_vector) <= 0)) &&
            ((face_direction == 2) || (minor_vector_normalized.dot(-1 * (current_inverse_eigen - position) - minor_vector)) <= 0) &&
            a * current_inverse.x + b * current_inverse.y + c * current_inverse.z + d + 0.05 >= 0 &&
            transformed_inverse_iter->z > 0 &&
            std::pow(transformed_inverse_iter->x, 2) + std::pow(transformed_inverse_iter->y, 2) + std::pow(transformed_inverse_iter->z, 2) >= std::pow(real_extremes.block<3, 1>(0,0).norm(), 2)) {
            occluded_region.push_back(current_inverse);
//            std::cout << "point" << std::endl;
//            std::cout << (major_vector.dot((current_inverse_eigen - position) - major_vector) <= 0) << std::endl;
//            std::cout << ((major_vector.dot(-1 * (current_inverse_eigen - position) - major_vector)) >= 0) << std::endl;
//            std::cout << (middle_vector.dot((current_inverse_eigen - position) - middle_vector) <= 0) << std::endl;
//            std::cout << ((middle_vector.dot(-1 * (current_inverse_eigen - position) - middle_vector)) >= 0) << std::endl;
//            std::cout << (minor_vector.dot((current_inverse_eigen - position) - minor_vector) <= 0) << std::endl;
//            std::cout << ((minor_vector.dot(-1 * (current_inverse_eigen - position) - minor_vector)) >= 0) << std::endl;
        }
    }

    //std::cout << "occluded region loop: " << Timer_toc(&timer) << std::endl;


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
