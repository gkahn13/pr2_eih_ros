//#include <pcl/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions.h>

#include <plane_recognition.h>

#include <Eigen/Eigen>

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "not enough input arguments! Please specify an input and output file" << std::endl;
        return(1);
    }

    std::string infile = argv[1];
    std::string outfile = argv[2];

    // load in the file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(infile, *cloud);
    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // calculate plane
    plane_recognition::calculate_plane(cloud, inliers, coefficients);

    double a = coefficients->values[0], b = coefficients->values[1],
                                            c = coefficients->values[2], d = coefficients->values[3];

    Eigen::Matrix3d plane_basis;
    plane_basis.block<3,1>(0, 0) = Eigen::Vector3d(a, b, c);
    plane_basis.block<3,1>(0, 0) = Eigen::Vector3d(-d / a);
    plane_basis.block<3,1>(0, 0) = Eigen::Vector3d(a, b, c).cross(Eigen::Vector3d(-d / a));

    Eigen::Affine3d basis_transformation = Eigen::Affine3d(plane_basis);

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, basis_transformation.inverse());

}

