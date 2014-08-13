#include <pcl/common/transforms.h>
#include <pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>


#include <Eigen/Eigen>


#include <plane_recognition.h>

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "not enough input arguments! Please specify an input and output file" << std::endl;
        return(1);
    }

    double eps = 0.01;
    if (argc > 3) {
        eps = atof(argv[3]);
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

//    std::cout << "model coefficients: " << std::endl << a << std::endl << b << std::endl << c << std::endl << d << std::endl;
    Eigen::Vector3d n = Eigen::Vector3d(a, b, c);
    Eigen::FullPivLU<Eigen::RowVector3d> lu_decomp(n.transpose());
    Eigen::MatrixXd null_space = lu_decomp.kernel();

//    std::cout << "cols: " << null_space.cols() << std::endl;
//    std::cout << "rows: " << null_space.rows() << std::endl;

    Eigen::Vector3d b1 = null_space.block<3, 1>(0, 0);
    Eigen::Vector3d b2 = null_space.block<3, 1>(0, 1);

    Eigen::Matrix3d plane_basis;
    b1.normalize();
    b2.normalize();
    n.normalize();
    plane_basis.block<3,1>(0, 0) = b1;
    plane_basis.block<3,1>(0, 1) = b2;
    plane_basis.block<3,1>(0, 2) = n;

    Eigen::Affine3d basis_transformation = Eigen::Affine3d(plane_basis);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, basis_transformation.inverse().cast<float>());


    pcl::PointCloud<pcl::PointXYZ> plane_cloud;

    pcl::PointCloud<pcl::PointXYZ>::iterator transformed_iter;
    pcl::PointXYZ current;
    for (transformed_iter = transformed_cloud->begin();
            transformed_iter != transformed_cloud->end();
            transformed_iter++)
    {
        current = *transformed_iter;
//        std::cout << "z: " << current.z;
        if (fabs(current.z + d) < eps)   // && xyz_point.y >= -0.05 + 1.35 && xyz_point.y <= 0.05 + 1.35) {
        {
            pcl::PointXYZ new_current;
            new_current.x = current.x;
            new_current.y = current.y;
            new_current.z = -d;
            plane_cloud.push_back(new_current);
        }
    }

    //pcl::io::savePCDFileASCII(outfile, *transformed_cloud);
    pcl::io::savePCDFileASCII(outfile, plane_cloud);

    return(0);

}

