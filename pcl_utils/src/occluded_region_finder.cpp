#include <tsdf_converter.h>
#include <cluster_extraction.h>
#include <cluster_projection.h>


int main(int argc, char** argv) {

    if (argc < 5)
    {
        std::cerr << "Not enough arguments! Please specify three input files and an output file prefix" << std::endl;
        exit(1);
    }

    char* infile1 = argv[1];
    char* infile2 = argv[2];
    std::string matrix_file = argv[3];
    std::string outfile = argv[4];

    int jump = 1;
    double voxel_size = 0.02;
    double cluster_tolerance = 0.02;
    int min_cluster_size = 100;
    int max_cluster_size = 25000;

    pcl::PointCloud<pcl::PointXYZ>::Ptr zero_crossing_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);
    PointCloudVoxelGrid::CloudType::Ptr inverse_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);;

    // load in the transformation matrix
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Zero();

    std::ifstream matrix_instream;
    matrix_instream.open(matrix_file.c_str());
    std::string current_input;
    for (int x = 0; x < transformation_matrix.rows(); x++)
    {
        for (int y = 0; y < transformation_matrix.cols(); y++)
        {
            getline(matrix_instream, current_input);
            transformation_matrix(x, y) = std::atof(current_input.c_str());
        }
    }
    matrix_instream.close();


    tsdf_converter::convert_tsdf(infile1, infile2, zero_crossing_cloud, foreground_cloud, inverse_cloud, jump, voxel_size);

    std::cout << "converted tsdf vectors" << std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters = cluster_extraction::extract_clusters(zero_crossing_cloud, cluster_tolerance, min_cluster_size, max_cluster_size);

    std::cout << "extracted positive clusters" << std::endl;

    pcl::PointCloud<pcl::PointXYZ> occluded_region;

    int j = 1;
    std::vector<pcl::PointCloud<pcl::PointXYZ> >::iterator cluster_iter;
    for (cluster_iter = clusters.begin(); cluster_iter != clusters.end(); cluster_iter++) {
        pcl::PointCloud<pcl::PointXYZ> current_cloud = *cluster_iter;

        occluded_region = cluster_projection::calculate_occluded(current_cloud, inverse_cloud, zero_crossing_cloud, transformation_matrix);

        std::stringstream ss;
        ss << outfile << "_cloud_cluster_" << j << ".pcd";
        pcl::io::savePCDFileASCII(ss.str(), current_cloud);

        std::stringstream ss2;
        ss2 << outfile << "_occluded_region_" << j << ".pcd";
        try {
            pcl::io::savePCDFileASCII(ss2.str(), occluded_region);
        } catch (pcl::IOException) {
            std::cout << "couldn't find occluded region for cluster " << j << std::endl;
        }

        std::cout << "cluster " << j << " complete" << std::endl;

        j++;
    }

    std::cout << "clusters finished" << std::endl;

    pcl::io::savePCDFileASCII(outfile + "_zero.pcd", *zero_crossing_cloud);
    pcl::io::savePCDFileASCII(outfile + "_foreground.pcd", *foreground_cloud);
    pcl::io::savePCDFileASCII(outfile + "_inverse.pcd", *inverse_cloud);

    std::cout << "done" << std::endl;

    return 0;

}
