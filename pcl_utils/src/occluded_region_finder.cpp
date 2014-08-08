#include <occluded_region_finder.h>
#include <timer.h>

namespace occluded_region_finder {

void find_occluded_regions(std::vector<float> tsdf_distances, std::vector<short> tsdf_weights, Eigen::Matrix4d transformation_matrix, bool saving, std::string outfile) {

    Timer timer = Timer();

    int jump = 1;
    double voxel_size = 0.02;
    double cluster_tolerance = 0.02;
    int min_cluster_size = 100;
    int max_cluster_size = 25000;

    pcl::PointCloud<pcl::PointXYZ>::Ptr zero_crossing_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);
    PointCloudVoxelGrid::CloudType::Ptr inverse_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);

    Timer_tic(&timer);
    tsdf_converter::convert_tsdf(tsdf_distances, tsdf_weights, zero_crossing_cloud, foreground_cloud, inverse_cloud, jump, voxel_size);
    std::cout << "convert tsdf: " << Timer_toc(&timer) << std::endl;

    std::cout << "converted tsdf vectors" << std::endl;

    Timer_tic(&timer);

    std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters = cluster_extraction::extract_clusters(zero_crossing_cloud, cluster_tolerance, min_cluster_size, max_cluster_size);

    std::cout << "cluster extraction: " << Timer_toc(&timer) << std::endl;
    Timer_tic(&timer);

    std::cout << "extracted positive clusters" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_inverse(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);

    pcl::PointCloud<pcl::PointXYZ> occluded_region;

    std::vector<Eigen::Matrix<double, 4, 1> > means;
    std::vector<Eigen::Matrix3d> covariances;

    Eigen::Matrix<double, 4, 1> mean;
    Eigen::Matrix3d covariance;
    int j = 1;
    std::vector<pcl::PointCloud<pcl::PointXYZ> >::iterator cluster_iter;
    for (cluster_iter = clusters.begin(); cluster_iter != clusters.end(); cluster_iter++) {
        pcl::PointCloud<pcl::PointXYZ> current_cloud = *cluster_iter;

        occluded_region = cluster_projection::calculate_occluded(current_cloud, inverse_cloud, zero_crossing_cloud, transformation_matrix, projected_inverse, plane_coeff);

        pcl::compute3DCentroid(occluded_region, mean);
        pcl::computeCovarianceMatrix(occluded_region, mean, covariance);

//        std::cout << mean << std::endl;
//        std::cout << covariance << std::endl;

        if (saving) {
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
        }

        std::cout << "cluster " << j << " complete" << std::endl;

        j++;
    }

    std::cout << "clusters finished" << std::endl;

    if (saving) {
        pcl::io::savePCDFileASCII(outfile + "_zero.pcd", *zero_crossing_cloud);
//        pcl::io::savePCDFileASCII(outfile + "_foreground.pcd", *foreground_cloud);
//        pcl::io::savePCDFileASCII(outfile + "_inverse.pcd", *inverse_cloud);
    }

    std::cout << "occlusion finding: " << Timer_toc(&timer) << std::endl;

    std::cout << "done" << std::endl;
}

}


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
    bool saving = std::atoi(argv[5]);



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

    std::vector<float>* tsdf_distances = new std::vector<float>;
    std::vector<short>* tsdf_weights = new std::vector<short>;

    tsdf_converter::read_files(infile1, infile2, tsdf_distances, tsdf_weights);

    occluded_region_finder::find_occluded_regions(*tsdf_distances, *tsdf_weights, transformation_matrix, saving, outfile);

    return 0;

}
