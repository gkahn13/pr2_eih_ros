#include <pcl_utils/tsdf_converter.h>

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "not enough input arguments!" << std::endl;
        return 1;
    }

    std::string dist_file = argv[1];
    std::string weight_file = argv[2];
    std::string output_file = argv[3];
    int jump = 1;
    if (argc > 4) {
        jump = std::atoi(argv[4]);
    }

    std::vector<float>* distances = new std::vector<float>;
    std::vector<short>* weights = new std::vector<short>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::cout << "about to read files" << std::endl;
    tsdf_converter::read_files(dist_file, weight_file, distances, weights);

    std::cout << "about to get cloud" << std::endl;
    tsdf_converter::get_weight_cloud(*distances, *weights, cloud, jump);

    std::cout << "about to save cloud (size: " << cloud->size() << ")"<< std::endl;

    pcl::io::savePCDFileASCII(output_file, *cloud);

    std::cout << "done" << std::endl;
}
