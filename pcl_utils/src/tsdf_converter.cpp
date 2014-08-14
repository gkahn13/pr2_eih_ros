#include <pcl_utils/tsdf_converter.h>
#include <pcl_utils/timer.h>

namespace tsdf_converter {
void read_files(std::string distance_file, std::string weight_file, std::vector<float>* tsdf_distances, std::vector<short>* tsdf_weights) {

    // read the raw binary files into vectors

    std::ifstream is1(distance_file.c_str(), std::ios::binary);
    is1.seekg(0, std::ifstream::end);
    long size1 = is1.tellg();
    is1.seekg(0, std::ifstream::beg);
//    std::cout << size1 / sizeof(float) << std::endl;
    tsdf_distances->resize(size1 / sizeof(float));
    is1.read(reinterpret_cast<char*>(&((*tsdf_distances)[0])), size1);
    //is1.close();

    std::ifstream is2(weight_file.c_str(), std::ios::binary);
    is2.seekg(0, std::ifstream::end);
    long size2 = is2.tellg();
    is2.seekg(0, std::ifstream::beg);
//    std::cout << size2 / sizeof(short) << std::endl;
    tsdf_weights->resize(size2 / sizeof(short));
    is2.read(reinterpret_cast<char*>(&((*tsdf_weights)[0])), size2);
    //is2.close();

}

void convert_tsdf(std::vector<float> tsdf_distances, std::vector<short> tsdf_weights, pcl::PointCloud<pcl::PointXYZ>::Ptr zero_crossing_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr inverse_cloud,
                  int jump, double voxel_size) {
    // loop the pointcloud, finding zero crossing points and (optionally) "foreground" points

    double resolution = 512;
    double size = 2;



    for (int z = 0; z < resolution; z = z + jump) {
        for (int y = 0; y < resolution; y = y + jump) {
            for (int x = 0; x < resolution; x = x + jump) {
                float current_distance = tsdf_distances[resolution * resolution * z + resolution * y + x];
                short current_weight = tsdf_weights[resolution * resolution * z + resolution * y + x];
                pcl::PointXYZ current;
                current.x = x * size / resolution;
                current.y = y * size / resolution;
                current.z = z * size / resolution;

                if (current_weight > 0 && current_distance > 0.2 && current_distance < 0.8) {
                    zero_crossing_cloud->push_back(current);
                }

                if (current_weight > 0 && current_distance > 0.5) {
                    foreground_cloud->push_back(current);
                }
            }
        }
    }



    PointCloudVoxelGrid vox_grid = PointCloudVoxelGrid(foreground_cloud, voxel_size);
    vox_grid.get_inverse_cloud(inverse_cloud);


}

}

// now handled in occluded_region_finder.cpp

//int main(int argc, char** argv)
//{
//    if (argc < 4)
//    {
//        std::cerr << "Not enough arguments! Please specify at least two input and one output files" << std::endl;
//        exit(1);
//    }
//
//    char* infile1 = argv[1];
//    char* infile2 = argv[2];
//    char* outfile1 = argv[3];
//    // if inverse is true, will output foreground cloud and inverse cloud
//    // if false, will only output the zero crossing pointcloud
//    bool inverse = false;
//    char* outfile2 = "";
//    char* outfile3 = "";
//    if (argc >= 6) {
//        outfile2 = argv[4];
//        outfile3 = argv[5];
//        inverse = true;
//    }
//
//    int jump = 1;
//    if (argc >= 7) {
//        jump = std::atoi(argv[6]);
//    }
//
//    double voxel_size = 0.02;
//    if (argc >= 8) {
//        voxel_size = std::atof(argv[7]);
//    }
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr zero_crossing_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);
//    PointCloudVoxelGrid::CloudType::Ptr inverse_cloud;
//
//
//    tsdf_converter::convert_tsdf(infile1, infile2, zero_crossing_cloud, foreground_cloud, inverse_cloud, jump, voxel_size);
//
//    pcl::io::savePCDFileASCII(outfile1, *zero_crossing_cloud);
//    pcl::io::savePCDFileASCII(outfile2, *foreground_cloud);
//    pcl::io::savePCDFileASCII(outfile3, *inverse_cloud);
//
//    return(0);
//
//}
