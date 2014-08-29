#include <pcl_utils/tsdf_converter.h>
#include <pcl_utils/timer.h>
#include <climits>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>


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

void convert_tsdf(std::vector<float> tsdf_distances, std::vector<short> tsdf_weights, pcl::PointCloud<pcl::PointXYZ>::Ptr zero_crossing_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr inverse_cloud) {
    // loop the pointcloud, finding zero crossing points and "foreground" points

    double resolution = 512;
    double size = 2;

    // get parameters
    int jump, prob;
    float voxel_size, min_x, max_x, min_y, max_y, min_z, max_z, tsdf_min_distance, tsdf_max_distance;
    ros::param::param<int>("/occlusion_parameters/tsdf_converter_jump", jump, 1);
    ros::param::param<float>("/occlusion_parameters/voxel_size", voxel_size, 0.02f);
    ros::param::param<int>("/occlusion_parameters/downsampling_rate", prob, 100);
    ros::param::param<float>("/occlusion_parameters/min_x", min_x, 0);
    ros::param::param<float>("/occlusion_parameters/max_x", max_x, 2);
    ros::param::param<float>("/occlusion_parameters/min_y", min_y, 0);
    ros::param::param<float>("/occlusion_parameters/max_y", max_y, 2);
    ros::param::param<float>("/occlusion_parameters/min_z", min_z, 0);
    ros::param::param<float>("occlusion_parameters/max_z", max_z, 2);
    ros::param::param<float>("occlusion_parameters/tsdf_min_distance", tsdf_min_distance, 0.2);
    ros::param::param<float>("occlusion_parameters/tsdf_max_distance", tsdf_max_distance, 0.8);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inverse_full (new pcl::PointCloud<pcl::PointXYZ>);

    boost::mt19937 gen;
    boost::random::uniform_int_distribution<> dist(1, prob);


    int i = 0;
    for (int z = 0; z < resolution; z = z + jump) {
        for (int y = 0; y < resolution; y = y + jump) {
            for (int x = 0; x < resolution; x = x + jump) {
                float current_distance = tsdf_distances[resolution * resolution * z + resolution * y + x];
                short current_weight = tsdf_weights[resolution * resolution * z + resolution * y + x];
                pcl::PointXYZ current;
                current.x = x * size / resolution;
                current.y = y * size / resolution;
                current.z = z * size / resolution;

                if (current.x >= min_x && current.x <= max_x &&
                    current.y >= min_y && current.y <= max_y &&
                    current.z >= min_z && current.z <= max_z) {

                    if (current_weight > 0 && current_distance > tsdf_min_distance && current_distance < tsdf_max_distance) {
                        zero_crossing_cloud->push_back(current);
                    }

                    if (current_weight > 0 && current_distance > 0.5) {
                    //if (current_weight > 0.1 && current_distance > 0.5) {
                        foreground_cloud->push_back(current);
                    }

                    if ((current_weight < 50 && current_weight > 0 && dist(gen) % prob == 0) /*|| (current_weight == 0 && dist(gen) % prob == 0)*/) {
                        inverse_full->push_back(current);
                        //inverse_cloud->push_back(current);
                    }
                }

                i++;
            }
        }
    }

//    pcl::VoxelGrid<pcl::PointXYZ> vg;
//    vg.setInputCloud (inverse_full);
//    float leaf_size = 0.01f;
//    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
//    vg.filter (*inverse_cloud);
////    *inverse_cloud = *inverse_full;


    PointCloudVoxelGrid vox_grid = PointCloudVoxelGrid(foreground_cloud, voxel_size);
    vox_grid.get_inverse_cloud(inverse_cloud);

    std::cout << "number of points in inverse cloud: " << inverse_cloud->size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::iterator iter;
    for (iter = inverse_full->begin(); iter != inverse_full->end(); iter++) {
        inverse_cloud->push_back(*iter);
    }

    std::cout << "number of points in inverse cloud (after adding low weight points): " << inverse_cloud->size() << std::endl;


}

void get_weight_cloud(std::vector<float> tsdf_distances, std::vector<short> tsdf_weights, pcl::PointCloud<pcl::PointXYZRGB>::Ptr weights_cloud, int jump) {
    double resolution = 512;
    double size = 2;


    for (int z = 0; z < resolution; z = z + jump) {
        for (int y = 0; y < resolution; y = y + jump) {
            for (int x = 0; x < resolution; x = x + jump) {
                float current_distance = tsdf_distances[resolution * resolution * z + resolution * y + x];
                short current_weight = tsdf_weights[resolution * resolution * z + resolution * y + x];
                pcl::PointXYZRGB current;
                current.x = x * size / resolution;
                current.y = y * size / resolution;
                current.z = z * size / resolution;


                if (current_weight > 0 && current_weight < 1000 && current_distance > 0.1 && current_distance < 0.9) {
                    current.g = (current_weight / 128.0) * 70 + 15;
                    current.r = 0;
                    current.b = 0;
                    weights_cloud->push_back(current);
                }
            }
        }
    }

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

