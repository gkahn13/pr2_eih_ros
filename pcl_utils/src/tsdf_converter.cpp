#include "ros/ros.h"
#include <ros/console.h>

#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions.h>

#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>
#include <algorithm>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>


int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::cerr << "Not enough arguments! Please specify two input and one output files" << std::endl;
        exit(1);
    }

    char* infile1 = argv[1];
    char* infile2 = argv[2];
    char* outfile = argv[3];


    std::vector<float> tsdf_distances;
    std::vector<short> tsdf_weights;

    // read the raw binary files into vectors

    std::ifstream is1(infile1, std::ios::binary);
    is1.seekg(0, std::ifstream::end);
    long size1 = is1.tellg();
    is1.seekg(0, std::ifstream::beg);
    std::cout << size1 / sizeof(float) << std::endl;
    tsdf_distances.resize(size1 / sizeof(float));
    is1.read(reinterpret_cast<char*>(&tsdf_distances[0]), size1);
    //is1.close();

    std::ifstream is2(infile2, std::ios::binary);
    is2.seekg(0, std::ifstream::end);
    long size2 = is2.tellg();
    is2.seekg(0, std::ifstream::beg);
    std::cout << size2 / sizeof(short) << std::endl;
    tsdf_weights.resize(size2 / sizeof(short));
    is2.read(reinterpret_cast<char*>(&tsdf_weights[0]), size2);
    //is2.close();


    //exit(0);

    double resolution = 512;
    double size = 2;

    pcl::PointCloud<pcl::PointXYZ> new_cloud;

    int jump = 4;
    for (int z = 0; z < resolution; z = z + jump) {
        for (int y = 0; y < resolution; y = y + jump) {
            for (int x = 0; x < resolution; x = x + jump) {
                float current_distance = tsdf_distances[resolution * resolution * z + resolution * y + x];
                short current_weight = tsdf_weights[resolution * resolution * z + resolution * y + x];
                pcl::PointXYZ current;
                current.x = x * size / resolution;
                current.y = y * size / resolution;
                current.z = z * size / resolution;
                if (current_weight > 0 && current_distance < 0.5) {
                    new_cloud.push_back(current);
                }
            }
        }
    }


    std::cout << "cloud height: " << new_cloud.height << std::endl;
    std::cout << "cloud width: " << new_cloud.width << std::endl;

    pcl::io::savePCDFileASCII(outfile, new_cloud);

    /*

    std::cout << "finished creating pointcloud, about to publish" << std::endl;

    ros::init(argc, argv, "tsdf_publisher");
	ros::NodeHandle nh("~");

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("tsdf_zero_crossing", 1);


    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(new_cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "/kinfu_frame";

    std::cout << "converted, ready to publish" << std::endl;

    ros::spinOnce();

    while(ros::ok()) {
        ros::Duration(2).sleep();
        ros::spinOnce();
        pub.publish(cloud_msg);
        std::cout << "published" << std::endl;
    }

    */

    return(0);

}
