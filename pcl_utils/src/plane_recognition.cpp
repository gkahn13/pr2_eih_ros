#include <plane_recognition.h>
#include <pcl/io/pcd_io.h>

using namespace std;

namespace plane_recognition
{
void calculate_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients)
{
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_MLESAC);
//    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    // input cloud
    seg.setInputCloud (cloud);

    seg.segment (*inliers, *coefficients);

//    // get the method, in order to tweak parameters
//    pcl::SACSegmentation<pcl::PointXYZ>::SampleConsensusPtr method = seg.getMethod();
//    cout << "max number of iterations: " << method->getMaxIterations() << endl;

}
}


// TODO: Compiling was getting mad about this main. I should move it to another file, eventually.
/*
int
main (int argc, char** argv)
{
    string infile = argv[1];
    string outfile = argv[2];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(infile, *cloud);
    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    plane_recognition::calculate_plane(cloud, inliers, coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    pcl::PointCloud<pcl::PointXYZ> new_cloud;

    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
        //std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
        //                                       << cloud->points[inliers->indices[i]].y << " "
        //                                        << cloud->points[inliers->indices[i]].z << std::endl;
        int current_index = inliers->indices[i];
        pcl::PointXYZ current_point = cloud->points[current_index];
        // hardcoded value for end of table
        if (current_point.z < 1.1)
        {
            new_cloud.push_back(current_point);
        }
    }

    pcl::io::savePCDFileASCII(outfile, new_cloud);

    return (0);

}
*/
