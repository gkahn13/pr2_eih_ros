#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/PCLPointCloud2.h>

using namespace std;

int main(int argc, char** argv) {
  string infile = argv[1];
  string outfile = argv[2];

  //pcl::PointCloud2::Ptr input_cloud_2 (new pcl::PointCloud2);
  pcl::PointCloud<pcl::Boundary>::Ptr input_cloud (new pcl::PointCloud<pcl::Boundary>);

  if (pcl::io::loadPCDFile<pcl::Boundary>(infile, *input_cloud) == -1) {
    cout << "Could not find input file." << endl;
    exit(1);
  }

  //pcl::io::loadPCDFile<pcl::Boundary>(infile, *input_cloud);

  pcl::PointCloud<pcl::Boundary>::Ptr input_cloud_boundary (new pcl::PointCloud<pcl::Boundary>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> output = pcl::PointCloud<pcl::PointXYZ>();

  pcl::io::loadPCDFile<pcl::Boundary>(infile, *input_cloud_boundary);
  pcl::io::loadPCDFile<pcl::PointXYZ>(infile, *input_cloud_xyz);

  pcl::PointCloud<pcl::Boundary>::iterator boundary_iter;
  pcl::PointCloud<pcl::PointXYZ>::iterator xyz_iter;
  for (boundary_iter = input_cloud_boundary->begin(), xyz_iter = input_cloud_xyz->begin();
       boundary_iter != input_cloud_boundary->end();
       boundary_iter++, xyz_iter++) {
    pcl::Boundary current = *boundary_iter;
    if (current.boundary_point) {
      output.push_back(*xyz_iter);
    }
  }

  //fromPCLPointCloud2(*input_cloud_2, *input_cloud_xyz);
  
  pcl::io::savePCDFileASCII(outfile, output);

}
