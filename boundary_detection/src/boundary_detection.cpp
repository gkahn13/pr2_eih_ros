#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>

using namespace std;

int main(int argc, char** argv) {
  string infile = argv[1];
  string outfile = argv[2];
  if (argc > 3) {
    string outplyfile = argv[3];
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr computed_normal_cloud (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointNormal> combined_cloud;
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(infile, *input_xyz_cloud) == -1) {
    cout << "Could not find input file." << endl;
    exit(1);
  }

  cout << "loaded file" << endl;
  
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> estimator;
  estimator.setInputCloud(input_xyz_cloud);
  
  // setup the search tree for normal estimation
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  estimator.setSearchMethod(tree);
  // specify the search radius
  estimator.setRadiusSearch(0.03);
  // calculate normals

  cout << "about to compute normals" << endl;
  estimator.compute(*computed_normal_cloud);
  cout << "normals computed, about to merge clouds" << endl;

  // merge the pointclouds
  pcl::concatenateFields(*input_xyz_cloud, *computed_normal_cloud, combined_cloud);

  cout << "clouds merged, saving" << endl;

  pcl::io::savePCDFileASCII(outfile, combined_cloud);

  // TODO: write ply file?

}
