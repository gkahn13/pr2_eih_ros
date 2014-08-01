#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using namespace std;

int main(int argc, char** argv) {
  string infile = argv[1];
  string outfile = argv[2];

  pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud (new pcl::PointCloud<pcl::PointNormal>);
  
  if (pcl::io::loadPCDFile<pcl::PointNormal>(infile, *input_cloud) == -1) {
    cout << "Could not find input file." << endl;
    exit(1);
  }

  pcl::io::savePLYFileASCII(outfile, *input_cloud);

}
