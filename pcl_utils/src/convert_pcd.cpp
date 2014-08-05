#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using namespace std;

int main(int argc, char** argv)
{
    string infile = argv[1];
    string outfile = argv[2];

    std::string option = "xyz";
    if (argc > 3) {
        option = argv[3];
    }

    typedef pcl::PointXYZ current_point_type;
    if (option == "xyz") {
        typedef pcl::PointXYZ current_point_type;
    } else {
        typedef pcl::PointNormal current_point_type;
    }

    pcl::PointCloud<current_point_type>::Ptr input_cloud (new pcl::PointCloud<current_point_type>);

    if (pcl::io::loadPCDFile<current_point_type>(infile, *input_cloud) == -1)
    {
        cout << "Could not find input file." << endl;
        exit(1);
    }

    pcl::io::savePLYFileASCII(outfile, *input_cloud);

}
