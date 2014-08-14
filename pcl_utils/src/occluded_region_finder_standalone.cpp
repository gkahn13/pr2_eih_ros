#include <pcl_utils/occluded_region_finder.h>

int main(int argc, char** argv)
{

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

    ros::init(argc, argv, "occlusion_detection");
    //ros::Duration(2).sleep();
    ros::NodeHandle nh("~");
    ros::Publisher markers_pub;
    ros::Publisher points_pub;
    ros::Publisher regions_pub;
    markers_pub = nh.advertise<visualization_msgs::MarkerArray> ("objects", 1);
    points_pub = nh.advertise<sensor_msgs::PointCloud2> ("occluded_points", 1);
    regions_pub = nh.advertise<pcl_utils::OccludedRegionArray> ("occluded_regions", 1);

    ros::spinOnce();


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

//    std::cout << "transformation matrix: " << std::endl << transformation_matrix << std::endl;

    std::vector<float>* tsdf_distances = new std::vector<float>;
    std::vector<short>* tsdf_weights = new std::vector<short>;

    tsdf_converter::read_files(infile1, infile2, tsdf_distances, tsdf_weights);

    occluded_region_finder::find_occluded_regions(*tsdf_distances, *tsdf_weights, transformation_matrix, saving, outfile, markers_pub, points_pub, regions_pub);

    return 0;

}

