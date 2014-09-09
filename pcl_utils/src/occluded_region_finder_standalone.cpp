#include <pcl_utils/occluded_region_finder.h>
#include <iostream>
#include <boost/program_options.hpp>
#include <pcl_utils/BoundingBox.h>

namespace po = boost::program_options;

int main(int argc, char** argv)
{

//    if (argc < 5)
//    {
//        std::cerr << "Not enough arguments! Please specify three input files and an output file prefix" << std::endl;
//        exit(1);
//    }

    std::string infile1, infile2, matrix_file, outfile;
    bool saving;

    po::options_description desc("Run the occluded region finder on a pointcloud.");
    desc.add_options()
    ("saving,s", "Whether the output should be saved.")
     ("distances-file,d", po::value<std::string >(), "The input file for kinfu distances.")
     ("weights-file,w", po::value<std::string >(), "The input file for kinfu weights.")
     ("matrix-file,m", po::value<std::string >(), "The input file for the transformation matrix.")
     ("output-file-prefix,o", po::value<std::string >()->default_value(""), "The prefix to be appended to the output files.");
     po::positional_options_description pos;
     pos.add("distances-file", 1);
     pos.add("weights-file", 1);
     pos.add("matrix-file", 1);
     pos.add("output-file-prefix", 1);

    po::variables_map opts;

    try
    {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), opts);
        po::notify(opts);
        saving = opts.count("saving");
        std::cout << "saving: " << saving << std::endl;
        infile1 = opts["distances-file"].as<std::string >();
        infile2 = opts["weights-file"].as<std::string >();
        matrix_file = opts["matrix-file"].as<std::string >();
        outfile = opts["output-file-prefix"].as<std::string >();
    }
    catch (std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        std::cout << desc << std::endl;
        return 1;
    }

    //int nRobots = opts["robots"].as<int>();




    ros::init(argc, argv, "occlusion_detection");
    //ros::Duration(2).sleep();
    ros::NodeHandle nh("~");
    ros::Publisher markers_pub;
    ros::Publisher points_pub;
    ros::Publisher regions_pub, plane_pub, object_points_pub, plane_points_pub;
    markers_pub = nh.advertise<visualization_msgs::MarkerArray> ("objects", 1);
    points_pub = nh.advertise<sensor_msgs::PointCloud2> ("occluded_points", 1);
    regions_pub = nh.advertise<pcl_utils::OccludedRegionArray> ("occluded_regions", 1);
    plane_pub = nh.advertise<pcl_utils::BoundingBox>("plane_bounding_box", 1);
    object_points_pub = nh.advertise<sensor_msgs::PointCloud2>("graspable_points", 1);
    plane_points_pub = nh.advertise<sensor_msgs::PointCloud2>("plane_points", 1);

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

    tsdf_converter::read_files(infile1.c_str(), infile2.c_str(), tsdf_distances, tsdf_weights);

//    std::cout << "press enter to start" << std::endl;
//    std::string unused;
//    getline(cin, unused);

    // TODO: must download current_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    occluded_region_finder::find_occluded_regions(*tsdf_distances, *tsdf_weights, current_points, transformation_matrix, saving, outfile, markers_pub, points_pub, regions_pub, plane_pub, object_points_pub, plane_points_pub);

    return 0;

}

