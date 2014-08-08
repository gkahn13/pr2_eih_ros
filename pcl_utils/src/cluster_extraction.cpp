#include <cluster_extraction.h>

namespace cluster_extraction {

std::vector<pcl::PointCloud<pcl::PointXYZ> > extract_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double cluster_tolerance, int min_cluster_size, int max_cluster_size) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZ> > cloud_vector;
    //pcl::PCDWriter writer;
    //int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cloud_vector.push_back(*cloud_cluster);
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

        //functionality moved to occluded_region_finder.cpp
//        std::stringstream ss;
//        ss << "cloud_cluster_" << j << ".pcd";
//        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
//        j++;

    }

    return cloud_vector;
}
}

// now handled in occluded_region_finder.cpp

//int
//main (int argc, char** argv)
//{
//    // Read in the cloud data
//    pcl::PCDReader reader;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    std::string file = "table_scene_lms400.pcd";
//    if (argc > 1)
//    {
//        file = argv[1];
//    }
//    double cluster_tolerance = 0.02; // 2cm
//    if (argc > 2)
//    {
//        cluster_tolerance = std::atof(argv[2]);
//    }
//    int min_cluster_size = 100;
//    if (argc > 3) {
//        min_cluster_size = std::atoi(argv[3]);
//    }
//    int max_cluster_size = 25000;
//    if (argc > 4) {
//        max_cluster_size = std::atoi(argv[4]);
//    }
//    reader.read (file, *cloud);
//
//    std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters = cluster_extraction::extract_clusters(cloud, cluster_tolerance, min_cluster_size, max_cluster_size);
//
//    return (0);
//}
