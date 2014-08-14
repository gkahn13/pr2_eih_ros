#include <pcl_utils/occluded_region_finder.h>
#include <pcl/common/transforms.h>
#include <pcl_utils/timer.h>

namespace occluded_region_finder
{


Eigen::Vector3f calculate_corner(Eigen::Vector3f new_position, pcl::PointXYZ max_point_OBB, Eigen::Matrix3f rotational_matrix_OBB, int a1, int a2, int min_direction)
{
    Eigen::Vector3f corner;
    if (min_direction == 2) {
        corner = new_position + a1 * rotational_matrix_OBB * Eigen::Matrix3f::Identity().block<3, 1>(0,0) * (min_direction != 0) * (max_point_OBB.x) +
                 a2 * rotational_matrix_OBB * Eigen::Matrix3f::Identity().block<3, 1>(0,1) * (min_direction != 1) * (max_point_OBB.y) +
                 rotational_matrix_OBB * Eigen::Matrix3f::Identity().block<3, 1>(0,2) * (min_direction != 2) * (max_point_OBB.z);
    }

    if (min_direction == 1) {
        corner = new_position + a1 * rotational_matrix_OBB * Eigen::Matrix3f::Identity().block<3, 1>(0,0) * (min_direction != 0) * (max_point_OBB.x) +
                 rotational_matrix_OBB * Eigen::Matrix3f::Identity().block<3, 1>(0,1) * (min_direction != 1) * (max_point_OBB.y) +
                 a2 * rotational_matrix_OBB * Eigen::Matrix3f::Identity().block<3, 1>(0,2) * (min_direction != 2) * (max_point_OBB.z);
    }

    if (min_direction == 0) {
        corner = new_position + rotational_matrix_OBB * Eigen::Matrix3f::Identity().block<3, 1>(0,0) * (min_direction != 0) * (max_point_OBB.x) +
                 a1 * rotational_matrix_OBB * Eigen::Matrix3f::Identity().block<3, 1>(0,1) * (min_direction != 1) * (max_point_OBB.y) +
                 a2 * rotational_matrix_OBB * Eigen::Matrix3f::Identity().block<3, 1>(0,2) * (min_direction != 2) * (max_point_OBB.z);
    }

    return corner;

}

int calculate_face(pcl::PointXYZ min_point_OBB, pcl::PointXYZ max_point_OBB, Eigen::Vector3f position, Eigen::Matrix3f rotational_matrix_OBB, int j, visualization_msgs::MarkerArrayPtr markers,
                   pcl_utils::OccludedRegion* occ_message)
{
    Eigen::Matrix3f adder = Eigen::Matrix3f::Zero();
    Eigen::Vector3f min_vector(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f max_vector(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);

    Eigen::Quaternionf quat (rotational_matrix_OBB);

    double min_norm = INFINITY;
    int min_direction = 0;
    int forward_back = -1;
    for (int i = 0; i < 3; i++)
    {

        pcl::PointXYZ current_point;

        adder(i, i) = 1;

        std::stringstream ss3;
        ss3 << "sphere min corner" << j << i;

        double current_norm;
        Eigen::Vector3f current_vector = rotational_matrix_OBB * adder * max_vector + position;
        current_norm = current_vector.norm();
        current_point.x = current_vector(0);
        current_point.y = current_vector(1);
        current_point.z = current_vector(2);
        //viewer->addSphere(current_point, 0.005, ss3.str());
        if (current_norm < min_norm)
        {
            min_norm = current_norm;
            min_direction = i;
            forward_back = 1;
        }


        current_vector = position - rotational_matrix_OBB * adder * max_vector; // not sure why this is minus, but it works
        current_point.x = current_vector(0);
        current_point.y = current_vector(1);
        current_point.z = current_vector(2);

        current_norm = current_vector.norm();
        ss3 << "sphere max corner" << j << i;
        //viewer->addSphere(current_point, 0.005, ss3.str());

        if (current_norm < min_norm)
        {
            min_norm = current_norm;
            min_direction = i;
            forward_back = -1;
        }

        adder(i, i) = 0;


    }

    adder(min_direction, min_direction) = 1;

    Eigen::Vector3f new_position = position + forward_back * rotational_matrix_OBB * adder * max_vector;

    std::stringstream ss2;
    int i = 0;
    for (int a1 = -1; a1 <= 1; a1 = a1 + 2) {
        for (int a2 = -1 * a1; a1 * a2 <= 1; a2 = a2 + a1 * 2) {
            Eigen::Vector3f corner;
            corner = calculate_corner(new_position, max_point_OBB, rotational_matrix_OBB, a1, a2, min_direction);

//            std::cout << "corner: " << std::endl << corner << std::endl;

            ss2.str("");
            geometry_msgs::Point32 corner_point;
            corner_point.x = corner(0);
            corner_point.y = corner(1);
            corner_point.z = corner(2);
            occ_message->front_face.points.push_back(corner_point);

//                ss2 << "corner " << j << " " << i;
//                viewer->addSphere(corner_point, 0.01, ss2.str());
//                std::cout << "corner: " << std::endl << corner << std::endl;
//                ss2.str("");
//                ss2 << "point " << j << " " << i;
//                viewer->addText3D(ss2.str(), corner_point, 0.005, 1, 1, 1, ss2.str());

            i++;
        }
    }


    ss2 << "face" << j;
//    viewer->addCube (new_position, quat, (min_direction != 0) * (max_point_OBB.x - min_point_OBB.x), (min_direction != 1) * (max_point_OBB.y - min_point_OBB.y), (min_direction != 2) * (max_point_OBB.z - min_point_OBB.z), ss2.str());


    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time::now();
    marker.id = j + 1000;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = new_position(0);
    marker.pose.position.y = new_position(1);
    marker.pose.position.z = new_position(2);
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.scale.x = (min_direction != 0) * (max_point_OBB.x - min_point_OBB.x);
    marker.scale.y = (min_direction != 1) * (max_point_OBB.y - min_point_OBB.y);
    marker.scale.z = (min_direction != 2) * (max_point_OBB.z - min_point_OBB.z);
    marker.color.a = 1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // later, write the below function to make things cleaner
//    visualization_msgs::Marker marker = create_marker("/camera_rgb_optical_frame", ros::Time(0), j + 1000, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, new_position(0), new_position(1), new_position(2),
//                                                      quat.x(), quat.y(), quat.z(), quat.w(), (min_direction != 0) * (max_point_OBB.x - min_point_OBB.x), marker.scale.y = (min_direction != 1) * (max_point_OBB.y - min_point_OBB.y),
//                                                      marker.scale.z = (min_direction != 2) * (max_point_OBB.z - min_point_OBB.z), 1, 0, 1, 0);

    markers->markers.push_back(marker);

    return min_direction;
}


void find_occluded_regions(std::vector<float> tsdf_distances, std::vector<short> tsdf_weights, Eigen::Matrix4d transformation_matrix, bool saving, std::string outfile, ros::Publisher markers_pub, ros::Publisher points_pub, ros::Publisher regions_pub) //,
                           //pcl::PointCloud<pcl::PointXYZ>::Ptr zero_crossing_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_cloud, PointCloudVoxelGrid::CloudType::Ptr inverse_cloud)
{

    std::cout << "entered occluded region finder" << std::endl;
    visualization_msgs::MarkerArrayPtr markers = visualization_msgs::MarkerArrayPtr(new visualization_msgs::MarkerArray);

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setBackgroundColor (0, 0, 0);
//    viewer->addCoordinateSystem (0.5, 0);
//    viewer->initCameraParameters ();

    pcl::PointCloud<pcl::PointXYZ>::Ptr zero_crossing_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);
    PointCloudVoxelGrid::CloudType::Ptr inverse_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);

    Timer timer = Timer();

    int jump = 1;
    double voxel_size = 0.02;
    double cluster_tolerance = 0.02;
    int min_cluster_size = 100;
    int max_cluster_size = 25000;

    Timer_tic(&timer);
    tsdf_converter::convert_tsdf(tsdf_distances, tsdf_weights, zero_crossing_cloud, foreground_cloud, inverse_cloud, jump, voxel_size);
    std::cout << "convert tsdf: " << Timer_toc(&timer) << std::endl;

    std::cout << "converted tsdf vectors" << std::endl;
    std::cout << "zero crossing: " << zero_crossing_cloud->width << std::endl;
    std::cout << "foreground: " << foreground_cloud->width << std::endl;
    std::cout << "inverse crossing: " << inverse_cloud->width << std::endl;

    Timer_tic(&timer);

    std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters = cluster_extraction::extract_clusters(zero_crossing_cloud, cluster_tolerance, min_cluster_size, max_cluster_size);

//    std::cout << "cluster extraction: " << Timer_toc(&timer) << std::endl;
    Timer_tic(&timer);

//    std::cout << "extracted positive clusters" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_inverse(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);


    std::vector<Eigen::Matrix<double, 4, 1> > means;
    std::vector<Eigen::Matrix3d> covariances;

    Eigen::Matrix<double, 4, 1> mean;
    Eigen::Matrix3d covariance;

    pcl_utils::OccludedRegionArray regions;

    int j = 1;
    std::vector<pcl::PointCloud<pcl::PointXYZ> >::iterator cluster_iter;
    for (cluster_iter = clusters.begin(); cluster_iter != clusters.end(); cluster_iter++)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr occluded_region(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_occluded_region(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_current_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl_utils::OccludedRegion region;

        *current_cloud = *cluster_iter;

        *occluded_region = cluster_projection::calculate_occluded(*current_cloud, inverse_cloud, zero_crossing_cloud, transformation_matrix, projected_inverse, plane_coeff);

        if (occluded_region->size() > 0) {
        std::cout << "occluded region size: " << occluded_region->size() << std::endl;
        pcl::transformPointCloud(*occluded_region, *transformed_occluded_region, transformation_matrix);
        pcl::transformPointCloud(*current_cloud, *transformed_current_cloud, transformation_matrix);

//        sensor_msgs::PointCloud2 published_cloud;
//        toROSMsg(*zero_crossing_cloud, published_cloud);
//        published_cloud.header.frame_id = "/camera_rgb_optical_frame";
//        published_cloud.header.stamp = ros::Time(0);
//        points_pub.publish(published_cloud);


        pcl::compute3DCentroid(*transformed_occluded_region, mean);
//        pcl::computeCovarianceMatrix(*transformed_occluded_region, mean, covariance);
        pcl::computeCovarianceMatrixNormalized(*transformed_occluded_region, mean, covariance); // ????


        means.push_back(mean);
        covariances.push_back(covariance);

        region.gaussian.mean.x = mean(0);
        region.gaussian.mean.y = mean(1);
        region.gaussian.mean.z = mean(2);

        for (int x = 0; x < covariance.rows(); x++) {
            for (int y = 0; y < covariance.cols(); y++) {
                region.gaussian.covariance.push_back(covariance(x, y));
            }
        }




//        std::cout << mean << std::endl;
//        std::cout << "extremes: " << std::endl << PointCloudVoxelGrid::calculate_extremes(transformed_occluded_region) << std::endl;
//        std::cout << covariance << std::endl;

        pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
//        feature_extractor.setInputCloud (transformed_occluded_region);
        feature_extractor.setInputCloud(transformed_current_cloud);
        feature_extractor.compute ();


        pcl::PointXYZ min_point_OBB;
        pcl::PointXYZ max_point_OBB;
        pcl::PointXYZ position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;

        feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        feature_extractor.getEigenValues (major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter (mass_center);

//        std::cout << "eigenvalues: " << major_value << ", " << middle_value << ", " << minor_value << std::endl;
//        std::cout << "eigenvectors: " << std::endl << major_vector << std::endl << middle_vector << std::endl << minor_vector << std::endl;


//        Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
//        std::cout << "eigen vectors (computed from covariance matrix):" << std::endl << eigen_solver.eigenvectors() << std::endl;
//        std::cout << "eigen values (computed from covariance matrix):" << std::endl << eigen_solver.eigenvalues() << std::endl;
//
//        major_value = std::real(eigen_solver.eigenvalues()(0));
//        middle_value = std::real(eigen_solver.eigenvalues()(1));
//        minor_value = std::real(eigen_solver.eigenvalues()(2));

//        major_vector.normalize();
//        middle_vector.normalize();
//        minor_vector.normalize();



        std::stringstream ss;
        ss << outfile << "_cloud_cluster_" << j << ".pcd";
        std::stringstream ss2;
        ss2 << outfile << "_occluded_region_" << j << ".pcd";

        if (saving)
        {
            pcl::io::savePCDFileASCII(ss.str(), *current_cloud);
            try
            {
                pcl::io::savePCDFileASCII(ss2.str(), *occluded_region);
            }
            catch (pcl::IOException)
            {
                std::cout << "couldn't find occluded region for cluster " << j << std::endl;
            }
        }


        Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);


//        viewer->addPointCloud<pcl::PointXYZ> (transformed_current_cloud, ss.str());
        if ((j == 4 || j == 5 || j == 6 || true) && mass_center.norm() < mean.norm())
        {

            int direction = calculate_face(min_point_OBB, max_point_OBB, position, rotational_matrix_OBB, j, markers, &region);
            //direction = 20;
//            viewer->addPointCloud<pcl::PointXYZ> (transformed_occluded_region, ss2.str());



            feature_extractor.setInputCloud (transformed_occluded_region);
//            feature_extractor.setInputCloud(transformed_current_cloud);
            feature_extractor.compute ();


            pcl::PointXYZ min_point_OBB;
            pcl::PointXYZ max_point_OBB;
            pcl::PointXYZ position_OBB;
            Eigen::Matrix3f rotational_matrix_OBB;
            float major_value, middle_value, minor_value;
            Eigen::Vector3f major_vector, middle_vector, minor_vector;
            Eigen::Vector3f mass_center;

            feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
            feature_extractor.getEigenValues (major_value, middle_value, minor_value);
            feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
            feature_extractor.getMassCenter (mass_center);

            Eigen::Matrix3f eigenvector_rotation;
            eigenvector_rotation.block<3,1>(0, 0) = major_vector;
            eigenvector_rotation.block<3,1>(0, 1) = middle_vector;
            eigenvector_rotation.block<3,1>(0, 2) = minor_vector;

            Eigen::Quaternion<float> eigen_quat(eigenvector_rotation);


            std::stringstream ss3;
            ss3 << "sphere" << j;
//            viewer->addSphere(position_OBB, 0.005, ss3.str());
            pcl::PointXYZ p;
            p.x = mass_center(0);
            p.y = mass_center(1);
            p.z = mass_center(2);
//            ss3 << "sphere max corner" << j;
//            viewer->addSphere(max_point_OBB, 0.005, ss3.str());
//            ss3 << "sphere min corner" << j;
//            viewer->addSphere(min_point_OBB, 0.005, ss3.str());
//            std::cout << "position: " << std::endl << position << std::endl;
//            std::cout << "min: " << std::endl << min_point_OBB << std::endl;
//            std::cout << "max: " << std::endl << max_point_OBB << std::endl;

            //Eigen::Quaternionf quat (rotational_matrix_OBB);

            visualization_msgs::Marker marker;
            marker.header.frame_id = "/camera_rgb_optical_frame";
            marker.header.stamp = ros::Time::now();
            //marker.header.seq = j;
            marker.id = j + 500;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = mass_center(0);
            marker.pose.position.y = mass_center(1);
            marker.pose.position.z = mass_center(2);
            marker.pose.orientation.x = eigen_quat.x();
            marker.pose.orientation.y = eigen_quat.y();
            marker.pose.orientation.z = eigen_quat.z();
            marker.pose.orientation.w = eigen_quat.w();
            marker.scale.x = 2 * sqrt(major_value);
            marker.scale.y = 2 * sqrt(middle_value); //(min_direction != 1) * (max_point_OBB.y - min_point_OBB.y);
            marker.scale.z = 2 * sqrt(minor_value); //(min_direction != 2) * (max_point_OBB.z - min_point_OBB.z);
            marker.color.a = 0.3;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;

            markers->markers.push_back(marker);

            //visualization_msgs::Marker marker;
            marker.header.frame_id = "/camera_rgb_optical_frame";
            marker.header.stamp = ros::Time::now();
            //marker.header.
            marker.id = j + 20000;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = mass_center(0);
            marker.pose.position.y = mass_center(1);
            marker.pose.position.z = mass_center(2);
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            markers->markers.push_back(marker);

//            std::cout << "point: " << transformed_occluded_region->at(0) << std::endl;

            sensor_msgs::PointCloud2 published_cloud;
            toROSMsg(*transformed_occluded_region, published_cloud);
//            std::cout << "number of points: " << transformed_occluded_region->size() << std::endl;
            published_cloud.header.frame_id = "/camera_rgb_optical_frame";
            published_cloud.header.stamp = ros::Time::now();
            points_pub.publish(published_cloud);
            ros::spinOnce();


            region.points = published_cloud;

            regions.regions.push_back(region);

        }
        }


//        std::cout << "cluster " << j << " complete" << std::endl << std::endl;

        j++;


    }

    regions.header.frame_id = "/camera_rgb_optical_frame";
    regions.header.stamp = ros::Time::now();

//    std::cout << "clusters finished" << std::endl;

//    while(!viewer->wasStopped())
//    {
//        viewer->spinOnce (100);
//        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }

    if (saving)
    {
        pcl::io::savePCDFileASCII(outfile + "_zero.pcd", *zero_crossing_cloud);
//        pcl::io::savePCDFileASCII(outfile + "_foreground.pcd", *foreground_cloud);
//        pcl::io::savePCDFileASCII(outfile + "_inverse.pcd", *inverse_cloud);
    }

    std::cout << "occlusion finding: " << Timer_toc(&timer) << std::endl;

    markers_pub.publish(*markers);
    ros::spinOnce();

   regions_pub.publish(regions);
   ros::spinOnce();

//    std::cout << "published" << std::endl;

//    std::cout << "done" << std::endl;
}

}


