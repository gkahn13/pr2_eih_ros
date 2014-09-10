#include <pcl_utils/occluded_region_finder.h>
#include <pcl/common/transforms.h>
#include <pcl_utils/timer.h>
#include <pcl_utils/plane_recognition.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

namespace occluded_region_finder
{

Eigen::Matrix3f calculate_rotation(Eigen::Vector3f A, Eigen::Vector3f B)
{
	Eigen::Matrix3f G = Eigen::Matrix3f::Zero();
	G(0, 0) = A.dot(B);
	G(0, 1) = -A.cross(B).norm();
	G(1, 0) = A.cross(B).norm();
	G(1, 1) = A.dot(B);
	G(2, 2) = 1;

	Eigen::Matrix3f Finv = Eigen::Matrix3f::Zero();
	Finv.col(0) = A;
	Finv.col(1) = B - A.dot(B) * A;
	Finv.col(1) = Finv.col(1) / Finv.col(1).norm();
	Finv.col(2) = B.cross(A);
	return Finv * G * Finv.inverse();
}

void publish_graspable(pcl::PointCloud<pcl::PointXYZ>::Ptr zero_crossing_cloud, pcl::ModelCoefficients::Ptr plane_coeff, ros::Publisher object_points_pub) {
	float min_x, max_x, min_y, max_y, min_z, max_z;
	ros::param::param<float>("/occlusion_parameters/min_x", min_x, 0);
	ros::param::param<float>("/occlusion_parameters/max_x", max_x, 2);
	ros::param::param<float>("/occlusion_parameters/min_y", min_y, 0);
	ros::param::param<float>("/occlusion_parameters/max_y", max_y, 2);
	ros::param::param<float>("/occlusion_parameters/min_z", min_z, 0);
	ros::param::param<float>("/occlusion_parameters/max_z", max_z, 2);

	float table_cutoff;
	ros::param::param<float>("/occlusion_parameters/table_cutoff_above", table_cutoff, 0.005f);
	pcl::PointCloud<pcl::PointXYZ> new_points;
	tf::TransformListener listener;
	tf::StampedTransform tf_transform;
	listener.waitForTransform("/kinfu_frame", "/base_link", ros::Time(0), ros::Duration(5));
	listener.lookupTransform("/kinfu_frame", "/base_link", ros::Time(0), tf_transform);
	Eigen::Affine3d transform_affine;
	tf::transformTFToEigen(tf_transform, transform_affine);
	for (pcl::PointCloud<pcl::PointXYZ>::iterator iter = zero_crossing_cloud->begin();
			iter != zero_crossing_cloud->end(); iter++)
	{
		if (iter->x >= min_x && iter->x <= max_x &&
				iter->y >= min_y && iter->y <= max_y &&
				iter->z >= min_z && iter->z <= max_z) {
			if (plane_coeff->values[0] * iter->x + plane_coeff->values[1] * iter->y + plane_coeff->values[2] * iter->z + plane_coeff->values[3] - table_cutoff >= 0) {
				pcl::PointXYZ new_point;
				new_point = pcl::transformPoint(*iter, transform_affine.inverse());
				new_points.push_back(new_point);
			}
		}
	}

	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(new_points, cloud_msg);
	cloud_msg.header.stamp = ros::Time::now();
	cloud_msg.header.frame_id = "/base_link";
	object_points_pub.publish(cloud_msg);
}


Eigen::Vector3f calculate_corner(Eigen::Vector3f new_position, pcl::PointXYZ max_point_OBB, Eigen::Matrix3f rotational_matrix_OBB, int a1, int a2, int min_direction)
{
	Eigen::Matrix3f id = Eigen::Matrix3f::Identity();
	Eigen::Vector3f corner;
	if (min_direction == 2)
	{
		corner = new_position + a1 * rotational_matrix_OBB * id.col(0) * (min_direction != 0) * (max_point_OBB.x) +
				a2 * rotational_matrix_OBB * id.col(1) * (min_direction != 1) * (max_point_OBB.y) +
				rotational_matrix_OBB * id.col(2) * (min_direction != 2) * (max_point_OBB.z);
	}

	if (min_direction == 1)
	{
		corner = new_position + a1 * rotational_matrix_OBB * id.col(0) * (min_direction != 0) * (max_point_OBB.x) +
				rotational_matrix_OBB * id.col(1) * (min_direction != 1) * (max_point_OBB.y) +
				a2 * rotational_matrix_OBB * id.col(2) * (min_direction != 2) * (max_point_OBB.z);
	}

	if (min_direction == 0)
	{
		corner = new_position + rotational_matrix_OBB * id.col(0) * (min_direction != 0) * (max_point_OBB.x) +
				a1 * rotational_matrix_OBB * id.col(1) * (min_direction != 1) * (max_point_OBB.y) +
				a2 * rotational_matrix_OBB * id.col(2) * (min_direction != 2) * (max_point_OBB.z);
	}

	return corner;

}

Eigen::Vector2i calculate_face(pcl::PointXYZ min_point_OBB, pcl::PointXYZ max_point_OBB, Eigen::Vector3f position, Eigen::Matrix3f rotational_matrix_OBB, int j, visualization_msgs::MarkerArrayPtr markers,
		pcl_utils::OccludedRegion* occ_message, std::vector<Eigen::Vector3f> *corners, bool rotate_box)
{

	Eigen::Vector3f weights (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
	Eigen::Vector3f position_vector = position / position.norm();

	int direction = 0;
	double max_dot_product = -INFINITY;
	for (int i = 0; i < 3; i++)
	{
		double current_dot_product = position_vector.dot(rotational_matrix_OBB.col(i));
		if (current_dot_product > max_dot_product) {
			direction = i;
			max_dot_product = current_dot_product;
		}
	}

	Eigen::Matrix3f U;
	if (rotate_box) {
		U = occluded_region_finder::calculate_rotation(rotational_matrix_OBB.col(direction), position_vector);
	} else {
		U = Eigen::Matrix3f::Identity();
	}

	Eigen::Matrix3f new_vectors = U * rotational_matrix_OBB;

	Eigen::Vector3f front_face_center = position - weights(direction) * new_vectors.col(direction);

	Eigen::Quaternionf quat(new_vectors);

	for (int a1 = -1; a1 <= 1; a1 = a1 + 2)
	{
		for (int a2 = -1 * a1; a1 * a2 <= 1; a2 = a2 + a1 * 2)
		{
			Eigen::Vector3f corner;
			corner = calculate_corner(front_face_center, max_point_OBB, new_vectors, a1, a2, direction);

			//            std::cout << "corner: " << std::endl << corner << std::endl;

			geometry_msgs::Point32 corner_point;
			corner_point.x = corner(0);
			corner_point.y = corner(1);
			corner_point.z = corner(2);
			occ_message->front_face.points.push_back(corner_point);
			corners->push_back(corner);
		}
	}


	visualization_msgs::Marker marker;
	marker.header.frame_id = "/camera_rgb_optical_frame";
	marker.header.stamp = ros::Time::now();
	marker.id = j + 1000;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = front_face_center(0);
	marker.pose.position.y = front_face_center(1);
	marker.pose.position.z = front_face_center(2);
	marker.pose.orientation.x = quat.x();
	marker.pose.orientation.y = quat.y();
	marker.pose.orientation.z = quat.z();
	marker.pose.orientation.w = quat.w();
	marker.scale.x = (direction != 0) * (max_point_OBB.x - min_point_OBB.x);
	marker.scale.y = (direction != 1) * (max_point_OBB.y - min_point_OBB.y);
	marker.scale.z = (direction != 2) * (max_point_OBB.z - min_point_OBB.z);
	marker.color.a = 1;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;


	markers->markers.push_back(marker);

	//Eigen::Vector2i output(min_direction, forward_back);
	Eigen::Vector2i output(direction, -1);
	return output;
}


void find_occluded_regions(std::vector<float> tsdf_distances, std::vector<short> tsdf_weights, pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr, Eigen::Matrix4d transformation_matrix, bool saving, std::string outfile,
		ros::Publisher markers_pub, ros::Publisher points_pub, ros::Publisher regions_pub, ros::Publisher plane_pub, ros::Publisher object_points_pub, ros::Publisher plane_points_pub) //,
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
	Timer timer2 = Timer();
	Timer timer3 = Timer();

	Timer_tic(&timer);
	tsdf_converter::convert_tsdf(tsdf_distances, tsdf_weights, zero_crossing_cloud, foreground_cloud, inverse_cloud);
	std::cout << "convert tsdf: " << Timer_toc(&timer) << std::endl;

	std::cout << "converted tsdf vectors" << std::endl;
	std::cout << "zero crossing: " << zero_crossing_cloud->width << std::endl;
	std::cout << "foreground: " << foreground_cloud->width << std::endl;
	std::cout << "inverse crossing: " << inverse_cloud->width << std::endl;

	// fit a plane to the zero-crossing points to find a table top
	pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr unused(new pcl::PointIndices);
	plane_recognition::calculate_plane(zero_crossing_cloud, unused, plane_coeff, plane_pub, markers, plane_points_pub); // was zero_crossing_cloud

	occluded_region_finder::publish_graspable(current_cloud_ptr, plane_coeff, object_points_pub); // was zero_crossing_cloud

	Timer_tic(&timer);

	std::vector<pcl::PointCloud<pcl::PointXYZ> >* clusters = new std::vector<pcl::PointCloud<pcl::PointXYZ> >;
	int num_plane_clusters = cluster_extraction::extract_clusters(zero_crossing_cloud, clusters, plane_points_pub);
	std::cout << "number of planar clusters: " << num_plane_clusters << std::endl;
	std::cout << "number of regular clusters: " << clusters->size() - num_plane_clusters << std::endl;

	std::cout << "cluster extraction: " << Timer_toc(&timer) << std::endl;

	Timer_tic(&timer);

	//    std::cout << "extracted positive clusters" << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr projected_inverse(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_inverse(new pcl::PointCloud<pcl::PointXYZ>);


	std::vector<Eigen::Matrix<double, 4, 1> > means;
	std::vector<Eigen::Matrix3d> covariances;

	Eigen::Matrix<double, 4, 1> mean;
	Eigen::Matrix3d covariance;

	pcl_utils::OccludedRegionArray regions;

	int j = 1;
	std::vector<pcl::PointCloud<pcl::PointXYZ> >::iterator cluster_iter;
	std::cout << "number of clusters: " << clusters->size() << std::endl;
	for (cluster_iter = clusters->begin(); cluster_iter != clusters->end(); cluster_iter++)
	{
		Timer_tic(&timer3);
		if (j == 1 || true)
		{
			//            std::cout << "press enter to continue" << std::endl;;
			//            std::string unused;
			//            getline(cin, unused);
			std::cout << std::endl << "cluster: " << j << std::endl;
			pcl::PointCloud<pcl::PointXYZ>::Ptr occluded_region(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_occluded_region(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_current_cloud(new pcl::PointCloud<pcl::PointXYZ>);

			pcl_utils::OccludedRegion region;

			*current_cloud = *cluster_iter;
			std::cout << "number of points: " << current_cloud->size() << std::endl;

			pcl::transformPointCloud(*current_cloud, *transformed_current_cloud, transformation_matrix);

			// Create the filtering object: downsample the dataset using a leaf size of 1cm
			//            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
			//            pcl::VoxelGrid<pcl::PointXYZ> vg;
			//            vg.setInputCloud (transformed_current_cloud);
			//            float leaf_size = 0.05f;
			//            vg.setLeafSize (leaf_size, leaf_size, leaf_size);
			//            vg.filter (*cloud_filtered);

			Timer_tic(&timer2);
			pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
			//        feature_extractor.setInputCloud (transformed_occluded_region);
			feature_extractor.setInputCloud(transformed_current_cloud);
			//            feature_extractor.setInputCloud(cloud_filtered);
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

			bool face_use_eigenvalues;
			ros::param::param<bool>("/occlusion_parameters/face_use_eigenvalues", face_use_eigenvalues, false);

			if (face_use_eigenvalues) {
				min_point_OBB.x = -major_value;
				min_point_OBB.y = -middle_value;
				min_point_OBB.z = -minor_value;
				min_point_OBB.x = major_value;
				min_point_OBB.y = middle_value;
				min_point_OBB.z = minor_value;
			}

			std::cout << "cluster feature extraction: " << Timer_toc(&timer2) << std::endl;
			Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
			std::vector<Eigen::Vector3f> corners;

			Timer_tic(&timer2);
			Eigen::Vector2i directions = calculate_face(min_point_OBB, max_point_OBB, position, rotational_matrix_OBB, j, markers, &region, &corners, j > num_plane_clusters);
			std::cout << "corners size: " << corners.size() << std::endl;
			std::cout << "calculate front face: " << Timer_toc(&timer2) << std::endl;


			Timer_tic(&timer2);
			*occluded_region = cluster_projection::calculate_occluded(*current_cloud, inverse_cloud, zero_crossing_cloud, transformation_matrix, transformed_inverse, projected_inverse, plane_coeff,
					directions(0), directions(1), min_point_OBB, max_point_OBB, position, rotational_matrix_OBB, markers, corners, plane_pub);
			std::cout << "cluster projection: " << Timer_toc(&timer2) << std::endl;
			std::cout << "occluded_region size: " << occluded_region->size() << std::endl;

			if (occluded_region->size() > 0) // TODO: filter based on number of points?
			{
				//                std::cout << "occluded region size: " << occluded_region->size() << std::endl;
				pcl::transformPointCloud(*occluded_region, *transformed_occluded_region, transformation_matrix);

				//        sensor_msgs::PointCloud2 published_cloud;
				//        toROSMsg(*zero_crossing_cloud, published_cloud);
				//        published_cloud.header.frame_id = "/camera_rgb_optical_frame";
				//        published_cloud.header.stamp = ros::Time(0);
				//        points_pub.publish(published_cloud);

				Timer_tic(&timer2);
				pcl::compute3DCentroid(*transformed_occluded_region, mean);
				//        pcl::computeCovarianceMatrix(*transformed_occluded_region, mean, covariance);
				pcl::computeCovarianceMatrixNormalized(*transformed_occluded_region, mean, covariance); // ????
				std::cout << "calculate mean/cov: " << Timer_toc(&timer2) << std::endl;


				means.push_back(mean);
				covariances.push_back(covariance);

				region.gaussian.mean.x = mean(0);
				region.gaussian.mean.y = mean(1);
				region.gaussian.mean.z = mean(2);

				for (int x = 0; x < covariance.rows(); x++)
				{
					for (int y = 0; y < covariance.cols(); y++)
					{
						region.gaussian.covariance.push_back(covariance(x, y));
					}
				}




				//        std::cout << mean << std::endl;
				//        std::cout << "extremes: " << std::endl << PointCloudVoxelGrid::calculate_extremes(transformed_occluded_region) << std::endl;
				//        std::cout << covariance << std::endl;


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



				//        viewer->addPointCloud<pcl::PointXYZ> (transformed_current_cloud, ss.str());
				if (mass_center.norm() < mean.norm())
				{

					//direction = 20;
					//            viewer->addPointCloud<pcl::PointXYZ> (transformed_occluded_region, ss2.str());


					Timer_tic(&timer2);
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


					std::cout << "occluded region feature extraction: " << Timer_toc(&timer2) << std::endl;

					if (major_value != 0 && middle_value != 0 && minor_value != 0)
					{

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
						marker.scale.x = isnanf(2 * sqrt(major_value)) ? 0 : (2 * sqrt(major_value));
						marker.scale.y = isnanf(2 * sqrt(middle_value)) ? 0 : (2 * sqrt(middle_value)); //(min_direction != 1) * (max_point_OBB.y - min_point_OBB.y);
						marker.scale.z = isnanf(2 * sqrt(minor_value)) ? 0 : (2 * sqrt(minor_value)); //(min_direction != 2) * (max_point_OBB.z - min_point_OBB.z);
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

						//                    std::cout << "mass center: " << mass_center << std::endl;

						markers->markers.push_back(marker);


						//            std::cout << "point: " << transformed_occluded_region->at(0) << std::endl;

						sensor_msgs::PointCloud2 published_cloud;
						toROSMsg(*transformed_occluded_region, published_cloud);
						//            std::cout << "number of points: " << transformed_occluded_region->size() << std::endl;
						published_cloud.header.frame_id = "/camera_rgb_optical_frame";
						published_cloud.header.stamp = ros::Time::now();
						points_pub.publish(published_cloud);
						ros::spinOnce();

						// also publish the clusters themselves. TODO: make this a separate publisher?
						toROSMsg(*transformed_current_cloud, published_cloud);
						published_cloud.header.frame_id = "/camera_rgb_optical_frame";
						published_cloud.header.stamp = ros::Time::now();
						points_pub.publish(published_cloud);
						ros::spinOnce();


						region.points = published_cloud;

						regions.regions.push_back(region);
					} // all eigenvalues non-zero

				}
				else
				{
					std::cout << "cloud " << j << "isn't behind its occlusion" << std::endl;
					std::cout << markers->markers.size() << std::endl;
					markers->markers.pop_back();
					std::cout << markers->markers.size() << std::endl;
				}
			}
			else
			{
				std::cout << "cloud " << j << "is too small" << std::endl;
				std::cout << markers->markers.size() << std::endl;
				markers->markers.pop_back();
				std::cout << markers->markers.size() << std::endl;
			}
		}

		//        std::cout << "cluster " << j << " complete" << std::endl << std::endl;
		std::cout << "cluster " << j << " total time: " << Timer_toc(&timer3) << std::endl;

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
		pcl::PointCloud<pcl::PointXYZ> transformed_zero_crossing;
		pcl::transformPointCloud(*zero_crossing_cloud, transformed_zero_crossing, transformation_matrix);
		pcl::io::savePCDFileASCII(outfile + "_zero.pcd", transformed_zero_crossing);
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


