#include <pointcloud_voxel_grid.h>

PointCloudVoxelGrid::PointCloudVoxelGrid(CloudType::Ptr cloud, double resolution) {
    res = resolution;
    Eigen::Matrix<double, 6, 1> extremes = calculate_extremes(cloud);
    minimum = extremes.block<3, 1>(0, 0);
    maximum = extremes.block<3, 1>(3, 0);

    x_size = ceil((extremes(3) - extremes(0)) / resolution);
    y_size = ceil((extremes(4) - extremes(1)) / resolution);
    z_size = ceil((extremes(5) - extremes(2)) / resolution);

    std::cout << "sizes: " << std::endl;
    std::cout << x_size << std::endl;
    std::cout << y_size << std::endl;
    std::cout << z_size << std::endl;

    std::cout << minimum << std::endl;
    std::cout << maximum << std::endl;



    array = Eigen::VectorXi(x_size * y_size * z_size);

    set_all(NOT_VISITED);

    walk_cloud(cloud);

}

/** Walks through the cloud, stuffing each point into the voxel grid. */
void PointCloudVoxelGrid::walk_cloud(CloudType::Ptr cloud) {
    CloudType::iterator cloud_iter;
    for (cloud_iter = cloud->begin(); cloud_iter != cloud->end(); cloud_iter++) {
        set(*cloud_iter, VISITED);
    }
}

/** Goes through the grid, creating a cloud of points that have not been visited. */
PointCloudVoxelGrid::CloudType PointCloudVoxelGrid::get_inverse_cloud() {
    CloudType output_cloud;
    // TODO: reverse the order of these loops - i.e. x, y, z
    for (int z = 0; z < z_size; z++) {
        for (int y = 0; y < y_size; y++) {
            for (int x = 0; x < x_size; x++) {
                if (!get(x, y, z)) {
                    output_cloud.push_back(voxel_to_point(voxel(x, y, z)));
                }
            }
        }
    }

    return output_cloud;

}

/** Converts a point to an integer voxel. */
PointCloudVoxelGrid::voxel PointCloudVoxelGrid::point_to_voxel(PointType point) {
    Eigen::Vector3d point_vector(point.x, point.y, point.z);
    Eigen::Vector3d transformed = (point_vector - minimum) / res;
//    std:: << "transforming!" << std::endl;
//    std::cout << point_vector << std::endl;
//    std::cout << transformed << std::endl;
    return Eigen::Vector3i(floor(transformed(0)), floor(transformed(1)), floor(transformed(2))); // we'll see if this works
}

/** Converts a voxel to a point. */
PointCloudVoxelGrid::PointType PointCloudVoxelGrid::voxel_to_point(voxel v) {
    PointType output;
    Eigen::Vector3d transformed_d = v.cast<double>() * res + minimum.cast<double>();
    //voxel transformed(floor(transformed_d(0)), floor(transformed_d(1)), floor(transformed_d(2)));
    output.x = transformed_d(0);
    output.y = transformed_d(1);
    output.z = transformed_d(2);

    return output;
}

Eigen::Matrix<double, 6, 1> PointCloudVoxelGrid::calculate_extremes(CloudType::Ptr cloud) {
    double min_x = INFINITY, min_y = INFINITY, min_z = INFINITY,
    max_x = -INFINITY, max_y = -INFINITY, max_z = -INFINITY;
    CloudType::iterator cloud_iter;
    for (cloud_iter = cloud->begin(); cloud_iter != cloud->end(); cloud_iter++) {
        PointType current = *cloud_iter;
        if (current.x > max_x) {
            max_x = current.x;
        }

        if (current.x < min_x) {
            min_x = current.x;
        }

        if (current.y > max_y) {
            max_y = current.y;
        }

        if (current.y < min_y) {
            min_y = current.y;
        }

        if (current.z > max_z) {
            max_z = current.z;
        }

        if (current.z < min_z) {
            min_z = current.z;
        }
    }

    Eigen::Matrix<double, 6, 1> extremes;
    extremes << min_x,
                min_y,
                min_z,
                max_x,
                max_y,
                max_z;
    return extremes;
}
