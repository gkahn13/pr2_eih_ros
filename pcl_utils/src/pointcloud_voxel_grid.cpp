#include <pointcloud_voxel_grid.h>

PointCloudVoxelGrid::PointCloudVoxelGrid(CloudType cloud, double resolution) {
    Eigen::Vector6d extremes = calculate_extremes(cloud);
    minimum = extremes.block<3, 1>(0, 0);
    maximum = extremes.block<3, 1>(3, 0);

    x_size = ceil((extremes(0) - extremes(3)) / resolution);
    y_size = ceil((extremes(1) - extremes(4)) / resolution);
    z_size = ceil((extremes(2) - extremes(5)) / resolution);
    array = Eigen::VectorXi(x_size * y_size * z_size);

    set_all(NOT_VISITED);

    walk_cloud(cloud);

}

/** Walks through the cloud, stuffing each point into the voxel grid. */
void PointCloudVoxelGrid::walk_cloud(CloudType cloud) {
    CloudType::iterator cloud_iter;
    for (cloud_iter = cloud.begin(); cloud_iter != cloud.end(); cloud_iter++) {
        set(*cloud_iter, VISITED);
    }
}

/** Goes through the grid, creating a cloud of points that have not been visited. */
PointCloudVoxelGrid::CloudType PointCloudVoxelGrid::get_inverse_cloud() {
    CloudType output_cloud;
    // triple nested for loop here

}

/** Converts a point to an integer voxel. */
PointCloudVoxelGrid::voxel PointCloudVoxelGrid::point_to_voxel(PointType point) {
    Eigen::Vector3d point_vector(point.x, point.y, point.z);
    return(Eigen::Vector3i((point_vector - minimum) / resolution)); // we'll see if this works
}

/** Converts a voxel to a point. */
PointCloudVoxelGrid::PointType PointCloudVoxelGrid::voxel_to_point(voxel v) {

}
