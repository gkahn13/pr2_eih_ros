#ifndef POINTCLOUD_VOXEL_GRID_H_INCLUDED
#define POINTCLOUD_VOXEL_GRID_H_INCLUDED

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <math.h>


class PointCloudVoxelGrid
{
public:

    static const int NOT_VISITED = 0;
    static const int VISITED = 1;

    typedef Eigen::Vector3i voxel;
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> CloudType;

    PointCloudVoxelGrid(CloudType::Ptr cloud, double resolution);

    inline int get(int x, int y, int z) const
    {
        assert((x >= 0) && (x < x_size) && (y >= 0) && (y < y_size) && (z >= 0) && (z < z_size));
        return array(z + y*(z_size) + x*(y_size*z_size));
    }

    inline int get(voxel v) {
        return get(v(0), v(1), v(2));
    }

    int get(PointType point) {
        return get(point_to_voxel(point));
    }

    inline void set_all(int val)
    {
        for(int i=0; i < x_size*y_size*z_size; ++i)
        {
            array(i) = val;
        }
    }

    inline void set(int x, int y, int z, int val) {
        assert((x >= 0) && (x < x_size) && (y >= 0) && (y < y_size) && (z >= 0) && (z < z_size));
        array(z + y*(z_size) + x*(y_size*z_size)) = val;
    }

    inline void set(voxel v, int val) {
        set(v(0), v(1), v(2), val);
    }

    inline void set(PointType point, int val) {
        set(point_to_voxel(point), val);
    }

    inline void mark_as_seen(PointType point) {
        set(point, VISITED);
    }

    voxel point_to_voxel(PointType point);

    PointType voxel_to_point(voxel v);

    Eigen::Matrix<double, 6, 1> calculate_extremes(CloudType::Ptr cloud);

    void walk_cloud(CloudType::Ptr cloud);

    CloudType get_inverse_cloud();




private:
    int x_size, y_size, z_size;
    Eigen::VectorXi array;
    // these will be in the original space - i.e. meters
    Eigen::Vector3d minimum;
    Eigen::Vector3d maximum;
    double res;
};

#endif // POINTCLOUD_VOXEL_GRID_H_INCLUDED
