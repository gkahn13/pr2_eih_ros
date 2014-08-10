#ifndef __PR2_SIM_CAMERA_H__
#define __PR2_SIM_CAMERA_H__

#include "arm.h"
#include "simulator.h"

#include "pr2_utils/geometry/geometry2d.h"
#include "pr2_utils/geometry/geometry3d.h"

#include <Eigen/Eigen>
using namespace Eigen;

namespace pr2_sim {

class Camera {
public:
	Camera(Arm *arm, Simulator *sim, double height, double width, double focal_length,
			double fx, double fy, double cx, double cy, double max_range);
	Camera(Arm *arm, Simulator *sim) : Camera(arm, sim, 480, 640, .01, 525, 525, 319.5, 239.5, 1.5) { }

	Matrix4d get_pose();
//	Matrix4d get_pose(const VectorJ& joints);

	Vector2d pixel_from_point(const Vector3d& point);
	geometry3d::Segment segment_through_pixel(const Vector2d& pixel);

	std::vector<geometry3d::Pyramid> truncated_view_frustum(const std::vector<geometry3d::Triangle>& triangles3d, bool include_truncated_pyramids=true);
	std::vector<geometry2d::Triangle> project_triangles(const std::vector<geometry3d::Triangle>& triangles3d);

	bool is_in_fov(const Vector3d& point, const std::vector<geometry3d::Pyramid>& truncated_frustum);
	double signed_distance(const Vector3d& point, const std::vector<geometry3d::Pyramid>& truncated_frustum);

	void plot(Vector3d color, std::string frame="base_link", bool fill=false, bool with_sides=true, double alpha=0.25);

private:
	Arm *arm;
	Simulator *sim;

	double height, width;
	double fx, fy, cx, cy;
	double focal_length, max_range;

	Matrix4d tool_to_camera;
	Matrix3d P;
};

}

#endif
