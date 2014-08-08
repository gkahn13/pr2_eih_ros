#ifndef __GEOMETRY3D_H__
#define __GEOMETRY3D_H__

#include "geometry2d.h"
#include "pr2_utils/pr2_sim/simulator.h"

#include <vector>

#include <Eigen/Eigen>
using namespace Eigen;

namespace geometry3d {

const double epsilon = 1e-5;

class Segment;

class Hyperplane;
class Halfspace;

class Point;
class Triangle;
class Pyramid;
class RectangularPyramid;

class Segment {
public:
	Vector3d p0, p1;

	Segment(const Vector3d& p0_pt, const Vector3d& p1_pt) : p0(p0_pt), p1(p1_pt) { }

	/**
	 * \brief Finds closest point on segment to x
	 *        min_{0<=t<=1} ||t*(p1-p0) + p0 - x||_{2}^{2}
	 */
	inline Vector3d closest_point_to(const Vector3d& x) const {
		Vector3d v = p1 - p0;
		Vector3d b = p0 - x;

		double t = -v.dot(b) / v.squaredNorm();
		if ((0 <= t) && (t <= 1)) {
			return t*(p1 - p0) + p0;
		} else {
			if ((x - p0).norm() < (x - p1).norm()) {
				return p0;
			} else {
				return p1;
			}
		}
	}

	/**
	 * \brief Finds intersection point with another segment
	 * \param intersection stores intersection (if found)
	 * \return True if intersection found
	 */
	inline bool intersection(const Segment& other, Vector3d& intersection) const {
		// w = p1 - p0
		// v = other.p1 - other.p0
		// s*w + p0 = t*v + other.p0

		Vector3d w = p1 - p0;
		Vector3d v = other.p1 - other.p0;

		Matrix<double,2,3> A;
		A << w(0), v(0),
				w(1), v(1),
				w(2), v(2);
		Vector3d b = other.p0 - p0;

		if (fabs(A.determinant()) < epsilon) {
			return false;
		}

		Vector2d soln = A.lu().solve(b);
		double s = soln(0), t = -soln(1);

		intersection = s*w + p0;

		if ((-epsilon <= s) && (s <= 1+epsilon) && (-epsilon <= t) && (t <= 1+epsilon)) {
			return true;
		}
		return false;

	}

	void plot(pr2_sim::Simulator& sim, std::string frame, Vector3d color) const {
		Vector3d p0_world = sim.transform_from_to(p0, frame, "world");
		Vector3d p1_world = sim.transform_from_to(p1, frame, "world");
		sim.plot_segment(p0_world, p1_world, color);
	}
};

class Hyperplane {
public:
	Hyperplane(const Vector3d& o, const Vector3d n) : origin(o), normal(n) { }

	/**
	 * \brief Finds intersection point with another segment
	 * \param intersection stores intersection (if found)
	 * \return true if intersection found
	 */
	inline bool intersection(const Segment& segment, Vector3d& intersection) const {
		// x = t*(p1 - p0) + p0
		// n'*(x - origin) = 0
		//combine to get
		// n'*(t*(p1-p0) + p0 - origin) = 0
		// solve for t

		Vector3d v = segment.p1 - segment.p0;
		Vector3d w = segment.p0 - origin;
		double t = -normal.dot(w) / normal.dot(v);

		if ((0 <= t) && (t <= 1)) {
			intersection = t*(segment.p1 - segment.p0) + segment.p0;
			return true;
		}

		return false;
	}

private:
	Vector3d origin, normal;
};


class Halfspace {
public:
	Halfspace(const Vector3d& o, const Vector3d n) : origin(o), normal(n) { }

	/**
	 * \brief True if x is in the halfspace
	 */
	inline bool contains(const Vector3d& x) const {
		return (normal.dot(x - origin) >= epsilon);
	}

	inline Hyperplane get_hyperplane() const {
		return Hyperplane(origin, normal);
	}

	void plot(pr2_sim::Simulator& sim, std::string frame, Vector3d color) const {
		Vector3d o = sim.transform_from_to(origin, frame, "world");
		Vector3d n = sim.transform_from_to(normal, frame, "world");
		sim.plot_segment(o, o + 0.5*n, color);
	}

private:
	Vector3d origin, normal;
};


class Triangle {
public:
	Vector3d a, b, c;

	Triangle() { a.setZero(); b.setZero(); c.setZero(); }
	Triangle(const Vector3d& a_pt, const Vector3d& b_pt, const Vector3d c_pt) : a(a_pt), b(b_pt), c(c_pt) { }

	/**
	 * \brief Finds the closest point on triangle to x
	 * 		  by rotating and projecting
	 * 		  then return the closest point unrotated
	 */
	inline Vector3d closest_point_to(const Vector3d& x) const {
		Vector3d align_target = {0,0,1};
		Matrix3d rotation = rotation_to_align_with(align_target);

		Vector3d a_rot, b_rot, c_rot, x_rot;
		a_rot = rotation*a;
		b_rot = rotation*b;
		c_rot = rotation*c;
		x_rot = rotation*x;

		double tri_rot_z = a_rot(2);

		Vector2d p_rot_2d;
		p_rot_2d << x_rot(0), x_rot(1);

		geometry2d::Triangle tri_2d = geometry2d::Triangle(a_rot.segment<2>(0), b_rot.segment<2>(0), c_rot.segment<2>(0));
		if (tri_2d.is_inside(p_rot_2d)) {
			// then distance is just the z of the rotated triangle, but with x,y from the point
			// then rotate back to original frame
			Vector3d p_rot_proj;
			p_rot_proj << p_rot_2d, tri_rot_z;
			return rotation.inverse()*p_rot_proj;
		} else {
			// find closest point on 2d triangle
			// connect back in 3d, then rotate back to original frame
			Vector2d closest_pt_2d;
			bool found_closest_pt = tri_2d.closest_point_to(p_rot_2d, closest_pt_2d);
			assert(found_closest_pt);
			Vector3d closest_pt_3d;
			closest_pt_3d << closest_pt_2d, tri_rot_z;
			return rotation.inverse()*closest_pt_3d;
		}
	}

	/**
	 * \brief Returns shortest distance to point x
	 */
	inline double distance_to(const Vector3d& x) const {
		return (closest_point_to(x) - x).norm();
	}

	/**
	 * \brief Finds intersection of segment with triangle by
	 *        finding intersection of segment with hyperplane and
	 *        if the intersection is in the triangle, return true
	 */
	inline bool intersection(const Segment& segment, Vector3d& intersection) const {
		bool found_intersection = get_hyperplane().intersection(segment, intersection);
		if (found_intersection && (intersection - closest_point_to(intersection)).norm() < epsilon) {
			return true;
		}

		return false;
	}

	inline double area() const {
		Matrix3d rotation = rotation_to_align_with({0,0,1});
		return geometry2d::triangle_area((rotation*a).segment<2>(0),
				(rotation*b).segment<2>(0),
				(rotation*c).segment<2>(0));
	}

	inline std::vector<Vector3d> get_vertices() const {
		return {a, b, c};
	}

	inline std::vector<Segment> get_segments() const {
		return {Segment(a, b), Segment(b, c), Segment(c, a)};
	}

	inline Hyperplane get_hyperplane() const {
		Vector3d origin = (a+b+c)/3.0;
		Vector3d normal = (a-b).cross(a-c);
		return Hyperplane(origin, normal);
	}

	void plot(pr2_sim::Simulator& sim, std::string frame, Vector3d color, bool fill=false, double alpha=0.25) const {
		Vector3d a_world = sim.transform_from_to(a, frame, "world");
		Vector3d b_world = sim.transform_from_to(b, frame, "world");
		Vector3d c_world = sim.transform_from_to(c, frame, "world");

		sim.plot_segment(a_world, b_world, color);
		sim.plot_segment(b_world, c_world, color);
		sim.plot_segment(c_world, a_world, color);

		if (fill) {
			sim.plot_triangle(a_world, b_world, c_world, color, alpha);
		}
	}

private:
	/**
	 * \brief Finds rotation to align the normal of this triangle to the target
	 * \param target align normal to this
	 * \return rotation rotation used to align
	 */
	Matrix3d rotation_to_align_with(const Vector3d& target) const {
		Vector3d source = (b-a).cross(c-a);
		source.normalize();

		Matrix3d rotation = Matrix3d::Identity();

		double dot = source.dot(target);
		if (!isnan(dot)) {
			double angle = acos(dot);
			if (!isnan(angle)) {
				Vector3d cross = source.cross(target);
				double cross_norm = cross.norm();
				if ((!isnan(cross_norm)) && (cross_norm > epsilon)) {
					cross /= cross_norm;
					rotation = Eigen::AngleAxis<double>(angle, cross).toRotationMatrix();
				}
			}
		}

		return rotation;
	}
};

class Pyramid {
	/**
	 * A pyramid with origin base and points a, b, c
	 */
public:
	Pyramid(const Vector3d& base_pt, const Vector3d& a_pt, const Vector3d& b_pt, const Vector3d& c_pt) : base(base_pt), a(a_pt), b(b_pt), c(c_pt) { }

	/**
	 * \brief Checks if point p is inside by comparing against intersection of halfspaces
	 */
	inline bool is_inside(const Vector3d& p) const {
		for(const Halfspace& halfspace : get_halfspaces()) {
			if (!halfspace.contains(p)) {
				return false;
			}
		}
		return true;
	}

	/**
	 * \brief Computes signed-distance by computing the sign via is_inside
	 *        and the distance using the min distance to the faces
	 */
	inline double signed_distance(const Vector3d& point) const {
		double sign = (is_inside(point)) ? -1 : 1;
		double dist = INFINITY;
		for(const Triangle& tri : get_faces()) {
			dist = std::min(dist, tri.distance_to(point));
		}

		return sign*dist;
	}

	inline std::vector<Halfspace> get_halfspaces() const {
		return {Halfspace((base+a+b)/3.0, (a-base).cross(b-base)),
			Halfspace((base+b+c)/3.0, (b-base).cross(c-base)),
			Halfspace((base+c+a)/3.0, (c-base).cross(a-base)),
			Halfspace((a+b+c)/3.0, -(b-a).cross(c-a))};
	}

	inline std::vector<Triangle> get_faces() const {
		return {Triangle(base, a, b),
			Triangle(base, b, c),
			Triangle(base, c, a),
			Triangle(a, b, c)};
	}

	void plot(pr2_sim::Simulator& sim, std::string frame, Vector3d color, bool with_sides=false, bool fill=false, double alpha=0.25) const {
		Vector3d base_world = sim.transform_from_to(base, frame, "world");
		Vector3d a_world = sim.transform_from_to(a, frame, "world");
		Vector3d b_world = sim.transform_from_to(b, frame, "world");
		Vector3d c_world = sim.transform_from_to(c, frame, "world");

		if (with_sides) {
			sim.plot_segment(base_world, a_world, color);
			sim.plot_segment(base_world, b_world, color);
			sim.plot_segment(base_world, c_world, color);
		}

		sim.plot_segment(a_world, b_world, color);
		sim.plot_segment(b_world, c_world, color);
		sim.plot_segment(c_world, a_world, color);

		if (fill) {
			if (with_sides) {
				sim.plot_triangle(base_world, a_world, b_world, color, alpha);
				sim.plot_triangle(base_world, b_world, c_world, color, alpha);
				sim.plot_triangle(base_world, c_world, a_world, color, alpha);
			}

			sim.plot_triangle(a_world, b_world, c_world, color, alpha);
		}
	}

private:
	Vector3d base, a, b, c;
};

class RectangularPyramid {
	/**
	 *
	 * A pyramid with origin base and points a,b,c,d arranged as
	 * b --- a
	 * |     |
	 * |     |
	 * c --- d
	 */
public:
	RectangularPyramid(const Vector3d& base_pt, const Vector3d& a_pt, const Vector3d& b_pt,
			const Vector3d& c_pt, const Vector3d& d_pt) : base(base_pt), a(a_pt), b(b_pt), c(c_pt), d(d_pt) { }

	/**
	 * \brief Checks if point p is inside by comparing against intersection of halfspaces
	 */
	inline bool is_inside(const Vector3d& p) const {
		for(const Halfspace& halfspace : get_halfspaces()) {
			if (!halfspace.contains(p)) {
				return false;
			}
		}
		return true;
	}

	/**
	 * \brief Clips triangle against the faces (http://www.cs.uu.nl/docs/vakken/gr/2011/Slides/08-pipeline2.pdf)
	 */
	inline std::vector<Triangle> clip_triangle(const Triangle& triangle) const {
		std::vector<Triangle> triangles = {triangle};
		for(const Halfspace& halfspace : get_halfspaces()) {
			Hyperplane hyperplane = halfspace.get_hyperplane();

			// clip all triangles against the halfspace
			std::vector<Triangle> new_triangles;
			for(const Triangle& triangle : triangles) {
				std::vector<Vector3d> intersections;
				for(const Segment& segment : triangle.get_segments()) {
					Vector3d intersection;
					if (hyperplane.intersection(segment, intersection)) {
						intersections.push_back(intersection);
					}
				}
				assert(intersections.size() == 0 || intersections.size() == 2);

				std::vector<Vector3d> inside_vertices;
				for(const Vector3d& vertex : triangle.get_vertices()) {
					if (halfspace.contains(vertex)) {
						inside_vertices.push_back(vertex);
					}
				}

				if (intersections.size() == 2) {
					assert(inside_vertices.size() == 1 || inside_vertices.size() == 2);
					if (inside_vertices.size() == 1) {
						// then intersections form new border of triangle
						new_triangles.push_back(Triangle(inside_vertices[0], intersections[0], intersections[1]));
					} else {
						// create two triangles
						new_triangles.push_back(Triangle(inside_vertices[0], intersections[0], intersections[1]));
						if ((inside_vertices[1] - intersections[0]).norm() < (inside_vertices[1] - intersections[1]).norm()) {
							new_triangles.push_back(Triangle(inside_vertices[1], intersections[0], inside_vertices[0]));
						} else {
							new_triangles.push_back(Triangle(inside_vertices[1], intersections[1], inside_vertices[0]));
						}
					}
				} else {
					// all/none of the triangle in halfspace
					assert(inside_vertices.size() == 0 || inside_vertices.size() == 3);
					if (inside_vertices.size() == 3) {
						new_triangles.push_back(triangle);
					}
				}
			}

			triangles = new_triangles;
		}

		return triangles;
	}

	inline std::vector<Halfspace> get_halfspaces() const {
		return {Halfspace((base+a+d)/3.0, (a-base).cross(d-base)),
			Halfspace((base+b+a)/3.0, (b-base).cross(a-base)),
			Halfspace((base+c+b)/3.0, (c-base).cross(b-base)),
			Halfspace((base+d+c)/3.0, (d-base).cross(c-base)),
			Halfspace((a+b+c+d)/4.0, (b-a).cross(d-a))};
	}

	void plot(pr2_sim::Simulator& sim, std::string frame, Vector3d color, bool with_sides=false, bool fill=false, double alpha=0.25) const {
		Vector3d base_world = sim.transform_from_to(base, frame, "world");
		Vector3d a_world = sim.transform_from_to(a, frame, "world");
		Vector3d b_world = sim.transform_from_to(b, frame, "world");
		Vector3d c_world = sim.transform_from_to(c, frame, "world");
		Vector3d d_world = sim.transform_from_to(d, frame, "world");

		if (with_sides) {
			sim.plot_segment(base_world, a_world, color);
			sim.plot_segment(base_world, b_world, color);
			sim.plot_segment(base_world, c_world, color);
			sim.plot_segment(base_world, d_world, color);
		}

		sim.plot_segment(a_world, b_world, color);
		sim.plot_segment(b_world, c_world, color);
		sim.plot_segment(c_world, d_world, color);
		sim.plot_segment(d_world, a_world, color);

		if (fill) {
			if (with_sides) {
				sim.plot_triangle(base_world, a_world, b_world, color, alpha);
				sim.plot_triangle(base_world, b_world, c_world, color, alpha);
				sim.plot_triangle(base_world, c_world, d_world, color, alpha);
				sim.plot_triangle(base_world, d_world, a_world, color, alpha);
			}

			sim.plot_triangle(a_world, b_world, c_world, color, alpha);
			sim.plot_triangle(a_world, c_world, d_world, color, alpha);
		}
	}

private:
	Vector3d base, a, b, c, d;
};



}

#endif
