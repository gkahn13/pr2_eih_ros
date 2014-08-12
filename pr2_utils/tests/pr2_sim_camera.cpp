#include "pr2_utils/pr2_sim/camera.h"
#include "pr2_utils/pr2_sim/arm.h"
#include "pr2_utils/pr2_sim/simulator.h"
#include "pr2_utils/utils/timer.h"

void test_signed_distance() {
	pr2_sim::Simulator sim(true, false);
	pr2_sim::Arm arm(pr2_sim::Arm::right, &sim);
	arm.set_posture(pr2_sim::Arm::Posture::mantis);

	pr2_sim::Camera cam(&arm, &sim);
	//	std::vector<geometry3d::Triangle> triangles3d = {geometry3d::Triangle({.7,0,.8}, {.7,0,1.1}, {.7,-.3,.7})};
	//	std::vector<geometry3d::Triangle> triangles3d = {geometry3d::Triangle({.5,0,.5}, {.8,0,.6}, {.5,-.3,.9})};
	Vector3d table_center(.2,.7,.5);
	std::vector<geometry3d::Triangle> triangles3d = {
//			geometry3d::Triangle(table_center, table_center+Vector3d(.5,-1.4,0), table_center+Vector3d(.5,0,0)),
//			geometry3d::Triangle(table_center, table_center+Vector3d(0,-1.4,0), table_center+Vector3d(.5,-1.4,0)),
			geometry3d::Triangle(table_center+Vector3d(.25,-.7,0), table_center+Vector3d(.25,-.7,.2), table_center+Vector3d(.25,-1.2,0)),
			geometry3d::Triangle(table_center+Vector3d(.25,-1.2,0), table_center+Vector3d(.25,-.7,.2), table_center+Vector3d(.25,-1.2,.2))};

	double pos_step = .01;
	std::map<int,rave::Vector> delta_position =
	{
			{'a' , rave::Vector(0, pos_step, 0)},
			{'d' , rave::Vector(0, -pos_step, 0)},
			{'w' , rave::Vector(pos_step, 0, 0)},
			{'x' , rave::Vector(-pos_step, 0, 0)},
			{'+' , rave::Vector(0, 0, pos_step)},
			{'-' , rave::Vector(0, 0, -pos_step)},
	};

	rave::Vector point = pr2_sim::eigen_to_rave(table_center);

	char c;
	while ((c = pr2_sim::getch()) != 'q') {

		if (delta_position.count(c) > 0) {
			point += delta_position[c];

			pr2_utils::Timer timer;
			pr2_utils::Timer_tic(&timer);
			std::vector<geometry3d::TruncatedPyramid> truncated_frustum = cam.truncated_view_frustum(cam.get_pose(), triangles3d);
			double time = pr2_utils::Timer_toc(&timer);

			std::cout << "Truncated frustum time: " << time << " seconds\n";

			pr2_utils::Timer_tic(&timer);
			double sd = cam.signed_distance(pr2_sim::rave_to_eigen(point), truncated_frustum);
			time = pr2_utils::Timer_toc(&timer);

			std::cout << "Signed-distance time: " << time << " seconds\n";
			std::cout << "signed-distance: " << sd << "\n\n";

			sim.clear_plots();
			sim.plot_point(sim.transform_from_to(pr2_sim::rave_to_eigen(point), "base_link", "world"), {0,0,1}, .02);
			for(const geometry3d::Triangle& tri3d : triangles3d) {
				tri3d.plot(sim, "base_link", {0,0,1}, true, 0.25);
			}
			for(const geometry3d::TruncatedPyramid& pyramid : truncated_frustum) {
				pyramid.plot(sim, "base_link", {0,1,0}, true, true, 0.1);
			}

		}
	}
}

void test_truncated_view_frustum() {
	pr2_sim::Simulator sim(true, false);
	pr2_sim::Arm arm(pr2_sim::Arm::right, &sim);
	arm.set_posture(pr2_sim::Arm::Posture::mantis);

	pr2_sim::Camera cam(&arm, &sim);
	//	std::vector<geometry3d::Triangle> triangles3d = {geometry3d::Triangle({.7,0,.8}, {.7,0,1.1}, {.7,-.3,.7})};
	//	std::vector<geometry3d::Triangle> triangles3d = {geometry3d::Triangle({.5,0,.5}, {.8,0,.6}, {.5,-.3,.9})};
	Vector3d table_center(.2,.7,.5);
	std::vector<geometry3d::Triangle> triangles3d = {
			geometry3d::Triangle(table_center, table_center+Vector3d(.5,-1.4,0), table_center+Vector3d(.5,0,0)),
			geometry3d::Triangle(table_center, table_center+Vector3d(0,-1.4,0), table_center+Vector3d(.5,-1.4,0)),
			geometry3d::Triangle(table_center+Vector3d(.25,-.7,0), table_center+Vector3d(.25,-.7,.2), table_center+Vector3d(.25,-.9,0))};

	cam.plot(Vector3d(1,0,0));
	for(const geometry3d::Triangle& tri3d : triangles3d) {
		tri3d.plot(sim, "base_link", {0,0,1}, true, 0.25);
	}

	pr2_utils::Timer timer;
	pr2_utils::Timer_tic(&timer);
	std::vector<geometry3d::TruncatedPyramid> truncated_frustum = cam.truncated_view_frustum(cam.get_pose(), triangles3d);
	double time = pr2_utils::Timer_toc(&timer);

	std::cout << "Truncated frustum time: " << time << " seconds\n";

	std::cout << "Number of frustum pyramids: " << truncated_frustum.size() << "\n";
	for(const geometry3d::TruncatedPyramid& pyramid : truncated_frustum) {
		pyramid.plot(sim, "base_link", {0,1,0}, true, true, 0.1);
	}

	std::cout << "Press enter to exit\n";
	std::cin.ignore();
}

void test_pose() {
	pr2_sim::Simulator sim(true, false);
	pr2_sim::Arm arm(pr2_sim::Arm::right, &sim);
	arm.set_posture(pr2_sim::Arm::Posture::mantis);

	pr2_sim::Camera cam(&arm, &sim);

	Matrix4d arm_pose = arm.get_pose();
	Matrix4d cam_pose = cam.get_pose();

	std::cout << "arm_pose:\n" << arm_pose << "\n";
	std::cout << "cam_pose:\n" << cam_pose << "\n";

	sim.plot_transform(sim.transform_from_to(arm_pose, "base_link", "world"));
	sim.plot_transform(sim.transform_from_to(cam_pose, "base_link", "world"));

	std::cout << "Press enter to exit\n";
	std::cin.ignore();
}

int main() {
	test_signed_distance();
//	test_truncated_view_frustum();
//	test_pose();
}
