#include "pr2_utils/pr2/arm.h"

int main(int argc, char** argv) {
	// Init the ROS node
	ros::init(argc, argv, "robot_driver");
	log4cxx::LoggerPtr my_logger =
			log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
	my_logger->setLevel(
			ros::console::g_level_lookup[ros::console::levels::Info]);

	pr2::Arm arm(pr2::Arm::ArmType::right);

	std::cout << "Going to mantis\n";
	arm.go_to_posture(pr2::Arm::Posture::mantis);
	arm.teleop();

	std::cout << "Current joints: " << arm.get_joints().transpose() << "\n";
	std::cout << "Current pose:\n" << arm.get_pose() << "\n";

	std::cout << "Press enter to exit\n";
	std::cin.ignore();
}
