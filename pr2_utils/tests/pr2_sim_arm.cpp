#include "pr2_utils/pr2_sim/arm.h"

int main() {
	pr2_sim::Simulator sim(true, false);
	pr2_sim::Arm arm(pr2_sim::Arm::right, &sim);

	arm.set_posture(pr2_sim::Arm::Posture::mantis);
	arm.teleop();

	std::cout << "Press enter to exit\n";
	std::cin.ignore();
}
