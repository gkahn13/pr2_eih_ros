#ifndef __PR2_SIM_ARM_H__
#define __PR2_SIM_ARM_H__

#include <Eigen/Eigen>
using namespace Eigen;

namespace pr2_sim {

class Arm {
public:
	enum ArmType { left, right };
	enum Posture { untucked, tucked, up, side, mantis };
};

}

#endif
