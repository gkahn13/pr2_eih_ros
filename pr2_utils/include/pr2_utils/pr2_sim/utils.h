#ifndef __PR2_SIM_UTILS__
#define __PR2_SIM_UTILS__

#include <unistd.h>
#include <termios.h>

#include <Eigen/Eigen>
using namespace Eigen;

#include <openrave-core.h>
namespace rave = OpenRAVE;

namespace pr2_sim {

inline Matrix4d rave_to_eigen(const rave::Transform& rt) {
	rave::TransformMatrix rtm(rt);

	Matrix4d m = Matrix4d::Identity();
	int index = 0;
	for(int i=0; i < 3; ++i) {
		for(int j=0; j < 4; ++j) {
			m(i,j) = rtm.m[index++];
		}
	}

	for(int i=0; i < 3; ++i) {
		m(i, 3) = rtm.trans[i];
	}

	return m;
}

inline rave::Transform eigen_to_rave(const Matrix4d& m) {
	rave::TransformMatrix rtm;

	int index = 0;
	for(int i=0; i < 3; ++i) {
		for(int j=0; j < 4; ++j) {
			rtm.m[index++] = m(i,j);
		}
	}

	for(int i=0; i < 3; ++i) {
		rtm.trans[i] = m(i,3);
	}

	rave::Transform rt(rtm);
	return rt;
}

inline Vector3d rave_to_eigen(const rave::Vector& v) {
	return Vector3d(v.x, v.y, v.z);
}

inline rave::Vector eigen_to_rave(const Vector3d& m) {
	return rave::Vector(m(0), m(1), m(2));
}

inline char getch() {
	char buf = 0;
	struct termios old = {0};
	if (tcgetattr(0, &old) < 0) {
		perror("tcsetattr()");
	}
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(0, TCSANOW, &old) < 0) {
		perror("tcsetattr ICANON");
	}
	if (read(0, &buf, 1) < 0) {
		perror ("read()");
	}
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0) {
		perror ("tcsetattr ~ICANON");
	}
	return (buf);
}

}

#endif
