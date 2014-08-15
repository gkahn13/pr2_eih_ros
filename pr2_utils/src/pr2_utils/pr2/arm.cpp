#include "pr2_utils/pr2/arm.h"

namespace pr2 {

/**
 *
 * Constructors
 *
 */

Arm::Arm(ArmType a) : arm_type(a){

	sim = new pr2_sim::Simulator(false, true);
	if (a == ArmType::right) {
		arm_sim = new pr2_sim::Arm(pr2_sim::Arm::ArmType::right, sim);
	} else {
		arm_sim = new pr2_sim::Arm(pr2_sim::Arm::ArmType::left, sim);
	}


	arm_name = (a == ArmType::right) ? "rightarm" : "leftarm";
	std::string arm_letter = (a == ArmType::right) ? "r" : "l";
	for(const std::string& joint_name_suffix : joint_name_suffixes) {
		joint_names.push_back(arm_letter+joint_name_suffix);
	}

	// setup ROS
	nh_ptr = new ros::NodeHandle();

	// listen to /joint_states
	received_joint_state = false;
	joint_state_sub = nh_ptr->subscribe("/joint_states", 1, &Arm::_joint_state_callback, this);

	ROS_INFO("Waiting for first joint state...");
	while (!received_joint_state && !ros::isShuttingDown()) {
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	}

	// create joint command action client
	joint_command_client = new JointCommandClient(arm_letter+"_arm_controller/joint_trajectory_action", true);
	// wait for action server to come up
	ROS_INFO("Waiting for joint command server...");
	if (!joint_command_client->waitForServer(ros::Duration(5.0))) {
		ROS_ERROR("Failed to establish connection with joint command server, exiting!");
		exit(0);
	}

}

/**
 *
 * Trajectory methods
 *
 */

/**
 * \param pose_traj list of poses in frame "base_link"
 * \param speed execution speed in meters/sec
 * \param block if true, waits until trajectory is completed
 */
void Arm::execute_pose_trajectory(const std::vector<Matrix4d>& pose_traj, double speed, bool block) {
	std::vector<VectorJ> joint_traj;
	for(const Matrix4d& pose : pose_traj) {
		VectorJ joints;
		if (!ik(pose, joints)) {
			ROS_WARN("Invalid pose:");
			std::cout << pose << "\n";
			ROS_WARN("Not executing trajectory");
			return;
		}
		joint_traj.push_back(joints);
	}

	execute_joint_trajectory(joint_traj, speed, block);
}

void Arm::execute_joint_trajectory(const std::vector<VectorJ>& joint_traj, double speed, bool block) {
	control_msgs::JointTrajectoryGoal goal;

	goal.trajectory.joint_names = joint_names;
	goal.trajectory.header.stamp = ros::Time::now();
	goal.trajectory.points.resize(joint_traj.size());

	sim->update();
	VectorJ curr_joints = get_joints();
	Vector3d start_position = fk(curr_joints).block<3,1>(0,3);
	ROS_INFO_STREAM("\nStart position: " << start_position.transpose());
	double time_from_start = 0;
	for(int i=0; i < joint_traj.size(); ++i) {
		const VectorJ& next_joints = joint_traj[i];

		Vector3d next_position = fk(next_joints).block<3,1>(0,3);
		Vector3d curr_position = fk(curr_joints).block<3,1>(0,3);
		double duration = (next_position - curr_position).norm() / speed;
		time_from_start += duration;
		goal.trajectory.points[i].time_from_start = ros::Duration(time_from_start);
		ROS_INFO_STREAM("Position " << i << ": " << next_position.transpose());

		int num_joints = next_joints.rows();
		goal.trajectory.points[i].positions.resize(num_joints);
		for(int j=0; j < num_joints; ++j) {
			goal.trajectory.points[i].positions[j] = next_joints[j];
		}

		curr_joints = next_joints;
	}

	joint_command_client->sendGoal(goal);

	if (block) {
		ROS_INFO_STREAM("Executing joint trajectory for " << time_from_start << " seconds");
		ros::Duration(time_from_start).sleep();
	}
}

/**
 *
 * Command methods
 *
 */

void Arm::go_to_posture(Arm::Posture posture, double speed, bool block) {
	VectorJ j;

	// joint values defined for left arm
	switch(posture) {
	case Posture::untucked:
		j << 0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0;
		break;
	case Posture::tucked:
		j << 0.06, 1.25, 1.79, -1.68, -1.73, -0.10, -0.09;
		break;
	case Posture::up:
		j << 0.33, -0.35,  2.59, -0.15,  0.59, -1.41, -0.27;
		break;
	case Posture::side:
		j << 1.832,  -0.332,   1.011,  -1.437,   1.1  ,  -2.106,  3.074;
		break;
	case Posture::mantis:
		j << 2.03018192, -0.05474993, 1.011, -1.47618716,  0.55995636, -1.42855926,  0.823;
		break;
	default:
		j << 0, 0, 0, 0, 0, 0, 0;
		break;
	}

	if (arm_type == ArmType::right) {
		j << -j(0), j(1), -j(2), j(3), -j(4), j(5), -j(6);
	}

	go_to_joints(j, speed, block);
}

void Arm::go_to_pose(const Matrix4d& pose, double speed, bool block) {
	execute_pose_trajectory(std::vector<Matrix4d>(1, pose), speed, block);
}

void Arm::go_to_joints(const VectorJ& joints, double speed, bool block) {
	execute_joint_trajectory(std::vector<VectorJ>(1, joints), speed, block);
}

void Arm::teleop() {
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

	double angle_step = 2.0*(M_PI/180);
	std::map<int,rave::Vector> delta_angle =
	{
			{'p', rave::Vector(angle_step, 0, 0)},
			{'o', rave::Vector(-angle_step, 0, 0)},
			{'k', rave::Vector(0, angle_step, 0)},
			{'l', rave::Vector(0, -angle_step, 0)},
			{'n', rave::Vector(0, 0, angle_step)},
			{'m', rave::Vector(0, 0, -angle_step)},
	};

	ROS_INFO_STREAM(arm_name << " teleop");

	char c;
	while ((c = pr2_utils::getch()) != 'q') {

		bool send_command = false;
		rave::Transform pose = pr2_sim::eigen_to_rave(get_pose());
		if (delta_position.count(c) > 0) {
			pose.trans += delta_position[c];
			send_command = true;
		} else if (delta_angle.count(c) > 0) {
			pose.rot = rave::geometry::quatFromAxisAngle(rave::geometry::axisAngleFromQuat(pose.rot) + delta_angle[c]);
			send_command = true;
		}

		if (send_command) {
			go_to_pose(pr2_sim::rave_to_eigen(pose));
		}

		ros::spinOnce();
	}

	ROS_INFO_STREAM(arm_name << " end teleop");
}

/**
 *
 * State info methods
 *
 */

Matrix4d Arm::get_pose() {
	sim->update();
	return fk(get_joints());
}

VectorJ Arm::get_joints() {
	received_joint_state = false;
	while(!received_joint_state && !ros::isShuttingDown()) {
		ros::spinOnce();
	}
	return current_joints;
}

Matrix4d Arm::fk(const VectorJ& joints) {
	return arm_sim->fk(joints);
}

bool Arm::ik(const Matrix4d& pose, VectorJ& joints) {
	return arm_sim->ik(pose, joints);
}

/**
 *
 * Callback
 *
 */

void Arm::_joint_state_callback(const sensor_msgs::JointStateConstPtr& joint_state) {
	VectorJ new_joints;

	for(int i=0; i < joint_names.size(); ++i) {
		for(int j=0; j < joint_state->name.size(); ++j) {
			if (joint_state->name[j] == joint_names[i]) {
				new_joints(i) = joint_state->position[j];
			}
		}
	}

	// mod 4 and 6 (roll joints) to -pi to +pi
	for(const int& index : {4, 6}) {
		new_joints(index) = fmod(new_joints(index) + M_PI, 2*M_PI) - M_PI;
	}

	current_joints = new_joints;
	received_joint_state = true;
}

}
