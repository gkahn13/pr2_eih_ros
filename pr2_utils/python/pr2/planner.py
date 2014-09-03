import openravepy as rave
import trajoptpy
import trajoptpy.kin_utils as ku
from trajoptpy.check_traj import traj_is_safe, traj_collisions
import trajoptpy.math_utils as mu
import json

import rospy
import roslib
roslib.load_manifest('tfx')
import tfx

import numpy as np

from pr2_sim import simulator
from utils import utils

import IPython

class Planner:
    def __init__(self, arm_name, sim=None, interact=False):
        """
        :param arm_name: "left" or "right"
        :param sim: OpenRave simulator (or create if None)
        :param interact: enable trajopt viewer
        """
        assert arm_name == 'left' or arm_name == 'right'
        
        self.sim = sim
        if self.sim is None:
            self.sim = simulator.Simulator()
            
        self.robot = self.sim.robot
        self.manip = self.sim.larm if arm_name == 'left' else self.sim.rarm
        
        if interact:
            trajoptpy.SetInteractive(True)
        
        self.tool_frame = '{0}_gripper_tool_frame'.format(arm_name[0])
        self.joint_indices = self.manip.GetArmIndices()
        
    def get_joint_trajectory(self, start_joints, target_pose, n_steps=20, ignore_orientation=False, link_name=None):
        """
        Calls trajopt to plan collision-free trajectory
        
        :param start_joints: list of initial joints
        :param target_pose: desired pose of tool_frame (tfx.pose)
        :param n_steps: trajopt discretization
        :return None if traj not collision free, else list of joint values
        """
        link_name = link_name if link_name is not None else self.tool_frame
        
        assert len(start_joints) == len(self.joint_indices)
        assert target_pose.frame.count('base_link') == 1
        self.sim.update()
        
        # set active manipulator and start joint positions
        self.robot.SetDOFValues(start_joints, self.joint_indices)
        
        # initialize trajopt inputs
        rave_pose = tfx.pose(self.sim.transform_from_to(target_pose.matrix, target_pose.frame, 'world'))
        quat = rave_pose.orientation
        xyz = rave_pose.position
        quat_target = [quat.w, quat.x, quat.y, quat.z]
        xyz_target = [xyz.x, xyz.y, xyz.z]
        rave_mat = rave.matrixFromPose(np.r_[quat_target, xyz_target])
        
        init_joint_target = None
#         init_joint_target = self.sim.ik_for_link(rave_pose.matrix, self.manip, link_name, 0)
#         if init_joint_target is not None:
#             init_joint_target = self._closer_joint_angles(init_joint_target, start_joints)
        
        request = self._get_trajopt_request(xyz_target, quat_target, n_steps,
                                            ignore_orientation=ignore_orientation, link_name=link_name, init_joint_target=init_joint_target)
        
        # convert dictionary into json-formatted string
        s = json.dumps(request) 
        # create object that stores optimization problem
        prob = trajoptpy.ConstructProblem(s, self.sim.env)
        # do optimization
        result = trajoptpy.OptimizeProblem(prob)
        
        prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
        #num_upsampled_collisions = len(traj_collisions(result.GetTraj(), self.robot, n=100))
        num_upsampled_collisions = self._num_collisions(result.GetTraj())
        print('Number of collisions: {0}'.format(num_upsampled_collisions))
        self.robot.SetDOFValues(start_joints, self.joint_indices)
        if num_upsampled_collisions > 0:
        #if not traj_is_safe(result.GetTraj()[:], self.robot): # Check that trajectory is collision free
            return None
        else:
            return result.GetTraj()
        
    @staticmethod
    def _closer_joint_angles(new_joints, curr_joints):
        for i in [2, 4, 6]:
            new_joints[i] = utils.closer_angle(new_joints[i], curr_joints[i])
        return new_joints
        
    def _num_collisions(self, joint_traj, up_samples=100):
        traj_up = mu.interp2d(np.linspace(0,1,up_samples), np.linspace(0,1,len(joint_traj)), joint_traj)
        
        manip_links = [l for l in self.robot.GetLinks() if l not in self.manip.GetIndependentLinks()]
        
        num_collisions = 0
        for (i,row) in enumerate(traj_up):
            self.robot.SetDOFValues(row, self.joint_indices)
            is_collision = max([self.sim.env.CheckCollision(l) for l in manip_links])
            if is_collision:
                num_collisions += 1
                
        return num_collisions

        
    def _get_trajopt_request(self, xyz_target, quat_target, n_steps, ignore_orientation=False, link_name=None, init_joint_target=None):
        """
        :param xyz_target: 3d list
        :param quat_target: [w,x,y,z]
        :param n_steps: trajopt discretization
        :param ignore_orientation
        :param init_joint_target: if not None, joint initialization of target_pose for trajopt
        :return trajopt json request
        """
        link_name = link_name if link_name is not None else self.tool_frame
        rot_coeffs = [1,1,1] if not ignore_orientation else [0,0,0]
        
        request = {
            "basic_info" : {
                "n_steps" : n_steps,
                "manip" : str(self.manip.GetName()), 
                "start_fixed" : True 
                },
            "costs" : [
                {
                    "type" : "joint_vel",
                    "params": {"coeffs" : [1]} 
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [20], # 20
                        "continuous": False,
                        "dist_pen" : [0.025] # .025 
                        }
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [20], # 20
                        "continuous" : True,
                        "dist_pen" : [0.025] # .025 
                        }
                    },
                {
                        "type" : "pose",
                        "name" : "target_pose",
                        "params" : {"xyz" : xyz_target, 
                                    "wxyz" : quat_target,
                                    "link": link_name,
                                    "rot_coeffs" : rot_coeffs,
                                    "pos_coeffs" : [0,0,0],
                                    }
                        },
                ],
            "constraints" : [
                # BEGIN pose_target
                {
                    "type" : "pose",
                    "name" : "target_pose",
                    "params" : {"xyz" : xyz_target, 
                                "wxyz" : quat_target,
                                "link": link_name,
                                "rot_coeffs" : [0,0,0],
                                "pos_coeffs" : [1,1,1]
                                }
                     
                    },
                #END pose_target
                ],
            }
        
        if init_joint_target is not None:
                request["init_info"] = {
                                         "type" : "straight_line",
                                         "endpoint" : list(init_joint_target),
                                         }
        else:
            request["init_info"] = {
                                     "type" : "stationary",
                                     }
        
        
        return request
        
 