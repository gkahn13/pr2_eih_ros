import numpy as np
import random

from pr2_sim import arm, simulator
from utils import utils

import IPython

def test_ik():
    sim = simulator.Simulator(view=True)
    larm = arm.Arm('left',sim=sim)
    larm.set_posture('mantis')
    rarm = arm.Arm('right',sim=sim)
    rarm.set_posture('mantis')

    pose = rarm.get_pose()
    position = pose.position
    
    while True:     
        point = pose.position + np.array([np.random.uniform(0,1), np.random.uniform(-.3,.3), np.random.uniform(-.5,-.2)])
        sim.plot_point(sim.transform_from_to(point.array, 'base_link', 'world'), color=(0,1,0), size=.1)
     
        joints = rarm.ik_lookat(position, point)
        if joints is not None:
            rarm.set_joints(joints)
        else:
            print('Joints is None')
 
        sim.plot_transform(sim.transform_from_to(np.array(rarm.get_pose().matrix), 'base_link', 'world'))
        raw_input()
        sim.clear_plots()

if __name__ == '__main__':
    test_ik()