"""
Test walking along the gradient of the signed-distance
of a point to the view frustum
with respect to the arm joints
"""

import numpy as np

from pr2_sim import arm, camera, simulator
from geometry import geometry3d
from utils import utils

def signed_distance(j, point, triangles3d, rarm, cam, plot=False):
    rarm.set_joints(j)
    truncated_frustum = cam.truncated_view_frustum(triangles3d, plot=plot)
    return cam.signed_distance(point, truncated_frustum)

def signed_distance_grad(j, point, triangles3d, rarm, cam, step=1e-5):
    grad = np.zeros(j.shape)
    
    j_p, j_m = np.array(j), np.array(j)
    for i in xrange(len(j)):
        j_p[i] = j[i] + step
        j_m[i] = j[i] - step
        
        sd_p = signed_distance(j_p, point, triangles3d, rarm, cam)
        sd_m = signed_distance(j_m, point, triangles3d, rarm, cam)
        
        grad[i] = (sd_p - sd_m) / (2*step)
        
        j_p[i] = j[i]
        j_m[i] = j[i]
        
    return grad

def test_gradient_sd():
    sim = simulator.Simulator(view=True)
    larm = arm.Arm('left',sim=sim)
    larm.set_posture('mantis')
    rarm = arm.Arm('right',sim=sim)
    rarm.set_posture('mantis')
    
    cam = camera.Camera(rarm, sim)
#     triangles3d = []
#     triangles3d = [geometry3d.Triangle([.7,0,.8],[.7,0,1.1],[.7,-.3,.7])]
#     triangles3d = [geometry3d.Triangle([.5,0,.5],[.8,0,.6],[.5,-.3,.9])]
#     triangles3d = [geometry3d.Triangle([np.random.uniform(.2,.5), np.random.uniform(-.5,0), np.random.uniform(.25,.75)],
#                                        [np.random.uniform(.2,.5), np.random.uniform(-.5,0), np.random.uniform(.25,.75)],
#                                        [np.random.uniform(.2,.5), np.random.uniform(-.5,0), np.random.uniform(.25,.75)]) for _ in xrange(3)]
    table_center = np.array([.2,.7,.5])
    triangles3d = [#geometry3d.Triangle(table_center, table_center+np.array([.5,-1.4,0]), table_center+np.array([.5,0,0])),
                   #geometry3d.Triangle(table_center, table_center+np.array([0,-1.4,0]), table_center+np.array([.5,-1.4,0])),
                   geometry3d.Triangle(table_center+np.array([.25,-.7,0]), table_center+np.array([.25,-.7,.2]), table_center+np.array([.25,-1.2,0])),
                   geometry3d.Triangle(table_center+np.array([.25,-1.2,0]), table_center+np.array([.25,-.7,.2]), table_center+np.array([.25,-1.2,.2]))]
    
#     point = np.array([0.5, 0.7, 0.66]) # sd: 0.115138180967
#     point = np.array([0.5, -2, 0.66])
    point = np.array([0.640, -0.170,  0.560]) # sd: 0.0189611173538
    j = rarm.get_joints()
    
    while True:
        sim.clear_plots()
        sim.plot_point(sim.transform_from_to(point, 'base_link', 'world'), size=.02, color=(0,0,1))
        cam.plot(frame='base_link')
        for tri3d in triangles3d:
            tri3d.plot(sim, frame='base_link', fill=True)
        
        sd = signed_distance(j, point, triangles3d, rarm, cam, plot=True)
        sd_grad = signed_distance_grad(j, point, triangles3d, rarm, cam)
        
        print('sd: {0}'.format(sd))
        print('sd_grad: {0}'.format(sd_grad))
        
        print('Press enter to continue (or "q" to exit)')
        char = utils.Getch.getch()
        if char == 'q':
            break
        
        j -= (np.pi/64.)*(sd_grad/np.linalg.norm(sd_grad))
        rarm.set_joints(j)
    

if __name__ == '__main__':
    test_gradient_sd()