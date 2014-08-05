import roslib
roslib.load_manifest('tfx')
import tfx

import numpy as np
from matplotlib import pyplot as plt

import time

from geometry import geometry2d, geometry3d
from pr2_sim import simulator, arm

# height=640., width=480., focal_length=.01, fx=480.*2., fy=640.*2., cx=480./2. + 0.5, cy=640./2. + 0.5
wrist_to_hand = tfx.transform((-0.106925711716, -0.00652973239027, -0.0566985277547),
                              (0.5013894362349414, 0.46636457585894514, 0.5099879560929402, 0.5206006149209386))

class Camera:
    def __init__(self, arm, sim, tool_to_camera=None,
                 height=480., width=640., focal_length=.01,
                 fx=525., fy=525., cx=319.5, cy=239.5, max_range=1.5):
                 #fx=640.*2., fy=480.*2., cx=640./2.+0.5, cy=480./2.+0.5, max_range=1.5):
        self.arm = arm
        self.sim = sim
        self.tool_to_camera = tool_to_camera if tool_to_camera is not None else tfx.transform(wrist_to_hand)
        
        self.height = height
        self.width = width
        
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        
        self.P = np.array([[fx,  0, cx],
                           [ 0, fy, cy],
                           [ 0,  0,  1]])
        
        self.focal_length = focal_length # TODO: not sure if needed
        self.max_range = 1.5
        self.height_m = focal_length*(height/fy)
        self.width_m = focal_length*(width/fx)
        
    ##############
    # state info #
    ##############
                
    def get_pose(self):
        """
        :return tfx.pose from frame 'base_link'
        """
        return tfx.pose(self.sim.transform_from_to(self.tool_to_camera.matrix, self.arm.tool_frame, 'base_link'), frame='base_link')
        
    #########################
    # camera matrix methods #
    #########################
        
    def pixel_from_point(self, point):
        """
        Projects point onto image plane
        
        :param point: tfx.pose/tfx.point
        :return 2d np.array (even if outside of image plane)
        """
        assert point.frame == 'base_link'
        
        cam_pose_mat = np.array(self.get_pose().as_tf().matrix)
        point_pose_mat = np.array(point.as_pose().matrix)
        point_transform_cam = np.linalg.solve(cam_pose_mat, point_pose_mat)
        #point_transform_cam = np.array(self.get_pose().as_tf()*point.as_pose())
        y = np.dot(self.P, point_transform_cam[:3,3])
        
        pixel = np.array([y[1]/float(y[2]), y[0]/float(y[2])])
        return pixel
        
    def segment_through_pixel(self, pixel):
        """
        Returns segment from camera origin through pixel
        
        :param pixel: 2d list/np.array
        :return geometry3d.Segment (with endpoints in frame 'base_link')
        """
        assert (0 <= pixel[0] <= self.height) and (0 <= pixel[1] <= self.width)
        
        pixel = np.array(pixel)
        pixel_centered = pixel - np.array([self.height/2., self.width/2.])
        
        pixel3d_centered_m = np.array([self.focal_length*(pixel_centered[1]/self.fx),
                                       self.focal_length*(pixel_centered[0]/self.fy),
                                       self.focal_length])
        
        transform = self.get_pose().as_tf()
        p0 = transform.position.array
        p1_dir = ((transform*tfx.pose(pixel3d_centered_m)).position.array - p0)
        p1_dir /= np.linalg.norm(p1_dir)
        p1 = p0 + self.max_range*p1_dir
        
        return geometry3d.Segment(p0, p1)
    
    #####################
    # calculate frustum #
    #####################
        
    def project_triangles(self, triangles3d):
        """
        Projects 3d triangles onto image plane and returns 2d triangles
        
        :param triangles3d: list of geometry3d.Triangle with points in frame 'base_link'
        :return list of geometry2d.Triangle with points in frame 'base_link'
        """
        triangles2d = list()
        for triangle3d in triangles3d:
            a_proj = self.pixel_from_point(tfx.point(triangle3d.a, frame='base_link'))
            b_proj = self.pixel_from_point(tfx.point(triangle3d.b, frame='base_link'))
            c_proj = self.pixel_from_point(tfx.point(triangle3d.c, frame='base_link'))
            
            triangles2d.append(geometry2d.Triangle(a_proj, b_proj, c_proj))
            
        return triangles2d
    
    ##################
    # visualizations #
    ##################
        
    def plot(self, color=(1,0,0)):
        """
        :param color: (r,g,b) [0,1]
        """
        origin = self.get_pose().position.array
        side_segments = [self.segment_through_pixel([0., 0.]),
                    self.segment_through_pixel([self.height, 0.]),
                    self.segment_through_pixel([self.height, self.width]),
                    self.segment_through_pixel([0., self.width])]
        
        end_points = [segment.p1 for segment in side_segments+[side_segments[0]]]
        end_segments = [geometry3d.Segment(end_points[i], end_points[i+1]) for i in xrange(len(end_points)-1)]
        
        for segment in side_segments+end_segments:
            p0_world = self.sim.transform_from_to(tfx.pose(segment.p0).matrix, 'base_link', 'world')[:3,3]
            p1_world = self.sim.transform_from_to(tfx.pose(segment.p1).matrix, 'base_link', 'world')[:3,3]
            geometry3d.Segment(p0_world, p1_world).plot(self.sim, color=color)
            
            
###########
#  TESTS  #
###########

def test_project_triangles():
    sim = simulator.Simulator(view=True)
    rarm = arm.Arm('right',sim=sim)
    rarm.set_posture('mantis')
    
    cam = Camera(rarm, sim)
    #triangles3d = [geometry3d.Triangle([.7,0,.8],[.7,0,1.1],[.7,-.3,.7])]
    triangles3d = [geometry3d.Triangle(cam.segment_through_pixel([1,1]).p1,
                                       cam.segment_through_pixel([cam.height-1,cam.width-1]).p1,
                                       cam.segment_through_pixel([1,cam.width-1]).p1)]
    triangles2d = cam.project_triangles(triangles3d)
    
    for triangle3d in triangles3d:
        triangle3d.plot(sim)
    cam.plot()
    
    fig = plt.figure()
    axes = fig.add_subplot(111)
    
    for t in triangles2d:
        a, b, c = t.a, t.b, t.c
        a_flip = [a[1], cam.height - a[0]]
        b_flip = [b[1], cam.height - b[0]]
        c_flip = [c[1], cam.height - c[0]]
        geometry2d.Triangle(a_flip,b_flip,c_flip).plot(axes, color='b')
    
    plt.xlim((-1, cam.width+1))
    plt.ylim((-1, cam.height+1))
        
    plt.show(block=False)
        
    print('Press enter to exit')
    raw_input()
    

def test_camera_teleop():
    sim = simulator.Simulator(view=True)
    rarm = arm.Arm('right',sim=sim)
    rarm.set_posture('mantis')
    
    cam = Camera(rarm, sim)
    
    rarm.teleop()
    cam.plot(sim)
    
    print('Press enter to exit')
    raw_input()

if __name__ == '__main__':
    test_project_triangles()
    #test_camera_teleop()
    