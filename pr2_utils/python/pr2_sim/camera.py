import roslib
roslib.load_manifest('tfx')
import tfx

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import delaunay

import time
import random

from geometry import geometry2d, geometry3d
from pr2_sim import simulator, arm
from utils import utils

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
        y = np.dot(self.P, point_transform_cam[:3,3])
        
        pixel = np.array([y[1]/float(y[2]), y[0]/float(y[2])])
        return pixel
        
    def segment_through_pixel(self, pixel):
        """
        Returns segment from camera origin through pixel
        
        :param pixel: 2d list/np.array
        :return geometry3d.Segment (with endpoints in frame 'base_link')
        """
        # assert (0 <= pixel[0] <= self.height) and (0 <= pixel[1] <= self.width)
        
        pixel = np.array(pixel)
        pixel_centered = pixel - np.array([self.height/2., self.width/2.])
        
        pixel3d_centered_m = self.max_range*np.array([pixel_centered[1]/self.fx,
                                                      pixel_centered[0]/self.fy,
                                                      1])
        
        transform = self.get_pose().as_tf()
        p0 = transform.position.array
        p1 = (transform*tfx.pose(pixel3d_centered_m)).position.array        
        
        return geometry3d.Segment(p0, p1)
    
    #####################
    # calculate frustum #
    #####################
    
    def truncated_view_frustum(self, triangles3d, plot=False):
        """
        Truncates the view frustum against environment triangles
        
        :param triangles3d: list of geometry3d.Triangle with points in frame 'base_link'
        :param plot: if True, plots in 3d (OpenRAVE) and 2d (matplotlib)
        :return list of geometry3d.Pyramid
        """
        start_time = time.time()
        
        frustum = geometry3d.RectangularPyramid(self.get_pose().position.array,
                                                self.segment_through_pixel([0,self.width]).p1,
                                                self.segment_through_pixel([0,0]).p1,
                                                self.segment_through_pixel([self.height,0]).p1,
                                                self.segment_through_pixel([self.height,self.width]).p1)
        
        clipped_triangles3d = list()
        for tri3d in triangles3d:
            clipped_triangles3d += frustum.clip_triangle(tri3d)
            
        triangles2d = self.project_triangles(clipped_triangles3d)
        
        segments2d = set()
        for tri2d in triangles2d:
            segments2d.update(tri2d.segments)
        
        # avoid redundant points
        points2d = {geometry2d.Point([0,0]),
                    geometry2d.Point([self.height,0]),
                    geometry2d.Point([0,self.width]),
                    geometry2d.Point([self.height,self.width])}
        # add vertices
        for seg2d in segments2d:
            points2d.add(geometry2d.Point(seg2d.p0))
            points2d.add(geometry2d.Point(seg2d.p1))
        # add intersections
        segments2d_list = list(segments2d)
        for i in xrange(len(segments2d_list)-1):
            seg2d = segments2d_list[i]
            for j in xrange(i+1, len(segments2d_list)):
                other_seg2d = segments2d_list[j]
                intersection = seg2d.intersection(other_seg2d)
                if intersection is not None:
                    points2d.add(geometry2d.Point(intersection))
                    
#         print('len(points2d): {0}'.format(len(points2d)))
        partition_triangles2d = set()
        for pt2d in points2d:            
#             print('len(segments2d): {0}'.format(len(segments2d)))
            
            # find other points that don't cross anything in segments2d
            p = pt2d.p
            points_in_los = set()
            for other_pt2d in points2d:
                other_p = other_pt2d.p
                if pt2d != other_pt2d:
                    seg2d = geometry2d.Segment(p, other_p)
                    for check_seg2d in segments2d:
                        intersection = seg2d.intersection(check_seg2d)
                        if intersection is not None and not seg2d.is_endpoint(intersection):
                            break
                    else:
                        points_in_los.add(geometry2d.Point(other_p)) 
                                
            # sort segments by angle
            seg2d_compare = geometry2d.Segment([0,0], [-1,0])
            segments2d_in_los_sorted = sorted([geometry2d.Segment(p, other.p) for other in points_in_los], key=lambda seg: seg.angle(seg2d_compare))
                
            new_partition_triangles2d = set()
            for i in xrange(len(segments2d_in_los_sorted)-1):
                tri_seg2d = geometry2d.Segment(segments2d_in_los_sorted[i].p1, segments2d_in_los_sorted[i+1].p1)
                # check top segment of triangle against segments2d
                for check_seg2d in segments2d:
                    intersection = tri_seg2d.intersection(check_seg2d)
                    if intersection is not None and not tri_seg2d.is_endpoint(intersection):
                        break
                else:        
                    new_partition_triangles2d.add(geometry2d.Triangle(p, tri_seg2d.p0, tri_seg2d.p1))
            
            # update partition and new segments
            partition_triangles2d.update(new_partition_triangles2d)
            for tri2d in new_partition_triangles2d:
                segments2d.update(tri2d.segments)
                
            if sum([tri2d.area for tri2d in partition_triangles2d]) >= self.width*self.height:
                break
            
        # now that we have tesselated the projection into triangles
        # find out which 3d triangle each projection belongs to
        # by shooting out rays through the 2d triangles
        camera_position = self.get_pose().position.array
        pyramids3d = list()
        for tri2d in partition_triangles2d:
            tri2d_vertices = tri2d.vertices
            center2d = sum(tri2d_vertices)/3.
            
            center_seg3d = self.segment_through_pixel(center2d)
            vertices_seg3d = [self.segment_through_pixel(vertex) for vertex in tri2d_vertices]
            
            min_dist, min_tri3d = np.inf, None
            for tri3d in triangles3d:
                intersection = tri3d.intersection(center_seg3d)
                if intersection is not None:
                    dist = np.linalg.norm(intersection - camera_position)
                    if dist < min_dist:
                        min_dist = dist
                        min_tri3d = tri3d
                    
            if min_tri3d is not None:
                hyperplane3d = min_tri3d.hyperplane
                tri3d_intersections = [hyperplane3d.intersection(vertex_seg3d) for vertex_seg3d in vertices_seg3d]
                #assert len(filter(lambda x: x is None, tri3d_intersections)) == 0
                if len(filter(lambda x: x is None, tri3d_intersections)) == 0:
                    pyramids3d.append(geometry3d.Pyramid(camera_position, tri3d_intersections[0], tri3d_intersections[1], tri3d_intersections[2]))
            else:
                pyramids3d.append(geometry3d.Pyramid(camera_position, vertices_seg3d[0].p1, vertices_seg3d[1].p1, vertices_seg3d[2].p1))
            
                
        if plot:
            print('Total time: {0}'.format(time.time() - start_time))
            total_area = sum([tri2d.area for tri2d in partition_triangles2d])
            print('Total area (should be {0}): {1}'.format(self.width*self.height, total_area))
                
            self.plot(frame='base_link')
            for tri3d in triangles3d:
                tri3d.plot(self.sim, frame='base_link', fill=True, color=(0,0,1))
                
            for pyramid in pyramids3d:
                pyramid.plot(self.sim, frame='base_link', fill=True, with_sides=True, color=(0,1,0), alpha=0.1)
            
            plt.close(1)
            fig = plt.figure(1)
            axes = fig.add_subplot(111)
            
            for tri2d in triangles2d:
                for segment in tri2d.segments:
                    p0, p1 = segment.p0, segment.p1
                    p0_flip = [p0[1], self.height - p0[0]]
                    p1_flip = [p1[1], self.height - p1[0]]
                    axes.plot([p0_flip[0], p1_flip[0]], [p0_flip[1], p1_flip[1]], 'b--o', linewidth=2.0)
                 
            axes.plot([0, 0, self.width, self.width, 0], [0, self.height, self.height, 0, 0], 'b--o', linewidth=2.0)
            
            for pt2d in points2d:
                axes.plot(pt2d.p[1], self.height - pt2d.p[0], 'rx', markersize=10.0)
                
            plt.xlim((-10, self.width+10))
            plt.ylim((-10, self.height+10))
            
            colors = plt.cm.jet(np.linspace(0, 1, len(partition_triangles2d)))
            for i, tri2d in enumerate(partition_triangles2d):
                x = [p[1] for p in tri2d.vertices]
                y = [self.height - p[0] for p in tri2d.vertices]
                axes.fill(x, y, color=colors[i], edgecolor=(1,1,1))
            
            plt.show(block=False)
            
        return pyramids3d
            
        
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
                         
    ###################
    # signed distance #
    ###################
    
    def signed_distance(self, point, truncated_frustum):
        """
        Calculates signed-distance of point to truncated view frustum
        
        :param point: tfx.point/np.array
        :param truncated_frustum: list of geometry3d.Pyramid
        :return float signed-distance
        """
        point = tfx.point(point).array
        return min([pyramid3d.signed_distance(point) for pyramid3d in truncated_frustum])
    
    ##################
    # visualizations #
    ##################
        
    def plot(self, frame='world', fill=False, with_sides=True, color=(1,0,0), alpha=0.25):
        """
        :param frame: frame in which points are defined in
        :param with_sides: if True, plots side edges too
        :param color: (r,g,b) [0,1]
        """
        frustum = geometry3d.RectangularPyramid(self.get_pose().position.array,
                                                self.segment_through_pixel([0,self.width]).p1,
                                                self.segment_through_pixel([0,0]).p1,
                                                self.segment_through_pixel([self.height,0]).p1,
                                                self.segment_through_pixel([self.height,self.width]).p1)
        frustum.plot(self.sim, frame=frame, fill=fill, with_sides=with_sides, color=color, alpha=alpha)
            
    