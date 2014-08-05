import time
import numpy as np

import roslib
roslib.load_manifest('tfx')
import tfx
import tf.transformations as tft

import geometry2d
import utils

import IPython

epsilon = 1e-5

class Pyramid:
    def __init__(self, base, a, b, c):
        """
        A pyramid with orign base and points a,b,c
        """
        self.base = np.array(base)
        self.a = np.array(a)
        self.b = np.array(b)
        self.c = np.array(c)
        
    def is_inside(self, p):
        """
        :param p: 3d point as list or np.array
        :return True if p is inside the pyramid, else False
        """
        p = np.array(p)
        
        halfspaces = self.get_halfspaces()
        return np.min([h.contains(p) for h in halfspaces])    
    
    def get_halfspaces(self):
        """
        :return list of halfspaces representing outward-pointing faces
        """
        base, a, b, c = self.base, self.a, self.b, self.c

        origins = [(base+a+b)/3.,
                   (base+b+c)/3.,
                   (base+c+a)/3.,
                   (a+b+c)/3.]
    
        normals = [np.cross(a-base, b-base),
                   np.cross(b-base, c-base),
                   np.cross(c-base, a-base),
                   -np.cross(b-a, c-a)]
        normals = [n/np.linalg.norm(n) for n in normals]
        
        return [Halfspace(origin, normal) for origin, normal in zip(origins, normals)]
    
    def plot(self, sim, with_sides=True, color=(1,0,0)):
        """
        Plots edges of the pyramid
        
        :param sim: Simulator instance
        :param with_sides: if True, plots side edges too
        :param color: (r,g,b) [0,1]
        """
        base, a, b, c = self.base, self.a, self.b, self.c
        
        if with_sides:
            sim.plot_segment(base, a, color=color)
            sim.plot_segment(base, b, color=color)
            sim.plot_segment(base, c, color=color)
            
        sim.plot_segment(a, b, color=color)
        sim.plot_segment(b, c, color=color)
        sim.plot_segment(c, a, color=color)
    
class Triangle:
    def __init__(self, a, b, c):
        self.a, self.b, self.c = np.array(a), np.array(b), np.array(c)
        
    def align_with(self, target):
        """
        Aligns the normal of this triangle to target
        
        :param target: 3d list or np.array
        :return (rotated triangle, rotation matrix)
        """
        target = np.array(target)
        source = np.cross(self.b - self.a, self.c - self.a)
        source /= np.linalg.norm(source)
    
        rotation = np.eye(3)
        
        dot = np.dot(source, target)
        if not np.isnan(dot):
            angle = np.arccos(dot)
            if not np.isnan(angle):
                cross = np.cross(source, target)
                cross_norm = np.linalg.norm(cross)
                if not np.isnan(cross_norm) and not cross_norm < epsilon:
                    cross = cross / cross_norm
                    rotation = tft.rotation_matrix(angle, cross)[:3,:3]

        return (Triangle(np.dot(rotation, self.a),
                        np.dot(rotation, self.b),
                        np.dot(rotation, self.c)),
                rotation)
        
    def closest_point_to(self, p):
        """
        Find distance to point p
        by rotating and projecting
        then return that closest point unrotated
        
        :param p: 3d list or np.array
        :return 3d np.array of closest point
        """
        p = np.array(p)
        # align with z-axis so all triangle have same z-coord
        tri_rot, rot = self.align_with([0,0,1])
        tri_rot_z = tri_rot.a[-1]
        p_rot = np.dot(rot, p)
        
        p_2d = p_rot[:2]
        tri_2d = geometry2d.Triangle(tri_rot.a[:2], tri_rot.b[:2], tri_rot.c[:2])
        
        if tri_2d.is_inside(p_2d):
            # projects onto triangle, so return difference in z
            return np.dot(np.linalg.inv(rot), np.array(list(p_2d) + [tri_rot_z]))
        else:
            closest_pt_2d = tri_2d.closest_point_to(p_2d)
            
            closest_pt_3d = np.array(list(closest_pt_2d) + [tri_rot_z])
            
            return np.dot(np.linalg.inv(rot), closest_pt_3d)
        
    def distance_to(self, p):
        """
        Find distance to point p
        by rotating and projecting
        
        :param p: 3d list or np.array
        :return float distance
        """
        closest_pt = self.closest_point_to(p)
        return np.linalg.norm(p - closest_pt)
    
    def intersection(self, segment):
        """
        Determine point where line segment intersects this triangle
        - find intersection of segment with hyperplane
        - if intersection is in the triangle, return it
        
        :param segment: 3d line segment
        :return 3d np.array if intersection, else None
        """
        origin = (self.a+self.b+self.c)/3.
        normal = np.cross(self.a-self.b, self.a-self.c)
        hyperplane = Hyperplane(origin, normal)
        
        intersection = hyperplane.intersection(segment)
        if intersection is not None and np.linalg.norm(intersection - self.closest_point_to(intersection)) < epsilon:
            return intersection
        
        return None
        
    
    def area(self):
        """
        :return area of the triangle
        """
        tri_rot, rot = self.align_with([0,0,1])
        tri_2d = geometry2d.Triangle(tri_rot.a[:2], tri_rot.b[:2], tri_rot.c[:2])
        return tri_2d.area()
        
    def plot(self, sim, color=(1,0,0)):
        """
        :param sim: Simulator instance
        :param color: (r,g,b) [0,1]
        """
        sim.plot_segment(self.a, self.b, color)
        sim.plot_segment(self.b, self.c, color)
        sim.plot_segment(self.c, self.a, color)
    
class Segment:
    def __init__(self, p0, p1):
        self.p0, self.p1 = np.array(p0), np.array(p1)
        
    def closest_point_to(self, x):
        """
        min_{0<=t<=1} ||t*(p1-p0) + p0 - x||_{2}^{2}
        
        :param x: 3d list or np.array
        :return 3d np.array of closest point on segment to x
        """
        x = np.array(x)
        v = self.p1 - self.p0
        b = self.p0 - x
        
        t = -np.dot(v, b) / np.dot(v, v)
        if (0 <= t <= 1):
            closest = t*(self.p1 - self.p0) + self.p0
            return closest
        else:
            if np.linalg.norm(x - self.p0) < np.linalg.norm(x - self.p1):
                return self.p0
            else:
                return self.p1
            
    def intersection(self, other):
        """
        Finds intersection point with another segment
        :param other: Segment
        :return None if no intersection, else [x,y] of intersection 
        """
        p0_other, p1_other = other.p0, other.p1
        
        # w = p1 - p0
        # v = p1_other - p0_other
        # s*w + p0 = t*v + p_other
        
        w = self.p1 - self.p0
        v = p1_other - p0_other
        
        A = np.vstack((w,v)).T
        b = p0_other - self.p0
        
        if np.abs(np.linalg.det(A)) < epsilon:
            return None
        
        soln = np.linalg.solve(A, b)
        s, t = soln[0], -soln[1]
        
        intersection = s*w + self.p0
        
        if ((-epsilon <= s) and (s <= 1+epsilon) and (-epsilon <= t) and (t <= 1+epsilon)):
            return intersection
        else:
            return None
            
    def plot(self, sim, color=(1,0,0)):
        """
        :param axes: pyplot axes
        :param color: character or (r,g,b) [0,1]
        """
        sim.plot_segment(self.p0, self.p1, color=color)
        
    
class Halfspace:
    def __init__(self, origin, normal):
        self.origin = origin
        self.normal = normal
        
    def contains(self, x):
        """
        :param x: 3d point as list or np.array
        :return True if x forms acute angle with plane normal, else False
        """
        return np.dot(self.normal, np.array(x) - self.origin) >= epsilon
    
    def plot(self, sim, color=(0,0,1)):
        """
        Plots the normal
        
        :param sim: Simulator instance
        :param color: (r,g,b) [0,1]
        """
        o, n = self.origin, self.normal
        sim.plot_segment(o, o + .05*n, color=color)
    
class Hyperplane:
    def __init__(self, origin, normal):
        self.origin = origin
        self.normal = normal
        
    def intersection(self, segment):
        """
        Finds intersection with a line segment
        
        :segment segment: 2d line segment
        :return 2d np.array, or None if no intersection
        """
        p0, p1 = segment.p0, segment.p1
        
        # x = t*(p1 - p0) + p0
        # n'*(x - origin) = 0
        # combine to get
        # n'*(t*(p1-p0) + p0 - origin) = 0
        # solve for t
        
        v = p1 - p0
        w = p0 - self.origin
        t = -np.dot(self.normal, w)/np.dot(self.normal, v)
        
        if 0 <= t <= 1:
            return t*(p1-p0) + p0
        else:
            return None
        
        
#########
# TESTS #
#########
        
def test_align_with():
    t = Triangle([0,0,1.2], [0,1,1.2], [1,0,1.2])
    
    t_rot, rot = t.align_with([0,0,1])
    
    print('t_rot:\n{0}\n{1}\n{2}'.format(t_rot.a, t_rot.b, t_rot.c))
        
def test_distance_to():
    t = Triangle([0,0,0], [0,1,0], [1,0,0])
    
    p = [0, 0, 1]
    dist = t.distance_to(p)
    print('Distance should be 1')
    print('Computed distance is: {0}'.format(dist))
    
    p = [-1, 0, 1]
    dist = t.distance_to(p)
    print('Distance should be sqrt(2)')
    print('Computed distance is: {0}'.format(dist))
        
def test_distance_to_plot():
    from pr2_sim import simulator
    
    sim = simulator.Simulator(view=True)
    
    t = Triangle(np.random.rand(3), np.random.rand(3), np.random.rand(3))
    t.plot(sim)
    
    p = 2*np.random.rand(3)
    closest_pt = t.closest_point_to(p)
    
    sim.plot_point(p, color=(0,0,1))
    sim.plot_point(closest_pt)
    
    IPython.embed()
    
def test_beam_inside():
    from pr2_sim import simulator
    
    sim = simulator.Simulator(view=True)
    
    base = [.5,0,0]
    a = [.6, .1, .5]
    b = [.4, .1, .5]
    c = [.4, -.1, .5]
    
    pyramid = Pyramid(base, a, b, c)
    pyramid.plot(sim)
    
    halfspaces = pyramid.get_halfspaces()
    for h in halfspaces:
        h.plot(sim)
        
    for i in xrange(1000):
        p = [np.random.uniform(.4,.6),
             np.random.uniform(-.1,.1),
             np.random.uniform(0,.5)]
        is_inside = pyramid.is_inside(p)
        print('is_inside: {0}'.format(is_inside))
        sim.plot_point(p)
        if is_inside:
            raw_input()
        sim.clear_plots(1)
        
    
    IPython.embed()
         
if __name__ == '__main__':
    #test_align_with()
    #test_distance_to()
    #test_distance_to_plot()
    test_beam_inside()